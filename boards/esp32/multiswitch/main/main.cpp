#include <array>
#include <span>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "parameters.h"
#include "configuration.h"
#include "pwm.h"
#include "status_led.h"

namespace Multiswitch {
    static constexpr const char* tag = "multiswitch";

    class Application {
    public:
        void run() {
            ESP_ERROR_CHECK(nvs_flash_init());
            load();
            mParameters.apply();
            SoftwarePwm::init();

            gpio_config_t led{};
            led.pin_bit_mask = uint64_t{1} << Board::led;
            led.mode = GPIO_MODE_OUTPUT;
            ESP_ERROR_CHECK(gpio_config(&led));
            gpio_config_t button{};
            button.pin_bit_mask = uint64_t{1} << Board::button;
            button.mode = GPIO_MODE_INPUT;
            button.pull_up_en = GPIO_PULLUP_ENABLE;
            ESP_ERROR_CHECK(gpio_config(&button));

            uart_config_t uart{};
            uart.baud_rate = Crsf::baudrate;
            uart.data_bits = UART_DATA_8_BITS;
            uart.parity = UART_PARITY_DISABLE;
            uart.stop_bits = UART_STOP_BITS_1;
            uart.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
            uart.source_clk = UART_SCLK_DEFAULT;
            ESP_ERROR_CHECK(uart_param_config(port, &uart));
            ESP_ERROR_CHECK(uart_set_pin(port, Board::crsfTx, Board::crsfRx,
                                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
            ESP_ERROR_CHECK(uart_driver_install(port, 16'384, 2'048, 0, nullptr, 0));
            ESP_LOGI(tag, "WeAct ESP32-C3, CRSF RX=%d TX=%d, eight software PWM outputs",
                     Board::crsfRx, Board::crsfTx);
            mNextCheck = esp_timer_get_time() + 300'000;
            while (true) {
                const uint64_t now = esp_timer_get_time();
                std::array<uint8_t, 256> bytes{};
                const int count = uart_read_bytes(port, bytes.data(), bytes.size(), 0);
                if (count < 0) {
                    ESP_LOGE(tag, "UART read failed");
                    ESP_ERROR_CHECK(ESP_FAIL);
                }
                for (int i = 0; i < count; ++i) {
                    mParser.process(bytes[i], now, [this, now](const auto frame) { receive(frame, now); });
                }
                periodic(now);
                if (count < int(bytes.size())) vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

    private:
        static constexpr uart_port_t port = UART_NUM_1;

        void load() {
            nvs_handle_t handle;
            const auto open = nvs_open("multiswitch", NVS_READONLY, &handle);
            if (open == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGI(tag, "No saved configuration; using defaults");
                return;
            }
            ESP_ERROR_CHECK(open);
            size_t size = sizeof(mConfig);
            auto result = nvs_get_blob(handle, "config-v4", &mConfig, &size);
            bool valid = result == ESP_OK && size == sizeof(mConfig) && mParameters.valid();
            bool migrated = false;
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                std::array<uint8_t, sizeof(MasterSlaveConfiguration)> legacy{};
                size = legacy.size();
                result = nvs_get_blob(handle, "config-v3", legacy.data(), &size);
                valid = result == ESP_OK && migrateMasterSlave({legacy.data(), size}, mConfig) && mParameters.valid();
                migrated = valid;
            }
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                std::array<uint8_t, sizeof(VirtualPatternConfiguration)> legacy{};
                size = legacy.size();
                result = nvs_get_blob(handle, "config-v2", legacy.data(), &size);
                valid = result == ESP_OK && migrateVirtualPatterns({legacy.data(), size}, mConfig) && mParameters.valid();
                migrated = valid;
            }
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                std::array<uint8_t, sizeof(LegacyConfiguration)> legacy{};
                size = legacy.size();
                result = nvs_get_blob(handle, "config-v1", legacy.data(), &size);
                valid = result == ESP_OK && migrateLegacy({legacy.data(), size}, mConfig) && mParameters.valid();
                migrated = valid;
            }
            nvs_close(handle);
            if (result == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(tag, "Configuration key missing; using defaults");
                mConfig = EEProm{};
                return;
            }
            if (result == ESP_ERR_NVS_INVALID_LENGTH || (result == ESP_OK && !valid)) {
                ESP_LOGW(tag, "Incompatible/invalid configuration; using defaults");
                mConfig = EEProm{};
                return;
            }
            ESP_ERROR_CHECK(result);
            if (migrated) {
                mSaveAt = esp_timer_get_time() + 500'000;
                ESP_LOGI(tag, "Configuration upgraded to four Morse text fields");
            }
        }

        void save() {
            nvs_handle_t handle;
            ESP_ERROR_CHECK(nvs_open("multiswitch", NVS_READWRITE, &handle));
            ESP_ERROR_CHECK(nvs_set_blob(handle, "config-v4", &mConfig, sizeof(mConfig)));
            ESP_ERROR_CHECK(nvs_commit(handle));
            nvs_close(handle);
            mSaveAt = 0;
            ESP_LOGI(tag, "Configuration saved");
        }

        void send(const std::byte type, const std::span<const uint8_t> payload) {
            std::array<uint8_t, Crsf::maxMessageSize> frame{};
            if (payload.size() > frame.size() - 4) {
                ESP_LOGE(tag, "CRSF reply too large: %u", unsigned(payload.size()));
                return;
            }
            frame[0] = uint8_t(Crsf::Address::StartByte);
            frame[1] = payload.size() + 2;
            frame[2] = uint8_t(type);
            std::copy(payload.begin(), payload.end(), frame.begin() + 3);
            CRC8 crc;
            for (size_t i = 2; i < payload.size() + 3; ++i) crc += frame[i];
            frame[payload.size() + 3] = crc;
            const size_t size = payload.size() + 4;
            if (uart_write_bytes(port, frame.data(), size) != int(size)) {
                ESP_LOGE(tag, "UART write failed");
                ESP_ERROR_CHECK(ESP_FAIL);
            }
        }

        void deviceInfo(const uint8_t destination) {
            ByteBuffer<60> payload;
            payload.push_back(destination);
            payload.push_back(mConfig.crsf_address);
            etl::push_back_ntbs(mParameters.name(), payload);
            etl::serializeBE(uint32_t{1234}, payload);
            etl::serializeBE(Board::hardwareVersion, payload);
            etl::serializeBE(Board::softwareVersion, payload);
            payload.push_back(mParameters.size() - 1);
            payload.push_back(0);
            send(Crsf::Type::Info, {payload.begin(), payload.size()});
        }

        void status() {
            ByteBuffer<7> payload;
            payload.push_back(Crsf::Address::Handset);
            payload.push_back(mConfig.crsf_address);
            payload.push_back(Crsf::PassThru::SubType::Switch);
            etl::serializeBE(Crsf::PassThru::AppId::Status, payload);
            payload.push_back(mConfig.addresses[0]);
            payload.push_back(0); // No external status inputs on this pin mapping.
            send(Crsf::Type::PassThru, {payload.begin(), payload.size()});
        }

        void parameterInfo(const uint8_t destination, const uint8_t index, const uint8_t chunk) {
            ESP_LOGD(tag, "Parameter read: src=0x%02x index=%u chunk=%u", destination, index, chunk);
            ByteBuffer<58> payload;
            const auto result = mParameters.read(destination, index, chunk, payload);
            if (result == Parameters::ReadResult::InvalidIndex) {
                ESP_LOGW(tag, "Invalid parameter index: %u", index);
                return;
            }
            if (result == Parameters::ReadResult::InvalidChunk) {
                ESP_LOGW(tag, "Invalid parameter chunk: %u", chunk);
                return;
            }
            send(Crsf::Type::ParamEntry, {payload.begin(), payload.size()});
        }

        void resetCommand(const uint8_t destination, const uint8_t index, const uint8_t step,
                          const uint64_t now) {
            if (step > uint8_t(Crsf::Lua::CmdStep::Query)) {
                ESP_LOGW(tag, "Invalid reset command step: %u", step);
                return;
            }
            const std::array<uint8_t, 1> data{step};
            if (mParameters.write(index, data) == Parameters::Result::Reset) {
                mSaveAt = now + 500'000;
                mConfigSessionUntil = 0;
                ESP_LOGI(tag, "Factory defaults restored");
            }
            ByteBuffer<58> payload;
            payload.push_back(destination);
            payload.push_back(mConfig.crsf_address);
            payload.push_back(index);
            payload.push_back(0);
            payload.push_back(mParameters.parameter(index).parent);
            payload.push_back(uint8_t(Parameters::Type::Command));
            etl::push_back_ntbs("Reset all", payload);
            payload.push_back(step == uint8_t(Crsf::Lua::CmdStep::Click) ?
                                  uint8_t(Crsf::Lua::CmdStep::AskConfirm) : uint8_t(Crsf::Lua::CmdStep::Idle));
            payload.push_back(100);
            etl::push_back_ntbs("Reset all?", payload);
            send(Crsf::Type::ParamEntry, {payload.begin(), payload.size()});
        }

        void receive(const std::span<const uint8_t> frame, const uint64_t now) {
            const auto type = std::byte{frame[2]};
            if (type == Crsf::Type::Channels || type == Crsf::Type::Link) {
                mLastSignal = now;
                mSawSignal = true;
                if (type == Crsf::Type::Channels) ++mChannels;
                else ++mLinks;
                return;
            }
            if (type == Crsf::Type::Command) {
                const bool accepted = mSwitches.command(frame, now,
                    [this](const uint32_t speed) { setBaud(speed); });
                if (accepted) {
                    mLastSignal = now;
                    mSawSignal = true;
                    ++mCommands;
                }
                else {
                    ESP_LOGD(tag, "Ignored unsupported/invalid switch command");
                }
                return;
            }
            if (frame.size() < 6) return;
            const uint8_t destination = frame[3];
            if (destination != mConfig.crsf_address &&
                !(type == Crsf::Type::Ping && destination == uint8_t(Crsf::Address::Broadcast))) return;
            if (type == Crsf::Type::Ping) {
                mLastSignal = now;
                mSawSignal = true;
                ++mPings;
                mTelemetrySuppressedUntil = now + 30'000'000;
                if (replies(now)) deviceInfo(frame[4]);
                return;
            }
            if (!replies(now)) return;
            if (type == Crsf::Type::ParamRead && frame.size() >= 8) {
                mConfigSessionUntil = now + 30'000'000;
                mTelemetrySuppressedUntil = now + 30'000'000;
                parameterInfo(frame[4], frame[5], frame[6]);
            }
            else if (type == Crsf::Type::ParamWrite && frame.size() >= 8) {
                mConfigSessionUntil = now + 30'000'000;
                mTelemetrySuppressedUntil = now + 30'000'000;
                if (mParameters.isReset(frame[5])) {
                    resetCommand(frame[4], frame[5], frame[6], now);
                    return;
                }
                const auto result = mParameters.write(frame[5], frame.subspan(6, frame.size() - 7), now);
                if (result == Parameters::Result::Changed) mSaveAt = now + 500'000;
                else if (result == Parameters::Result::Invalid) ESP_LOGW(tag, "Invalid parameter write: %u", frame[5]);
            }
        }

        bool replies(const uint64_t now) const {
            return mConfig.telemetry || now < mConfigSessionUntil;
        }

        void setBaud(const uint32_t speed) {
            ESP_ERROR_CHECK(uart_wait_tx_done(port, pdMS_TO_TICKS(100)));
            ESP_ERROR_CHECK(uart_set_baudrate(port, speed));
            ESP_ERROR_CHECK(uart_flush_input(port));
            mParser.reset();
            ESP_LOGI(tag, "CRSF baud rate: %lu", static_cast<unsigned long>(speed));
        }

        void failsafe() {
            mSwitches.failsafe();
        }

        void periodic(const uint64_t now) {
            if (mConnected && now - mLastSignal > 1'000'000) {
                mConnected = false;
                failsafe();
                ESP_LOGW(tag, "CRSF connection lost; failsafe mode %u", mConfig.failsafe_mode);
            }
            if (now >= mNextCheck) {
                mNextCheck = now + 300'000;
                if (mChannels > 10 || mLinks > 3 || mPings > 0 || mCommands > 0) {
                    if (!mConnected) ESP_LOGI(tag, "CRSF connected");
                    mConnected = true;
                }
                else if (!mConnected && (!mSawSignal || now - mLastSignal > 1'000'000)) {
                    mBaudIndex = (mBaudIndex + 1) % Crsf::baudrates.size();
                    setBaud(Crsf::baudrates[mBaudIndex]);
                }
                mChannels = mLinks = mPings = mCommands = 0;
            }

            const auto duties = mSwitches.periodic(now,
                [this](const auto payload) { send(Crsf::Type::Command, payload); });
            SoftwarePwm::update(duties, mConfig.pwm1);

            if (!gpio_get_level(gpio_num_t(Board::button))) {
                if (!mButtonSince) mButtonSince = now;
                if (!mButtonHandled && now - mButtonSince >= 1'000'000) {
                    mButtonHandled = true;
                    mConfigSessionUntil = now + 30'000'000;
                    ESP_LOGI(tag, "Configuration replies enabled for 30 seconds");
                }
            }
            else {
                mButtonSince = 0;
                mButtonHandled = false;
            }
            const bool on = mStatusLed.value(now, mConnected, mConfig.telemetry != 0, replies(now));
            ESP_ERROR_CHECK(gpio_set_level(gpio_num_t(Board::led), on != Board::ledActiveLow));
            if (mConnected && replies(now) && now >= mTelemetrySuppressedUntil && now >= mNextTelemetry) {
                mNextTelemetry = now + 500'000;
                status();
            }
            if (mSaveAt && now >= mSaveAt) save();
        }

        EEProm mConfig{};
        std::array<Output, 8> mOutputs;
        Switches mSwitches{mConfig, mOutputs};
        Parameters mParameters{mConfig, mOutputs, mSwitches};
        Parser mParser;
        StatusLed mStatusLed;
        uint64_t mNextCheck = 0;
        uint64_t mLastSignal = 0;
        uint64_t mSaveAt = 0;
        uint64_t mConfigSessionUntil = 0;
        uint64_t mTelemetrySuppressedUntil = 0;
        uint64_t mNextTelemetry = 0;
        uint64_t mButtonSince = 0;
        size_t mBaudIndex = 1;
        uint32_t mChannels = 0;
        uint32_t mLinks = 0;
        uint32_t mPings = 0;
        uint32_t mCommands = 0;
        bool mConnected = false;
        bool mSawSignal = false;
        bool mButtonHandled = false;
    };
}

extern "C" void app_main() {
    static Multiswitch::Application application;
    if (xTaskCreate([](void*) { application.run(); }, "multiswitch", 8'192, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE("multiswitch", "Could not create application task");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}
