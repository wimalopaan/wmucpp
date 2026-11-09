// SPDX-License-Identifier: GPL-3.0-or-later
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "relay.h"

namespace Relay {
    static constexpr const char* tag = "relay";

    class Application {
    public:
        void run() {
            ESP_ERROR_CHECK(nvs_flash_init());
            load();
            gpio_config_t led{};
            led.pin_bit_mask = uint64_t{1} << Board::led;
            led.mode = GPIO_MODE_OUTPUT;
            ESP_ERROR_CHECK(gpio_config(&led));
            ESP_ERROR_CHECK(gpio_set_level(gpio_num_t(Board::led), Board::ledActiveLow));
            initUart(inputPort, Board::inputTx, Board::inputRx, Crsf::baudrate, mInputEvents);
            initUart(modulePort, Board::relayTx, Board::relayRx,
                     Crsf::baudratesHandset[mConfig.baudrate], mModuleEvents);
            configure();
            mController.parameters().resetReason(unsigned(esp_reset_reason()));
            mController.start(esp_timer_get_time());
            ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));
            ESP_LOGI(tag, "WeAct ESP32-C3 relay: input RX=%d TX=%d, module RX=%d TX=%d",
                     Board::inputRx, Board::inputTx, Board::relayRx, Board::relayTx);
            auto previous = mController.state();
            while (true) {
                events(mInputEvents, false);
                events(mModuleEvents, true);
                read(inputPort, false);
                read(modulePort, true);
                const uint64_t now = esp_timer_get_time();
                mController.periodic(now);
                if (previous != mController.state()) {
                    previous = mController.state();
                    ESP_LOGI(tag, "Connection state=%u, channel output=%s",
                             unsigned(previous), mController.enabled() ? "on" : "off");
                }
                updateLed(now);
                if (mSaveAt && now >= mSaveAt) save();
                ESP_ERROR_CHECK(esp_task_wdt_reset());
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        void input(const std::span<const uint8_t> frame) { write(inputPort, frame); }
        void module(const std::span<const uint8_t> frame) {
            write(modulePort, frame);
            if (mConfig.half_duplex) {
                ESP_ERROR_CHECK(uart_wait_tx_done(modulePort, pdMS_TO_TICKS(100)));
            }
        }
        void error(const char* message) { ESP_LOGW(tag, "%s", message); }
        void changed(const uint64_t now) { mSaveAt = now + 500'000; }

        void configure() {
            ESP_ERROR_CHECK(uart_wait_tx_done(modulePort, pdMS_TO_TICKS(100)));
            ESP_ERROR_CHECK(uart_set_baudrate(modulePort, Crsf::baudratesHandset[mConfig.baudrate]));
            ESP_ERROR_CHECK(uart_set_mode(modulePort,
                mConfig.half_duplex ? UART_MODE_RS485_HALF_DUPLEX : UART_MODE_UART));
            if (mConfig.half_duplex) {
                ESP_ERROR_CHECK(gpio_od_enable(gpio_num_t(Board::relayTx)));
                ESP_ERROR_CHECK(gpio_pullup_en(gpio_num_t(Board::relayTx)));
            }
            ESP_ERROR_CHECK(uart_set_pin(modulePort, Board::relayTx,
                mConfig.half_duplex ? Board::relayTx : Board::relayRx,
                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
            ESP_ERROR_CHECK(gpio_set_direction(gpio_num_t(Board::relayTx),
                mConfig.half_duplex ? GPIO_MODE_INPUT_OUTPUT_OD : GPIO_MODE_INPUT_OUTPUT));
            ESP_ERROR_CHECK(uart_flush_input(modulePort));
            ESP_LOGI(tag, "Module baud=%lu, %s",
                     static_cast<unsigned long>(Crsf::baudratesHandset[mConfig.baudrate]),
                     mConfig.half_duplex ? "single-wire/open-drain" : "full-duplex");
        }

    private:
        static constexpr uart_port_t inputPort = UART_NUM_0;
        static constexpr uart_port_t modulePort = UART_NUM_1;

        static void initUart(const uart_port_t port, const int tx, const int rx,
                             const uint32_t baud, QueueHandle_t& queue) {
            uart_config_t config{};
            config.baud_rate = baud;
            config.data_bits = UART_DATA_8_BITS;
            config.parity = UART_PARITY_DISABLE;
            config.stop_bits = UART_STOP_BITS_1;
            config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
            config.source_clk = UART_SCLK_DEFAULT;
            ESP_ERROR_CHECK(uart_param_config(port, &config));
            ESP_ERROR_CHECK(uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
            ESP_ERROR_CHECK(uart_driver_install(port, 16'384, 2'048, 32, &queue, 0));
            ESP_ERROR_CHECK(uart_set_rx_timeout(port, 2));
            ESP_ERROR_CHECK(gpio_pullup_en(gpio_num_t(rx)));
        }

        static void write(const uart_port_t port, const std::span<const uint8_t> frame) {
            if (uart_write_bytes(port, frame.data(), frame.size()) != int(frame.size())) {
                ESP_LOGE(tag, "UART %u write failed", unsigned(port));
                ESP_ERROR_CHECK(ESP_FAIL);
            }
        }

        void read(const uart_port_t port, const bool module) {
            std::array<uint8_t, 256> bytes{};
            const int count = uart_read_bytes(port, bytes.data(), bytes.size(), 0);
            if (count < 0) {
                ESP_LOGE(tag, "UART %u read failed", unsigned(port));
                ESP_ERROR_CHECK(ESP_FAIL);
            }
            const std::span<const uint8_t> data{bytes.data(), size_t(count)};
            const uint64_t now = esp_timer_get_time();
            if (module) mController.module(data, now);
            else mController.input(data, now);
        }

        void events(const QueueHandle_t queue, const bool module) {
            uart_event_t event{};
            while (xQueueReceive(queue, &event, 0) == pdTRUE) {
                if (event.type == UART_FRAME_ERR || event.type == UART_PARITY_ERR) {
                    if (module) mController.moduleFrameError();
                    mController.resetParser(module);
                    ESP_LOGW(tag, "UART %s framing/parity error", module ? "module" : "input");
                }
                else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                    ESP_LOGE(tag, "UART %s receive overflow", module ? "module" : "input");
                    ESP_ERROR_CHECK(uart_flush_input(module ? modulePort : inputPort));
                    mController.resetParser(module);
                }
            }
        }

        void load() {
            nvs_handle_t handle;
            const auto open = nvs_open("relay", NVS_READONLY, &handle);
            if (open == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGI(tag, "No saved configuration; using defaults");
                return;
            }
            ESP_ERROR_CHECK(open);
            size_t size = sizeof(mConfig);
            const auto result = nvs_get_blob(handle, "config-v1", &mConfig, &size);
            nvs_close(handle);
            if (result == ESP_OK && mConfig.magic == EEPROM_MAGIC && mConfig.ardupilot_mode == 2) {
                mConfig.ardupilot_mode = 1;
            }
            if (result == ESP_ERR_NVS_NOT_FOUND || result == ESP_ERR_NVS_INVALID_LENGTH ||
                (result == ESP_OK && (size != sizeof(mConfig) || !mConfig.valid()))) {
                ESP_LOGW(tag, "Missing/incompatible/invalid configuration; using defaults");
                mConfig = EEProm{};
                return;
            }
            ESP_ERROR_CHECK(result);
        }

        void save() {
            nvs_handle_t handle;
            ESP_ERROR_CHECK(nvs_open("relay", NVS_READWRITE, &handle));
            ESP_ERROR_CHECK(nvs_set_blob(handle, "config-v1", &mConfig, sizeof(mConfig)));
            ESP_ERROR_CHECK(nvs_commit(handle));
            nvs_close(handle);
            mSaveAt = 0;
            ESP_LOGI(tag, "Configuration saved");
        }

        void updateLed(const uint64_t now) {
            using State = Controller<Application>::State;
            bool on = false;
            switch (mController.state()) {
            case State::Starting:
                break;
            case State::Init:
                on = true;
                break;
            case State::Disconnected:
                on = now % 400'000 < 100'000;
                break;
            case State::Connected:
                on = now % 2'000'000 < 200'000 ||
                    (now % 2'000'000 >= 400'000 && now % 2'000'000 < 600'000);
                break;
            }
            ESP_ERROR_CHECK(gpio_set_level(gpio_num_t(Board::led), on != Board::ledActiveLow));
        }

        EEProm mConfig{};
        Controller<Application> mController{mConfig, *this};
        QueueHandle_t mInputEvents = nullptr;
        QueueHandle_t mModuleEvents = nullptr;
        uint64_t mSaveAt = 0;
    };
}

extern "C" void app_main() {
    static Relay::Application application;
    if (xTaskCreate([](void*) { application.run(); }, "relay", 8'192, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE("relay", "Could not create application task");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}
