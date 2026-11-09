#pragma once

#include <array>
#include <cstdint>

#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#include "board.h"
#include "waveform.h"

namespace Multiswitch {
    class SoftwarePwm {
    public:
        static void init() {
            gpio_config_t pins{};
            pins.pin_bit_mask = allPins;
            pins.mode = GPIO_MODE_OUTPUT;
            ESP_ERROR_CHECK(gpio_config(&pins));
            REG_WRITE(GPIO_OUT_W1TC_REG, allPins);

            gptimer_config_t config{};
            config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
            config.direction = GPTIMER_COUNT_UP;
            config.resolution_hz = 1'000'000;
            ESP_ERROR_CHECK(gptimer_new_timer(&config, &mTimer));
            gptimer_event_callbacks_t callbacks{};
            callbacks.on_alarm = alarm;
            ESP_ERROR_CHECK(gptimer_register_event_callbacks(mTimer, &callbacks, nullptr));
            ESP_ERROR_CHECK(gptimer_enable(mTimer));
            gptimer_alarm_config_t first{};
            first.alarm_count = 1'000;
            ESP_ERROR_CHECK(gptimer_set_alarm_action(mTimer, &first));
            ESP_ERROR_CHECK(gptimer_start(mTimer));
        }

        static void update(const std::array<uint8_t, 8>& duties, const uint8_t frequency) {
            portENTER_CRITICAL(&mLock);
            for (size_t i = 0; i < duties.size(); ++i) mPending.duties[i] = duties[i];
            mPending.period = 10'000 / frequency;
            portEXIT_CRITICAL(&mLock);
        }

    private:
        static bool IRAM_ATTR alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void*) {
            // A frame boundary latches all channels atomically, including frequency changes.
            if (mPhase == 100) {
                portENTER_CRITICAL_ISR(&mLock);
                mActive = mPending;
                portEXIT_CRITICAL_ISR(&mLock);
                mFrameStart = event->alarm_value;
                mPhase = 0;
            }
            uint64_t now;
            ESP_ERROR_CHECK(gptimer_get_raw_count(timer, &now));
            const uint32_t elapsed = now > mFrameStart ? now - mFrameStart : 0;
            const auto edge = mActive.sample(elapsed, mPhase);
            REG_WRITE(GPIO_OUT_W1TC_REG, allPins & ~edge.high);
            REG_WRITE(GPIO_OUT_W1TS_REG, edge.high);
            mPhase = edge.nextPhase;
            gptimer_alarm_config_t next{};
            next.alarm_count = mFrameStart + uint64_t{mActive.period} * edge.nextPhase / 100;
            // Delayed interrupts must not re-arm an alarm behind the counter.
            if (next.alarm_count <= now + 5) next.alarm_count = now + 5;
            ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &next));
            return false;
        }

        static constexpr uint32_t allPins = [] {
            uint32_t mask = 0;
            for (const int pin : Board::outputs) mask |= uint32_t{1} << pin;
            return mask;
        }();
        static inline gptimer_handle_t mTimer = nullptr;
        static inline portMUX_TYPE mLock = portMUX_INITIALIZER_UNLOCKED;
        static inline Waveform mPending{};
        static inline Waveform mActive{};
        static inline uint64_t mFrameStart = 0;
        static inline uint8_t mPhase = 100;
    };
}
