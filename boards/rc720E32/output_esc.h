/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "output.h"
#include "iservo.h"

template<typename Devs>
struct EscOutputs {
    using devs = Devs;
    using debug = devs::debug;
    using esc1_pwm = devs::esc1_pwm;
    using esc2_pwm = devs::esc2_pwm;
    using esc32_1 = devs::esc32_1;
    using esc32_2 = devs::esc32_2;
    using esc32ascii_1 = devs::esc32ascii_1;
    using esc32ascii_2 = devs::esc32ascii_2;
    using vesc_1 = devs::vesc_1;
    using vesc_2 = devs::vesc_2;

    using esc1_sbus = devs::esc1_sbus;
    using esc2_sbus = devs::esc2_sbus;

    using bt2 = devs::bt2;

    template<uint8_t N>
    static inline void esc(const uint8_t e) {
        IO::outl<debug>("# esc", N, ": ", e);
        static_assert(N <= 1);
        using esc_pwm_t = std::conditional_t<(N == 0), Esc<esc1_pwm>, Esc<esc2_pwm>>;
        using esc_esc32_t = std::conditional_t<(N == 0), Esc<esc32_1>, Esc<esc32_2>>;
        using esc_esc32ascii_t = std::conditional_t<(N == 0), Esc<esc32ascii_1>, Esc<esc32ascii_2>>;
        using esc_vesc_t = std::conditional_t<(N == 0), Esc<vesc_1>, Esc<vesc_2>>;
        using bt2_t = std::conditional_t<(N == 0), void, Esc<bt2>>;
        using esc_sbus_t = std::conditional_t<(N == 0), Esc<esc1_sbus>, Esc<esc2_sbus>>;

        // std::integral_constant<uint8_t, sizeof(esc_pwm_t)>::_;
        static_assert(sizeof(esc_pwm_t) == sizeof(esc_esc32_t));
        static_assert(sizeof(esc_pwm_t) == sizeof(esc_esc32ascii_t));
        static_assert(sizeof(esc_pwm_t) == sizeof(esc_vesc_t));

        switch(e) {
        case 0: // PWM
            escs[N] = nullptr;
            escs[N] = std::make_unique<esc_pwm_t>();
            break;
        case 1: // Esc32/Serial
            escs[N] = nullptr;
            escs[N] = std::make_unique<esc_esc32_t>();
            break;
        case 2: // Esc32/Ascii
            escs[N] = nullptr;
            escs[N] = std::make_unique<esc_esc32ascii_t>();
            break;
        case 3: // V/Esc
            escs[N] = nullptr;
            escs[N] = std::make_unique<esc_vesc_t>();
            break;
        case 4: // sbus
            escs[N] = nullptr;
            escs[N] = std::make_unique<esc_sbus_t>();
            break;
        case 5: // None
            escs[N] = nullptr;
            break;
        case 6: // BT
            escs[N] = nullptr;
            if constexpr(N == 1) {
                escs[N] = std::make_unique<bt2_t>();
            }
            break;
        default:
            break;
        }
    }
    static inline std::pair<uint8_t, uint8_t> fwVersion(const uint8_t n) {
        if ((n < escs.size()) && escs[n]) {
            return escs[n]->fwVersion();
        }
        return {};
    }
    static inline std::pair<uint8_t, uint8_t> hwVersion(const uint8_t n) {
        if ((n < escs.size()) && escs[n]) {
            return escs[n]->hwVersion();
        }
        return {};
    }
    static inline uint16_t current(const uint8_t n) {
        if ((n < escs.size()) && escs[n]) {
            return escs[n]->current();
        }
        return 0;
    }
    static inline uint16_t rpm(const uint8_t n) {
        if ((n < escs.size()) && escs[n]) {
            return escs[n]->rpm();
        }
        return 0;
    }
    static inline uint16_t voltage(const uint8_t n) {
        if ((n < escs.size()) && escs[n]) {
            return escs[n]->voltage();
        }
        return 0;
    }
    static inline uint16_t temp(const uint8_t n) {
        if ((n < escs.size()) && escs[n]) {
            return escs[n]->temp();
        }
        return 0;
    }
    static inline void set(const uint8_t n, const uint16_t v) {
        if ((n < escs.size()) && escs[n]) {
            escs[n]->set(v);
        }
    }
    static inline void setChannel(const uint8_t n, const uint8_t ch, const uint16_t v) {
        if ((n < escs.size()) && escs[n]) {
            escs[n]->setChannel(ch, v);
        }
    }
    static inline void periodic() {
        for(const auto& e : escs) {
            if (e) {
                e->periodic();
            }
        }
    }
    static inline void ratePeriodic() {
        for(const auto& e : escs) {
            if (e) {
                e->ratePeriodic();
            }
        }
    }
    private:
    static inline std::array<std::unique_ptr<IEsc>, 2> escs{};
};
