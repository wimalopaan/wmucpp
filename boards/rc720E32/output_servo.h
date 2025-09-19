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
struct ServoOutputs {
    using devs = Devs;
    using debug = devs::debug;
    using servo1_ws = devs::srv1_waveshare;
    using servo2_ws = devs::srv2_waveshare;
    using servo1_ft = devs::srv1_feetech;
    using servo2_ft = devs::srv2_feetech;

    using servo1_sport = devs::srv1_sport;
    using servo2_sport = devs::srv2_sport;

    using srv1_pwm = devs::srv1_pwm;
    using srv2_pwm = devs::srv2_pwm;

    using servo1_mpx = devs::ppm_mpx1;
    // using servo2_mpx = devs::ppm_mpx2;

    using ws2812b_1 = devs::ws2812b_1;

    template<uint8_t N>
    static inline void offset(const uint16_t o) {
        static_assert(N <= 1);
        if (servos[N]) {
            servos[N]->offset(o);
        }
    }
    template<uint8_t N>
    static inline void speed(const uint16_t s) {
        static_assert(N <= 1);
        if (servos[N]) {
            servos[N]->speed(s);
        }
    }
    static inline void update() {
        for(const auto& s : servos) {
            if (s) {
                s->update();
            }
        }
    }
    static inline int8_t turns(const uint8_t n) {
        if ((n < servos.size()) && servos[n]) {
            return servos[n]->turns();
        }
        return 0;
    }
    static inline uint16_t actualPos(const uint8_t n) {
        if ((n < servos.size()) && servos[n]) {
            return servos[n]->actualPos();
        }
        return 0;
    }
    static inline std::pair<uint8_t, uint8_t> fwVersion(const uint8_t n) {
        if ((n < servos.size()) && servos[n]) {
            return servos[n]->fwVersion();
        }
        return {};
    }
    static inline std::pair<uint8_t, uint8_t> hwVersion(const uint8_t n) {
        if ((n < servos.size()) && servos[n]) {
            return servos[n]->hwVersion();
        }
        return {};
    }
    static inline void set(const uint8_t n, const uint16_t v) {
        if ((n < servos.size()) && servos[n]) {
            servos[n]->set(v);
        }
    }

    template<uint8_t N>
    static inline void servo(const uint8_t s) {
        IO::outl<debug>("# servo", N, ": ", s);
        static_assert(N <= 1);
        using srv_ws_t = std::conditional_t<(N == 0), Servo<servo1_ws>, Servo<servo2_ws>>;
        using srv_ft_t = std::conditional_t<(N == 0), Servo<servo1_ft>, Servo<servo2_ft>>;
        using srv_pwm = std::conditional_t<(N == 0), Servo<srv1_pwm>, Servo<srv2_pwm>>;
        using srv_sport = std::conditional_t<(N == 0), Servo<servo1_sport>, Servo<servo2_sport>>;
        // using srv_mpx_t = std::conditional_t<(N == 0), Servo<servo1_mpx>, Servo<servo2_mpx>>;
        using srv_mpx_t = Servo<servo1_mpx>;

        using srv_ws2812b_t = Servo<ws2812b_1>;

        static_assert(sizeof(srv_ft_t) == sizeof(srv_ws_t));
        static_assert(sizeof(srv_ft_t) == sizeof(srv_mpx_t));

        switch(s) {
        case 0: // analog FB
            servos[N] = nullptr;
            servos[N] = std::make_unique<srv_ft_t>();
            break;
        case 1: // PWM feedback
            servos[N] = nullptr;
            servos[N] = std::make_unique<srv_ft_t>();
            break;
        case 2: // serial
            servos[N] = nullptr;
            servos[N] = std::make_unique<srv_ws_t>();
            break;
        case 3: // pwm
            servos[N] = nullptr;
            servos[N] = std::make_unique<srv_pwm>();
            break;
        case 4: // S.port
            servos[N] = nullptr;
            servos[N] = std::make_unique<srv_sport>();
            break;
        case 5: // cppm-mpx
            servos[N] = nullptr;
            if constexpr(N == 0) {
                servos[N] = std::make_unique<srv_mpx_t>();
            }
            break;
        case 6: // ws2812
            servos[N] = nullptr;
            if constexpr(N == 0) {
                servos[N] = std::make_unique<srv_ws2812b_t>();
            }
            break;
        case 7: // none
            servos[N] = nullptr;
            break;
        default:
            break;
        }
    }
    static inline void zero() {
        for(const auto& s : servos) {
            if (s) {
                s->zero();
            }
        }
    }
    static inline void periodic() {
        for(const auto& s : servos) {
            if (s) {
                s->periodic();
            }
        }
    }
    static inline void ratePeriodic() {
        for(const auto& s : servos) {
            if (s) {
                s->ratePeriodic();
            }
        }
    }
    private:
    static inline std::array<std::unique_ptr<IServo>, 2> servos{};
};
