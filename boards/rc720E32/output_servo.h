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

    using servo1_mpx = devs::ppm_mpx1;
    // using servo2_mpx = devs::ppm_mpx2;

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
    static inline void update(/*const uint8_t n*/) {
        for(const auto& s : servos) {
            if (s) {
                s->update();
            }
        }
        // if ((n < servos.size()) && servos[n]) {
        //     servos[n]->update();
        // }
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

    template<uint8_t N>
    static inline void servo(const uint8_t s) {
        IO::outl<debug>("# servo", N, ": ", s);
        static_assert(N <= 1);
        using srv_ws_t = std::conditional_t<(N == 0), Servo<servo1_ws>, Servo<servo2_ws>>;
        using srv_ft_t = std::conditional_t<(N == 0), Servo<servo1_ft>, Servo<servo2_ft>>;
        // using srv_mpx_t = std::conditional_t<(N == 0), Servo<servo1_mpx>, Servo<servo2_mpx>>;
        using srv_mpx_t = Servo<servo1_mpx>;
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
        case 3: // cppm-mpx
            servos[N] = nullptr;
            if constexpr(N == 0) {
                servos[N] = std::make_unique<srv_mpx_t>();
            }
            break;
        case 4: // none
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
