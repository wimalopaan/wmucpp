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

    template<uint8_t N>
    static inline void esc(const uint8_t e) {
        IO::outl<debug>("# esc", N, ": ", e);
        static_assert(N <= 1);
        using esc_pwm_t = std::conditional_t<(N == 0), Esc<esc1_pwm>, Esc<esc2_pwm>>;
        using esc_esc32_t = std::conditional_t<(N == 0), Esc<esc32_1>, Esc<esc32_2>>;
        using esc_esc32ascii_t = std::conditional_t<(N == 0), Esc<esc32ascii_1>, Esc<esc32ascii_2>>;
        using esc_vesc_t = std::conditional_t<(N == 0), Esc<vesc_1>, Esc<vesc_2>>;

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
        case 4: // None
            escs[N] = nullptr;
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
    static inline void set(const uint8_t n, const uint16_t v) {
        if ((n < escs.size()) && escs[n]) {
            escs[n]->set(v);
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
