#pragma once

#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"
#include "units.h"

#include <type_traits>
#include <concepts>
#include <cstddef>
#include <array>

namespace External {
namespace TLC59108 {

template<typename Bus, auto Adr>
struct Switch {
    using bus= Bus;

    static inline constexpr uint8_t Mode1Reg = 0x00;
    static inline constexpr uint8_t Mode2Reg = 0x01;
    static inline constexpr uint8_t Pwm0Reg  = 0x02;
    static inline constexpr uint8_t Pwm1Reg  = 0x03;
    static inline constexpr uint8_t Pwm2Reg  = 0x04;
    static inline constexpr uint8_t Pwm3Reg  = 0x05;
    static inline constexpr uint8_t Pwm4Reg  = 0x06;
    static inline constexpr uint8_t Pwm5Reg  = 0x07;
    static inline constexpr uint8_t Pwm6Reg  = 0x08;
    static inline constexpr uint8_t Pwm7Reg  = 0x09;
    static inline constexpr uint8_t GrpPwmReg  = 0x0a;
    static inline constexpr uint8_t GrpFreqReg = 0x0b;
    static inline constexpr uint8_t Led0Reg    = 0x0c;
    static inline constexpr uint8_t Led1Reg    = 0x0d;

    static inline constexpr uint8_t AutoIncrFlag = 0b1000'0000;

    static inline constexpr uint8_t OscOffFlag    = 0b0001'0000;
    static inline constexpr uint8_t GrpDimFlag = 0b0010'0000;

    static inline void init() {
        std::array<std::byte, 2> data{std::byte(0 & ~OscOffFlag), std::byte(GrpDimFlag)};
        bus::write(Adr, std::byte(Mode1Reg | AutoIncrFlag), data);
    }

    static inline void pwm(const uint8_t out, const uint8_t pwm) {
        const uint8_t r = ((out & 0x07) + Pwm0Reg);
        bus::write(Adr, std::pair<uint8_t, uint8_t>{r, pwm});
    }

    static inline void on(const uint8_t out) {
        const uint8_t r = 0x0c;
        const uint8_t pwm = 0b01010101;
        bus::write(Adr, std::pair<uint8_t, uint8_t>{r, pwm});
    }
    static inline void off(const uint8_t out) {
        const uint8_t r = 0x0c;
        const uint8_t pwm = 0x00;
        bus::write(Adr, std::pair<uint8_t, uint8_t>{r, pwm});
    }

};

}
}
