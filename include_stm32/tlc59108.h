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

        template<typename Bus, auto Adr, typename Debug = void>
        struct Switch {
            using bus = Bus;

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
            static inline constexpr uint8_t OscOffFlag   = 0b0001'0000;
            static inline constexpr uint8_t GrpDimFlag   = 0b0010'0000;

            enum class State : uint8_t {Idle, Init, Wait, WriteState, WritePwm, WritePwmNState};
            enum class Event : uint8_t {None, Init, UpdatePwm, UpdateState, UpdateBoth};

            static inline void periodic() {
                bus::periodic();
                switch(mState) {
                case State::Idle:
                    if (const Event e = std::exchange(mEvent, Event::None); e == Event::Init) {
                        mState = State::Init;
                    }
                    else if (e == Event::UpdatePwm) {
                        mState = State::WritePwm;
                    }
                    else if (e == Event::UpdateState) {
                        mState = State::WriteState;
                    }
                    else if (e == Event::UpdateBoth) {
                        mState = State::WritePwmNState;
                    }
                    break;
                case State::Wait:
                    if (bus::isIdle()) {
                        mState = State::Idle;
                    }
                    break;
                case State::Init:
                    if (bus::isIdle()) {
                        std::array<std::byte, 2> data{std::byte(0 & ~OscOffFlag), std::byte(GrpDimFlag)};
                        bus::write(Adr, std::byte(Mode1Reg | AutoIncrFlag), data);
                        mState = State::WritePwm;
                    }
                    break;
                case State::WritePwm:
                    if (bus::isIdle()) {
                        bus::write(Adr, (uint8_t)(Pwm0Reg | AutoIncrFlag), mLedPwm);
                        mState = State::Wait;
                    }
                    break;
                case State::WritePwmNState:
                    if (bus::isIdle()) {
                        bus::write(Adr, (uint8_t)(Pwm0Reg | AutoIncrFlag), mLedPwm);
                        mState = State::WriteState;
                    }
                    break;
                case State::WriteState:
                    if (bus::isIdle()) {
                        bus::write(Adr, (uint8_t)(Led0Reg | AutoIncrFlag), mLedState);
                        mState = State::Wait;
                    }
                    break;
                }
            }
            static inline void init() {
                mEvent = Event::Init;
            }
            static inline void pwm(const uint8_t out, const uint8_t pwm) {
                const uint8_t scaled = (uint32_t(pwm) * 255) / 100;
                IO::outl<Debug>("TLC: pwm: ", out, " ", pwm, " ", scaled);
                if (scaled != mLedPwm[out]) {
                    mLedPwm[out] = scaled;
                    equalizePwm();
                    mEvent = Event::UpdatePwm;
                }
            }
            static inline void pwmon(const uint8_t out) {
                IO::outl<Debug>("TLC: pwmon: ", out);
                if (leds(out, 0b10)) {
                    mEvent = Event::UpdateState;
                }
            }
            static inline void on(const uint8_t out) {
                IO::outl<Debug>("TLC: on: ", out);
                if (leds(out, 0b01)) {
                    mEvent = Event::UpdateState;
                }
            }
            static inline void off(const uint8_t out) {
                IO::outl<Debug>("TLC: off: ", out);
                if (leds(out, 0b00)) {
                    mEvent = Event::UpdateState;
                }
            }
            static inline void grouping(const uint8_t g) {
                IO::outl<Debug>("TLC: grp: ", g);
                if (g > 3) return;
                mGrouping = g;
                equalizePwm();
            }
            private:
            static inline void equalizePwm() {
                switch(mGrouping) {
                case 0:
                    break;
                case 1:
                    mLedPwm[1] = mLedPwm[0];
                    mLedPwm[3] = mLedPwm[2];
                    mLedPwm[5] = mLedPwm[4];
                    mLedPwm[6] = mLedPwm[6];
                    break;
                case 2:
                    mLedPwm[1] = mLedPwm[0];
                    mLedPwm[2] = mLedPwm[0];
                    mLedPwm[3] = mLedPwm[0];
                    mLedPwm[5] = mLedPwm[4];
                    mLedPwm[6] = mLedPwm[4];
                    mLedPwm[7] = mLedPwm[4];
                    break;
                case 3:
                    mLedPwm[1] = mLedPwm[0];
                    mLedPwm[2] = mLedPwm[0];
                    mLedPwm[3] = mLedPwm[0];
                    mLedPwm[4] = mLedPwm[0];
                    mLedPwm[5] = mLedPwm[0];
                    mLedPwm[6] = mLedPwm[0];
                    mLedPwm[7] = mLedPwm[0];
                    break;
                }
            }
            static inline bool led(const uint8_t n, const uint8_t p) {
                if (n < 4) {
                    const uint8_t shift = 2 * n;
                    const uint8_t mask = (0b11 << shift);
                    const auto prev = mLedState[0];
                    mLedState[0] = (mLedState[0] & ~mask) | (p << shift);
                    return (prev != mLedState[0]);
                }
                else {
                    const uint8_t shift = 2 * (n - 4);
                    const uint8_t mask = (0b11 << shift);
                    const auto prev = mLedState[1];
                    mLedState[1] = (mLedState[1] & ~mask) | (p << shift);
                    return (prev != mLedState[1]);
                }
            }
            static inline bool leds(const uint8_t n, const uint8_t pattern) {
                if (n >= 8) return false;
                bool b = false;
                const uint8_t p = pattern & 0b11;
                switch(mGrouping) {
                case 0:
                    b |= led(n, p);
                    break;
                case 1:
                    if (!(n & 0b0000'0001)) {
                        const uint8_t ng = n & 0b0000'0110;
                        b |= led(ng, p);
                        b |= led(ng + 1, p);
                    }
                    break;
                case 2:
                    if (!(n & 0b0000'0011)) {
                        const uint8_t ng = n & 0b0000'0100;
                        b |= led(ng, p);
                        b |= led(ng + 1, p);
                        b |= led(ng + 2, p);
                        b |= led(ng + 3, p);
                    }
                    break;
                case 3:
                    if (!(n & 0b0000'0111)) {
                        b |= led(0, p);
                        b |= led(1, p);
                        b |= led(2, p);
                        b |= led(3, p);
                        b |= led(4, p);
                        b |= led(5, p);
                        b |= led(6, p);
                        b |= led(7, p);
                    }
                    break;
                }
                return b;
            }
            static inline uint8_t mGrouping{};
            static inline std::array<uint8_t, 2> mLedState{};
            static inline std::array<uint8_t, 8> mLedPwm{1, 1, 1, 1, 1, 1, 1, 1};
            static inline State mState{State::Idle};
            static inline Event mEvent{Event::None};
        };
    }
}
