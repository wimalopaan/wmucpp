/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "mcu/alternate.h"
#include "etl/fifo.h"
#include "spi.h"
#include "tick.h"

namespace External {
    template<uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct PCA9745 {
        using systemTimer = Config::systemTimer;
        using mosi = Config::mosi;
        using miso = Config::miso;
        using clk  = Config::clk;
        using cs   = Config::cs;
        using rst  = Config::reset;
        using oe   = Config::oe;

        using debug = Config::debug;
        using txDmaComponent = Config::txDmaComponent;

        struct SpiConfig {
            static inline constexpr size_t clockDivider = 64;
            using value_type = uint16_t;
            static inline constexpr size_t size = 256;
            struct Isr {
                static inline constexpr bool txComplete = true;
            };
            using rxDmaComponent = void;
            using txDmaComponent = PCA9745::txDmaComponent;
        };

        using spi = Mcu::Stm::V2::Spi<N, SpiConfig, MCU>;

        enum class State : uint8_t {Init, Setup, Idle, Write};
        enum class Event : uint8_t {None, TransferComplete};

        static inline constexpr External::Tick<systemTimer> initTicks{100ms};
        static inline constexpr External::Tick<systemTimer> sendTicks{200ms};

        struct Command {
            uint8_t address;
            uint8_t value;
        };

        static inline void init() {
            IO::outl<debug>("# PCA9745 init");
            Mcu::Arm::Atomic::access([]{
                mActive = true;
                mState = State::Init;
                mEvent = Event::None;
                spi::init();
            });
            rst::template dir<Mcu::Output>();
            rst::set();
            oe::template dir<Mcu::Output>();
            oe::reset();

            static constexpr uint8_t miso_af = Mcu::Stm::AlternateFunctions::mapper_v<miso, spi, Mcu::Stm::AlternateFunctions::MISO>;
            miso::afunction(miso_af);
            static constexpr uint8_t mosi_af = Mcu::Stm::AlternateFunctions::mapper_v<mosi, spi, Mcu::Stm::AlternateFunctions::MOSI>;
            mosi::afunction(mosi_af);
            static constexpr uint8_t clk_af = Mcu::Stm::AlternateFunctions::mapper_v<clk, spi, Mcu::Stm::AlternateFunctions::SCLK>;
            clk::afunction(clk_af);
            static constexpr uint8_t cs_af = Mcu::Stm::AlternateFunctions::mapper_v<cs, spi, Mcu::Stm::AlternateFunctions::CS>;
            cs::afunction(cs_af);
        }
        static inline void reset() {
            IO::outl<debug>("# PCA9745 reset");
            Mcu::Arm::Atomic::access([]{
                mActive = false;
                spi::reset();
            });
            miso::analog();
            mosi::analog();
            clk::analog();
            cs::analog();
            oe::analog();
            rst::analog();
        }
        static inline void ledControl(const uint8_t led, const uint8_t control) {
            const uint8_t ledByte = led / 4;
            const uint8_t ledShift = led % 4;
            const uint8_t ledMask = (0b11 << ledShift);
            mLedControl[ledByte] = (mLedControl[ledByte] & ~ledMask) | ((control << ledShift) & ledMask);
            mFifo.push_back(Command{uint8_t(0x02 + ledByte), mLedControl[ledByte]});
        }

        static inline void ledPwm(const uint8_t led, const uint8_t pwm) {
            mFifo.push_back(Command{uint8_t(0x08 + led), pwm});
        }
        static inline void ledGroup(const uint8_t led, const uint8_t group) {
            const uint8_t ledByte = led / 4;
            const uint8_t ledShift = led % 4;
            const uint8_t ledMask = (0b11 << ledShift);
            mLedGroups[ledByte] = (mLedGroups[ledByte] & ~ledMask) | ((group << ledShift) & ledMask);
            mFifo.push_back(Command{uint8_t(0x3a + ledByte), mLedGroups[ledByte]});
        }
        static inline void ledGradationMode(const uint8_t led, const bool on) {
            const uint8_t ledByte = led / 8;
            const uint8_t ledShift = led % 8;
            const uint8_t ledMask = (0b1 << ledShift);
            if (on) {
                mLedModes[ledByte] = mLedModes[ledByte] | ledMask;
            }
            else {
                mLedModes[ledByte] = mLedModes[ledByte] & ~ledMask;
            }
            mFifo.push_back(Command{uint8_t(0x38 + ledByte), mLedModes[ledByte]});
        }
        static inline void groupRampUp(const uint8_t group, const bool on) {
            const uint8_t mask = 0b1000'0000;
            if (on) {
                mGroupRamp[group] = (mGroupRamp[group] | mask);
            }
            else {
                mGroupRamp[group] = (mGroupRamp[group] & ~mask);
            }
            mFifo.push_back(Command{uint8_t(0x28 + (4 * group)), mGroupRamp[group]});
        }
        static inline void groupRampDown(const uint8_t group, const bool on) {
            const uint8_t mask = 0b0100'0000;
            if (on) {
                mGroupRamp[group] = (mGroupRamp[group] | mask);
            }
            else {
                mGroupRamp[group] = (mGroupRamp[group] & ~mask);
            }
            mFifo.push_back(Command{uint8_t(0x28 + (4 * group)), mGroupRamp[group]});
        }
        static inline void groupRampRate(const uint8_t group, const uint8_t rate) {
            const uint8_t mask = 0b0011'1111;
            mGroupRamp[group] = (mGroupRamp[group] & ~mask) | (rate & mask);
            mFifo.push_back(Command{uint8_t(0x28 + (4 * group)), mGroupRamp[group]});
        }
        static inline void groupStepTime(const uint8_t group, const uint8_t time) {
            mFifo.push_back(Command{uint8_t(0x29 + (4 * group)), uint8_t((time & 0b0011'1111) | 0b0100'0000)});
        }
        static inline void groupHoldTime(const uint8_t group, const uint8_t time) {
            mFifo.push_back(Command{uint8_t(0x2a + (4 * group)), uint8_t((time & 0b0011'1111) | 0b1100'0000)});
        }
        static inline void groupIRef(const uint8_t group, const uint8_t iref) {
            mFifo.push_back(Command{uint8_t(0x2b + (4 * group)), iref});
        }
        static inline void groupStart(const uint8_t group) {
            mFifo.push_back(Command{uint8_t(0x3e), uint8_t(0b11 << group)});
        }

        static inline void periodic() {
            mEvent.on(Event::TransferComplete, []{
                IO::outl<debug>("# PCA9745 TC -> Idle");
                mState = State::Idle;
            });
        }
        static inline void ratePeriodic() {
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Init:
                mStateTick.on(initTicks, []{
                    mState = State::Setup;
                });
                break;
            case State::Setup:
                if (!mFifo.empty()) {
                    mState = State::Write;
                }
                break;
            case State::Idle:
                if (!mFifo.empty()) {
                    mState = State::Write;
                }
                break;
            case State::Write:
                break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Init:
                    break;
                case State::Setup:
                    IO::outl<debug>("# PCA9745 Setup");
                    mFifo.push_back(Command{0x00, 0x00});
                    mFifo.push_back(Command{0x01, 0x14});
                    for(uint8_t i = 0; i < 16; ++i) {
                        mFifo.push_back(Command{uint8_t(0x18 + i), 0x7f}); // current
                    }
                    mFifo.push_back(Command{0x02, 0b1010'1010}); // individual brightness
                    mFifo.push_back(Command{0x03, 0b1010'1010});
                    mFifo.push_back(Command{0x04, 0b1010'1010});
                    mFifo.push_back(Command{0x05, 0b1010'1010});
                    break;
                case State::Idle:
                    IO::outl<debug>("# PCA9745 Idle");
                    break;
                case State::Write:
                    IO::outl<debug>("# PCA9745 Write");
                    spi::fillSendBuffer([](auto& data){
                        uint8_t n = 0;
                        for(uint16_t i = 0; i < data.size(); ++i) {
                            if (const auto c = mFifo.pop_front()) {
                                data[n++] = (c->address << 9) | c->value; // write address (9 shift)
                            }
                            else {
                                break;
                            }
                        }
                        return n;
                    });
                    break;
                }
            }
        }
        struct Isr {
            static inline void onTransferComplete(auto f) {
                if (mActive) {
                    auto f1 = [&]{
                        f();
                        mEvent = Event::TransferComplete;
                    };
                    spi::Isr::onTransferComplete(f1);
                }
            }
        };
        private:
        static inline std::array<uint8_t, 2> mLedModes{};
        static inline std::array<uint8_t, 4> mLedControl{0b1010'1010, 0b1010'1010, 0b1010'1010, 0b1010'1010};
        static inline std::array<uint8_t, 4> mLedGroups{};
        static inline std::array<uint8_t, 4> mGroupRamp{};
        static inline etl::FiFo<Command, 256> mFifo;
        static inline bool mActive = false;
        static inline etl::Event<Event> mEvent;
        static inline State mState = State::Init;
        static inline External::Tick<systemTimer> mStateTick;
    };
}
