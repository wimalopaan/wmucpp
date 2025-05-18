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

#include <cstring>

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "clock.h"
#include "timer.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "gpio.h"
#include "tick.h"
#include "i2c.h"

#include "etl/event.h"

namespace External {
    using namespace etl::literals;

    namespace V2 {
        template<typename Device, Mcu::Stm::I2C::Address Adr, typename systemTimer, typename Debug = void>
        struct PCAL6408 {
            using dev = Device;
            using debug = Debug;
            enum class State : uint8_t {Init, Setup, SetupWait, Idle,
                                        WritePullupOn, WritePullupOnWait, WritePulldownOn, WritePulldownOnWait,
                                        StartRead1, StartRead2,
                                        Read1, Read2,
                                        Stop};

            enum class Event : uint8_t {None, Start};

            static inline constexpr External::Tick<systemTimer> initTicks{500ms};
            static inline constexpr External::Tick<systemTimer> waitTicks{5ms};

            static inline constexpr std::byte pupdEnableReg{0x43};
            static inline constexpr std::byte pupdSelectReg{0x44};
            static inline constexpr std::byte inputReg{0x00};

            static inline void init() {
                mActive = false;
            }
            static inline void periodic() {
            }
            static inline Mcu::Stm::I2C::Address address() {
                return Adr;
            }
            static inline void activate(const bool on) {
                mActive = on;
            }
            static inline void startRead() {
                mEvent = Event::Start;
            }
            static inline void ratePeriodic() {
                const auto oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::Init:
                    mStateTick.on(initTicks, []{
                        dev::clearErrors();
                        mState = State::Setup;
                    });
                    break;
                case State::Stop:
                    break;
                case State::Setup:
                    if (dev::isIdle()) {
                        dev::write(Adr, {pupdEnableReg, std::byte{0xff}});
                        mState = State::SetupWait;
                    }
                    break;
                case State::SetupWait:
                    if (dev::isIdle()) {
                        mState = State::Idle;
                    }
                    break;
                case State::Idle:
                    if (mActive && dev::isIdle()) {
                        mEvent.on(Event::Start, []{
                            mState = State::WritePullupOn;
                        });
                    }
                    break;
                case State::WritePullupOn:
                    dev::write(Adr, {pupdSelectReg, std::byte{0xff}});
                    mState = State::WritePullupOnWait;
                    break;
                case State::WritePullupOnWait:
                    if (dev::isIdle()) {
                        // if (dev::errors() > 0) {
                        //     mState = State::Idle;
                        // }
                        // else {
                        //     mState = State::StartRead1;
                        // }
                        mState = State::StartRead1;
                    }
                    break;
                case State::StartRead1:
                    dev::read(Adr, inputReg, 1);
                    mState = State::Read1;
                    break;
                case State::Read1:
                    if (dev::readDataAvailable()) {
                        mValuePU = dev::readData()[0];
                        mState = State::WritePulldownOn;
                    }
                    mStateTick.on(waitTicks, []{
                        mState = State::Idle;
                    });
                    break;
                case State::WritePulldownOn:
                    dev::write(Adr, {pupdSelectReg, std::byte{0x00}});
                    mState = State::WritePulldownOnWait;
                    break;
                case State::WritePulldownOnWait:
                    if (dev::isIdle()) {
                        // if (dev::errors() > 0) {
                        //     mState = State::Idle;
                        // }
                        // else {
                        //     mState = State::StartRead2;
                        // }
                        mState = State::StartRead2;
                    }
                    break;
                case State::StartRead2:
                    dev::read(Adr, inputReg, 1);
                    mState = State::Read2;
                    break;
                case State::Read2:
                    if (dev::readDataAvailable()) {
                        mValuePD = dev::readData()[0];
                        mState = State::Idle;
                    }
                    mStateTick.on(waitTicks, []{
                        mState = State::Idle;
                    });
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Init:
                        IO::outl<debug>("# Pca Init ", Adr.value);
                        break;
                    case State::Stop:
                        IO::outl<debug>("# Pca Stop ", Adr.value);
                        break;
                    case State::Setup:
                        IO::outl<debug>("# Pca Setup ", Adr.value);
                        break;
                    case State::SetupWait:
                        IO::outl<debug>("# Pca SuWait ", Adr.value);
                        break;
                    case State::Idle:
                        IO::outl<debug>("# Pca Idle ", Adr.value);
                        break;
                    case State::WritePullupOn:
                        IO::outl<debug>("# Pca WPup ", Adr.value);
                        break;
                    case State::WritePullupOnWait:
                        IO::outl<debug>("# Pca WPupW ", Adr.value);
                        break;
                    case State::StartRead1:
                        IO::outl<debug>("# Pca SR1 ", Adr.value);
                        break;
                    case State::Read1:
                        IO::outl<debug>("# Pca R1 ", Adr.value);
                        break;
                    case State::WritePulldownOn:
                        IO::outl<debug>("# Pca WPdn ", Adr.value);
                        break;
                    case State::WritePulldownOnWait:
                        IO::outl<debug>("# Pca WPdnW ", Adr.value);
                        break;
                    case State::StartRead2:
                        IO::outl<debug>("# Pca SR2 ", Adr.value);
                        break;
                    case State::Read2:
                        IO::outl<debug>("# Pca R2 ", Adr.value);
                        break;
                    }
                }
            }
            static inline bool isIdle() {
                return mState == State::Idle;
            }
            static inline uint8_t switchValue(const uint8_t s) {
                if (s >= 8) return 0;
                const std::byte mask = std::byte(1 << s);
                const std::byte vu = mValuePU & mask;
                const std::byte vd = mValuePD & mask;
                if (vd != vu) {
                    return 1;
                }
                else {
                    if (vu == std::byte{0x00}) {
                        return 0;
                    }
                    else {
                        return 2;
                    }
                }
            }
            private:
            static inline bool mActive = false;
            static inline etl::Event<Event> mEvent;
            static inline std::byte mValuePU{};
            static inline std::byte mValuePD{};
            static inline External::Tick<systemTimer> mStateTick;
            static inline State mState{State::Init};
        };

    }
    template<typename Device, Mcu::Stm::I2C::Address address, typename systemTimer>
    struct PCAL6408 {
        using dev = Device;
        enum class State : uint8_t {Init, Setup, SetupWait, Idle,
                                    WritePullupOn, WritePullupOnWait, WritePulldownOn, WritePulldownOnWait,
                                    StartRead1, StartRead2,
                                    Read1, Read2};

        enum class Event : uint8_t {None, Start};

        static inline constexpr External::Tick<systemTimer> initTicks{500ms};

        static inline constexpr std::byte pupdEnableReg{0x43};
        static inline constexpr std::byte pupdSelectReg{0x44};
        static inline constexpr std::byte inputReg{0x00};

        static inline void init() {
            mActive = false;
        }
        static inline void periodic() {
        }
        static inline void activate(const bool on) {
            mActive = on;
        }
        static inline void startRead() {
            mEvent = Event::Start;
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
                if (dev::isIdle()) {
                    dev::write(address, {pupdEnableReg, std::byte{0xff}});
                    mState = State::SetupWait;
                }
                break;
            case State::SetupWait:
                if (dev::isIdle()) {
                    mState = State::Idle;
                }
                break;
            case State::Idle:
                if (mActive && dev::isIdle()) {
                    mEvent.on(Event::Start, []{
                        mState = State::WritePullupOn;
                    });
                }
                break;
            case State::WritePullupOn:
                dev::write(address, {pupdSelectReg, std::byte{0xff}});
                mState = State::WritePullupOnWait;
                break;
            case State::WritePullupOnWait:
                if (dev::isIdle()) {
                    // if (dev::errors() > 0) {
                    //     mState = State::Idle;
                    // }
                    // else {
                    //     mState = State::StartRead1;
                    // }
                    mState = State::StartRead1;
                }
                break;
            case State::StartRead1:
                dev::read(address, inputReg, 1);
                mState = State::Read1;
                break;
            case State::Read1:
                if (dev::readDataAvailable()) {
                    mValuePU = dev::readData()[0];
                    mState = State::WritePulldownOn;
                }
                break;
            case State::WritePulldownOn:
                dev::write(address, {pupdSelectReg, std::byte{0x00}});
                mState = State::WritePulldownOnWait;
                break;
            case State::WritePulldownOnWait:
                if (dev::isIdle()) {
                    // if (dev::errors() > 0) {
                    //     mState = State::Idle;
                    // }
                    // else {
                    //     mState = State::StartRead2;
                    // }
                    mState = State::StartRead2;
                }
                break;
            case State::StartRead2:
                dev::read(address, inputReg, 1);
                mState = State::Read2;
                break;
            case State::Read2:
                if (dev::readDataAvailable()) {
                    mValuePD = dev::readData()[0];
                    mState = State::Idle;
                }
                break;
            }
            if (oldState != mState) {
            }
        }
        static inline bool isIdle() {
            return mState == State::Idle;
        }
        static inline uint8_t switchValue(const uint8_t s) {
            if (s >= 8) return 0;
            const std::byte mask = std::byte(1 << s);
            const std::byte vu = mValuePU & mask;
            const std::byte vd = mValuePD & mask;
            if (vd != vu) {
                return 1;
            }
            else {
                if (vu == std::byte{0x00}) {
                    return 0;
                }
                else {
                    return 2;
                }
            }
        }
        private:
        static inline bool mActive = false;
        static inline etl::Event<Event> mEvent;
        static inline std::byte mValuePU{};
        static inline std::byte mValuePD{};
        static inline External::Tick<systemTimer> mStateTick;
        static inline State mState{State::Init};
    };
}
