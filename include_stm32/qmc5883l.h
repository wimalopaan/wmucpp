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

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "clock.h"
#include "timer.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "tick.h"
#include "i2c.h"
#include "etl/event.h"

namespace External {
    using namespace etl::literals;

    template<typename Device, Mcu::Stm::I2C::Address Adr, typename systemTimer, typename Debug = void>
    struct QMC5883L {
        using dev = Device;
        using debug = Debug;
        enum class State : uint8_t {Init,
                                    Setup, SetupWait1, SetupWait2,
                                    StartRead, Read,
                                    Idle};

        enum class Event : uint8_t {None, Start};

        static inline constexpr External::Tick<systemTimer> initTicks{10ms};
        static inline constexpr External::Tick<systemTimer> waitTicks{5ms};

        static inline void init() {
            // mActive = false;
        }
        static inline void periodic() {
        }
        static inline Mcu::Stm::I2C::Address address() {
            return Adr;
        }
        // static inline void activate(const bool on) {
        //     mActive = on;
        // }
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
            case State::Setup:
                if (dev::isIdle()) {
                    mStateTick.on(waitTicks, []{ // without timeout seems not to work
                        dev::write(Adr, {0x0B, 0x01}); // see datasheet: set/reset
                        mState = State::SetupWait1;
                    });
                }
                break;
            case State::SetupWait1:
                if (dev::isIdle()) {
                    mStateTick.on(waitTicks, []{ // without timeout seems not to work
                        dev::write(Adr, {0x09, 0b0000'0101}); // OSR=512, RNG=2G, ODR=50Hz, Mode=continuous
                        mState = State::SetupWait2;
                    });
                }
                break;
            case State::SetupWait2:
                if (dev::isIdle()) {
                    mState = State::Idle;
                }
                break;
            case State::Idle:
                if (dev::isIdle()) {
                    mEvent.on(Event::Start, []{
                        mState = State::StartRead;
                    });
                }
                break;
            case State::StartRead:
                if (dev::isIdle()) {
                    dev::read(Adr, std::byte{0x00}, 6);
                    mState = State::Read;
                }
                break;
            case State::Read:
                if (dev::readDataAvailable()) {
                    const auto& c = dev::readData();
                    mX = (uint16_t)c[0] + ((uint16_t)c[1] << 8);
                    mY = (uint16_t)c[2] + ((uint16_t)c[3] << 8);
                    mZ = (uint16_t)c[4] + ((uint16_t)c[5] << 8);
                    IO::outl<debug>("# Qmc5883 x: ", mX);
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
                    IO::outl<debug>("# Qmc5883 Init ", Adr.value);
                    break;
                case State::Setup:
                    IO::outl<debug>("# Qmc5883 Setup ", Adr.value);
                    break;
                case State::SetupWait1:
                    IO::outl<debug>("# Qmc5883 SuWait1 ", Adr.value);
                    break;
                case State::SetupWait2:
                    IO::outl<debug>("# Qmc5883 SuWait2 ", Adr.value);
                    break;
                case State::StartRead:
                    IO::outl<debug>("# Qmc5883 StRead ", Adr.value);
                    break;
                case State::Read:
                    IO::outl<debug>("# Qmc5883 Read ", Adr.value);
                    break;
                case State::Idle:
                    IO::outl<debug>("# Qmc5883 Idle ", Adr.value);
                    break;
                }
            }
        }
        static inline bool isPendingEvent() {
            return !!mEvent;
        }
        static inline bool isIdle() {
            return mState == State::Idle;
        }
        static inline int32_t x() {
            return mX;
        }
        static inline int32_t y() {
            return mY;
        }
        static inline int32_t z() {
            return mZ;
        }
        private:
        // static inline bool mActive = false;
        static inline int16_t mX = 0;
        static inline int16_t mY = 0;
        static inline int16_t mZ = 0;
        static inline etl::Event<Event> mEvent;
        static inline External::Tick<systemTimer> mStateTick;
        static inline State mState{State::Init};
    };
}
