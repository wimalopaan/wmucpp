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
    struct MPU6050 {
        using dev = Device;
        using debug = Debug;
        enum class State : uint8_t {Init,
                                    Setup,
                                    SetupWait,
                                    StartRead, Read,
                                    Idle};

        enum class Event : uint8_t {None, Start};

        static inline constexpr External::Tick<systemTimer> initTicks{800ms};
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
                        dev::write(Adr, {0x6B, 0x00}); // see datasheet
                        mState = State::SetupWait;
                    });
                }
                break;
            case State::SetupWait:
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
                    dev::read(Adr, std::byte{0x3b}, 6);
                    mState = State::Read;
                }
                break;
            case State::Read:
                if (dev::readDataAvailable()) {
                    const auto& c = dev::readData();
                    mAccX = (uint8_t)c[1] + ((uint8_t)c[0] << 8);
                    mAccY = (uint8_t)c[3] + ((uint8_t)c[2] << 8);
                    mAccZ = (uint8_t)c[5] + ((uint8_t)c[4] << 8);
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
                    IO::outl<debug>("# MPU6050 Init ", Adr.value);
                    break;
                case State::Setup:
                    IO::outl<debug>("# MPU6050 Setup ", Adr.value);
                    break;
                case State::SetupWait:
                    IO::outl<debug>("# MPU6050 SuWait2 ", Adr.value);
                    break;
                case State::StartRead:
                    IO::outl<debug>("# MPU6050 StRead ", Adr.value);
                    break;
                case State::Read:
                    IO::outl<debug>("# MPU6050 Read ", Adr.value);
                    break;
                case State::Idle:
                    IO::outl<debug>("# MPU6050 Idle ", Adr.value);
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
        static inline int32_t accX() {
            return mAccX;
        }
        static inline int32_t accY() {
            return mAccY;
        }
        static inline int32_t accZ() {
            return mAccZ;
        }
        private:
        // static inline bool mActive = false;
        static inline int16_t mAccX = 0;
        static inline int16_t mAccY = 0;
        static inline int16_t mAccZ = 0;
        static inline etl::Event<Event> mEvent;
        static inline External::Tick<systemTimer> mStateTick;
        static inline State mState{State::Init};
    };
}
