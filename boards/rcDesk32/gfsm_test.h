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

#include <chrono>
#include <utility>

#include "mcu/mcu.h"
#include "meta.h"
#include "i2c.h"
#include "tick.h"
#include "switches.h"

using namespace std::literals::chrono_literals;

#ifndef USE_RC720

template<typename... PP>
struct Distributor {
    static inline void set(const uint8_t i, const uint16_t v) {
        (PP::set(i, v), ...);
    }
};

template <typename Devs>
struct GFSM {
    using devs = Devs;
    using debug = devs::debug;
    using systemTimer = devs::systemTimer;
    using storage = devs::storage;

#ifndef USE_SWD
    using pulse_in = devs::pulse_in;
#endif

    using tp1 = devs::tp1;

    enum class State : uint8_t {Undefined, Init, I2CScan, Run};

    static inline void updateFromEeprom() {
    }

    static inline void init() {
        devs::init();
    }
    static inline void periodic() {
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
    }

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};

    static inline void ratePeriodic() {
        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            mStateTick.on(initTicks, [] static {
                mState = State::I2CScan;
            });
            break;
        case State::I2CScan:
            break;
        case State::Run:
            mStateTick.on(debugTicks, []{
#ifndef USE_SWD
                IO::out<debug>("# pulse:");
                for(uint8_t i = 0; i < 8; ++i) {
                    IO::out<debug>(" v[", i, "]:", pulse_in::value(i));
                }
                IO::outl<debug>();
#endif
            });
            update();
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                IO::outl<debug>("# Undef");
                break;
            case State::Init:
                IO::outl<debug>("# Init");
                break;
            case State::I2CScan:
                IO::outl<debug>("# I2C Scan");
                break;
            case State::Run:
                IO::outl<debug>("# Run");
                break;
            }
        }
    }

    static inline void setSw(const uint8_t index, const bool state) {
    }

    private:
    static inline void update() {
        static uint8_t n = 1;
        switch(n) {
        case 1:
            updateAnalogs();
            break;
        case 2:
            updateSMs();
            break;
        case 3:
            updateInc();
            break;
        case 4:
            updateCrsf();
            break;
        case 5:
            updateBluetooth();
            break;
        default:
            n = 0;
            break;
        }
        ++n;
    }
    static inline void updateAnalog(const uint8_t i, const uint8_t off = 0) {
        if (storage::eeprom.analogMaps[i].stream == Stream::Trainer) {
            for(uint8_t i = 0; i < 3; ++i) {
            }
        }
        else if (storage::eeprom.analogMaps[i].stream == Stream::VControls) {
            for(uint8_t i = 0; i < 3; ++i) {
            }
        }
    }
    static inline void updateAnalogs() {
        updateAnalog(0, 0);
        updateAnalog(1, 3);
    }
    static inline void updateSM(const uint8_t i, const auto f) {
        if (storage::eeprom.smMaps[i].stream == Stream::Trainer) {
            for(uint8_t i = 0; i < 6; ++i) {
            }
        }
        else if (storage::eeprom.smMaps[i].stream == Stream::VControls) {
            for(uint8_t i = 0; i < 6; ++i) {
            }
        }
    }
    static inline void updateSMs() {
    }
    static inline void updateInc() {
    }
    static inline void updateCrsf() {
    }
    static inline void updateBluetooth() {
    }
    static inline External::Tick<systemTimer> mStateTick;
    static inline State mState{State::Undefined};
};
#endif

#ifdef USE_RC720
template <typename Devs>
struct GFSM {
    using devs = Devs;
    using debug = devs::debug;
    using systemTimer = devs::systemTimer;
    using storage = devs::storage;
    using i2c = devs::i2c3;

    using radio = devs::radio;

    using switches = External::Switches<typename devs::pca0, typename devs::pca1>;

    using led1 = devs::ledBlinker1;
    using led2 = devs::ledBlinker2;

    using crsf_in = devs::crsf_in;
    using crsf_in_pa = crsf_in::input;
    using crsf_in_responder = crsf_in::output;

    enum class State : uint8_t {Undefined, Init, I2CScan, Run};

    static inline void init() {
        devs::init();
        switches::init();
        radio::init();
    }
    static inline void periodic() {
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
        i2c::periodic();
        switches::periodic();
        radio::periodic();
        crsf_in::periodic();
    }

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> switchesTicks{20ms};

    static inline void ratePeriodic() {
        led1::ratePeriodic();
        led2::ratePeriodic();
        i2c::ratePeriodic();
        radio::ratePeriodic();
        crsf_in::ratePeriodic();

        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                mState = State::I2CScan;
            });
            break;
        case State::I2CScan:
            if (i2c::isIdle()) {
                mState = State::Run;
            }
            break;
        case State::Run:
            switches::ratePeriodic();
            (++mSwitchesTick).on(switchesTicks, []{
                switches::startRead([](const uint8_t index, const uint8_t newState){
                    IO::outl<debug>("# Switch ", index, " ", newState);
                    if (index < 32) {
                        const uint8_t wIndex1 = 2 * index;
                        const uint8_t wIndex2 = 2 * index + 1;
                        const uint64_t mask1 = (1UL << wIndex1);
                        const uint64_t mask2 = (1UL << wIndex2);
                        const bool on1 = (newState == 0);
                        const bool on2 = (newState == 2);

                        mSwitchesState = (mSwitchesState & ~mask1) | (on1 ? mask1 : 0);
                        mSwitchesState = (mSwitchesState & ~mask2) | (on2 ? mask2 : 0);
                        radio::set(mSwitchesState);
                    }
                });
            });
            mStateTick.on(debugTicks, []{
                IO::outl<debug>("# i2c state:", (uint8_t)i2c::mState, " ", i2c::mIsr, " ", i2c::errors());
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                IO::outl<debug>("# Undef");
                break;
            case State::Init:
                IO::outl<debug>("# Init");
                led1::event(led1::Event::Steady);
                break;
            case State::I2CScan:
                IO::outl<debug>("# I2C Scan");
                if (i2c::scan([](const Mcu::Stm::I2C::Address a){
                              IO::outl<debug>("I2C: ", a.value);
                            })) {
                    IO::outl<debug>("# i2c scan start");
                }
                else {
                    IO::outl<debug>("# i2c scan failed");
                }
                break;
            case State::Run:
                led1::event(led1::Event::Slow);
                IO::outl<debug>("# Run");
                break;
            }
        }
    }
    private:
    static inline uint64_t mSwitchesState = 0;
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mSwitchesTick;
    static inline State mState{State::Undefined};
};

#endif
