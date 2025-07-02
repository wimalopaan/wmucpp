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
#include "rc/rc_2.h"

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
    using i2c1 = devs::i2c1;
    using i2c2 = devs::i2c2;
    using switches1 = devs::switches1;
    using switches2 = devs::switches2;

    using auxes1 = devs::auxes1;
    using auxes2 = devs::auxes2;

#ifdef USE_SWD
    using dist_hw = Distributor<typename devs::hwext2>;
    using dist_sb = Distributor<typename devs::sbus2>;
#else
    using dist_hw = Distributor<typename devs::hwext1, typename devs::hwext2>;
    using dist_sb = Distributor<typename devs::sbus1, typename devs::sbus2>;
#endif

    using adc = devs::adc;

    using enc1 = devs::enc1;
    using enc2 = devs::enc2;

    using sm1 = devs::sm1;
    using sm2 = devs::sm2;

    using bt = devs::bt;

    using led1 = devs::ledBlinker1;
    using led2 = devs::ledBlinker2;

    using crsf_in = devs::crsf_in;
    using crsf_in_pa = crsf_in::input;
    using crsf_in_responder = crsf_in::output;
    using crsf_cb = crsf_in::callback;

#ifndef USE_SWD
    using pulse_in = devs::pulse_in;
#endif

    using relay_bus = devs::relay_bus;

    using tp1 = devs::tp1;

    enum class State : uint8_t {Undefined, Init, I2CScan, Run};

    static inline void updateFromEeprom() {
        using cb = crsf_in::callback;
        cb::callbacks(true);
    }

    static inline void init() {
        busChannels.packed.ch0 = 992;
        busChannels.packed.ch1 = 992;
        busChannels.packed.ch2 = 992;
        busChannels.packed.ch3 = 992;
        busChannels.packed.ch4 = 992;
        busChannels.packed.ch5 = 992;
        busChannels.packed.ch6 = 992;
        busChannels.packed.ch7 = 992;
        busChannels.packed.ch8 = 992;
        busChannels.packed.ch9 = 992;
        busChannels.packed.ch10 = 992;
        busChannels.packed.ch11 = 992;
        busChannels.packed.ch12 = 992;
        busChannels.packed.ch13 = 992;
        busChannels.packed.ch14 = 992;
        busChannels.packed.ch15 = 992;
        devs::init();
        switches1::init();
        switches2::init();
    }
    static inline void periodic() {
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
        i2c1::periodic();
        i2c2::periodic();
        switches1::periodic();
        switches2::periodic();
        auxes1::periodic();
        auxes2::periodic();
        sm1::periodic();
        sm2::periodic();
        crsf_in::periodic();
        bt::periodic();
        relay_bus::periodic();
    }

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> telemTicks{5ms};

    union CRSF_Packet {
        struct __attribute__((packed)) {
            unsigned ch0 : 11;
            unsigned ch1 : 11;
            unsigned ch2 : 11;
            unsigned ch3 : 11;
            unsigned ch4 : 11;
            unsigned ch5 : 11;
            unsigned ch6 : 11;
            unsigned ch7 : 11;
            unsigned ch8 : 11;
            unsigned ch9 : 11;
            unsigned ch10 : 11;
            unsigned ch11 : 11;
            unsigned ch12 : 11;
            unsigned ch13 : 11;
            unsigned ch14 : 11;
            unsigned ch15 : 11;
        } packed;
        std::array<uint8_t, 22> raw;
    } ;
    static inline CRSF_Packet busChannels{};
    static inline uint64_t swichtes{};

    static inline uint32_t exp0{};

    static inline void ratePeriodic() {
        led1::ratePeriodic();
        led2::ratePeriodic();
        i2c1::ratePeriodic();
        i2c2::ratePeriodic();
        auxes1::ratePeriodic();
        auxes2::ratePeriodic();
        sm1::ratePeriodic();
        sm2::ratePeriodic();
        crsf_in::ratePeriodic();
        bt::ratePeriodic();
        relay_bus::ratePeriodic();

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
            if (i2c1::isIdle()) {
                mState = State::Run;
            }
            break;
        case State::Run:
            switches1::ratePeriodic();
            switches2::ratePeriodic();
            mStateTick.on(debugTicks, []{
                bt::setLed(99, true); // connection led
                // IO::outl<debug>("# i2c state:", (uint8_t)i2c1::mState, " ", i2c1::mIsr, " ", i2c1::errors());
                // IO::outl<debug>("# adc v0:", adc::values()[0], " v1:", adc::values()[1], " v2:", adc::values()[2], " v3:", adc::values()[3], " v4:", adc::values()[4], " v5:", adc::values()[5]);
                // IO::outl<debug>("# enc1:", enc1::value(), " enc2:", enc2::value());
                // IO::outl<debug>("# sm1 v0:", sm1::value(0), " v1:", sm1::value(1), " v2:", sm1::value(2), " v3:", sm1::value(3), " v4:", sm1::value(4), " v5:", sm1::value(5));
                // IO::outl<debug>("# sm1 v0:", sm2::value(0), " v1:", sm2::value(1), " v2:", sm2::value(2), " v3:", sm2::value(3), " v4:", sm2::value(4), " v5:", sm2::value(5));
#ifndef USE_SWD
                IO::out<debug>("# pulse:");
                for(uint8_t i = 0; i < 8; ++i) {
                    IO::out<debug>(" v[", i, "]:", pulse_in::value(i));
                }
                IO::outl<debug>();
                IO::out<debug>("# sw1:");
                for(uint8_t i = 0; i < 8; ++i) {
                    IO::out<debug>(" sw[", i, "]:", (uint8_t)pulse_in::mSwChannels[0].values[i]);
                }
                IO::outl<debug>();
#endif
            });
            update();
            (++mTelemTick).on(telemTicks, []{
                using bus_buf = relay_bus::messageBuffer;
                uint64_t s{};
                for(uint8_t i = 0; i < 8; ++i) {
                    if (pulse_in::mSwChannels[0].values[i] == 1) {
                        s |= (uint64_t{0b01} << (4 * i));
                    }
                    else if (pulse_in::mSwChannels[0].values[i] == 2) {
                        s |= (uint64_t{0b01} << (4 * i + 2));
                    }
                }
                for(uint8_t i = 0; i < 8; ++i) {
                    if (pulse_in::mSwChannels[1].values[i] == 1) {
                        s |= (uint64_t{0b01} << (4 * i + 32));
                    }
                    else if (pulse_in::mSwChannels[1].values[i] == 2) {
                        s |= (uint64_t{0b01} << (4 * i + 2 + 32));
                    }
                }
                if (s != swichtes) {
                    swichtes = s;
                    bus_buf::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::Command, [](auto& d){
                        d.push_back((uint8_t)0xc8);
                        d.push_back(RC::Protokoll::Crsf::V4::Address::Handset);
                        d.push_back(RC::Protokoll::Crsf::V4::CommandType::Switch);
                        d.push_back(RC::Protokoll::Crsf::V4::SwitchCommand::Set4M);
                        d.push_back((uint8_t)4);
                        d.push_back((uint8_t)240);
                        uint16_t sw = swichtes & 0xffff;
                        d.push_back(sw);
                        d.push_back((uint8_t)241);
                        sw = (swichtes >> 16) & 0xffff;
                        d.push_back(sw);
                        d.push_back((uint8_t)242);
                        sw = (swichtes >> 32) & 0xffff;
                        d.push_back(sw);
                        d.push_back((uint8_t)243);
                        sw = (swichtes >> 48) & 0xffff;
                        d.push_back(sw);
                    });
                }
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
                led2::event(led2::Event::Steady);
                break;
            case State::I2CScan:
                IO::outl<debug>("# I2C Scan");
                if (i2c1::scan([](const Mcu::Stm::I2C::Address a){
                               static uint8_t n = 0;
                               IO::outl<debug>("# I2C1: ", a.value);
                               switches1::activate(a);
                               crsf_cb::setI2CDev(0, n++, a.value);
                            })) {
                    IO::outl<debug>("# i2c1 scan start");
                }
                else {
                    IO::outl<debug>("# i2c1 scan failed");
                }
                if (i2c2::scan([](const Mcu::Stm::I2C::Address a){
                               static uint8_t n = 0;
                               IO::outl<debug>("# I2C2: ", a.value);
                               switches2::activate(a);
                               crsf_cb::setI2CDev(1, n++, a.value);
                            })) {
                    IO::outl<debug>("# i2c2 scan start");
                }
                else {
                    IO::outl<debug>("# i2c2 scan failed");
                }
                break;
            case State::Run:
                led1::event(led1::Event::Slow);
                IO::outl<debug>("# Run");
                break;
            }
        }
    }

    static inline void setSw(const uint8_t index, const bool state) {
        bt::setLed(index, state);
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
                dist_sb::set(storage::eeprom.analogMaps[i].position + i, adc::values()[i + off]);
            }
        }
        else if (storage::eeprom.analogMaps[i].stream == Stream::VControls) {
            for(uint8_t i = 0; i < 3; ++i) {
                dist_hw::set(storage::eeprom.analogMaps[i].position + i, adc::values()[i + off]);
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
                dist_sb::set(storage::eeprom.smMaps[i].position + i, f(i));
            }
        }
        else if (storage::eeprom.smMaps[i].stream == Stream::VControls) {
            for(uint8_t i = 0; i < 6; ++i) {
                dist_hw::set(storage::eeprom.smMaps[i].position + i, f(i));
            }
        }
    }
    static inline void updateSMs() {
        updateSM(0, [](const uint8_t i){return sm1::value(i);});
        updateSM(1, [](const uint8_t i){return sm2::value(i);});
    }
    static inline void updateInc() {
        if (storage::eeprom.incMaps[0].stream == Stream::Trainer) {
            dist_sb::set(storage::eeprom.incMaps[0].position, enc1::value());
        }
        else if (storage::eeprom.incMaps[0].stream == Stream::VControls) {
            dist_hw::set(storage::eeprom.incMaps[0].position, enc1::value());
        }
        if (storage::eeprom.incMaps[1].stream == Stream::Trainer) {
            dist_sb::set(storage::eeprom.incMaps[1].position + 1, enc2::value());
        }
        else if (storage::eeprom.incMaps[1].stream == Stream::VControls) {
            dist_hw::set(storage::eeprom.incMaps[1].position + 1, enc2::value());
        }
    }
    static inline void updateCrsf() {
        if (storage::eeprom.crsf_in > 0) {
            if (storage::eeprom.crsfInMap.stream == Stream::Trainer) {
                for(uint8_t i = 0; i < storage::eeprom.crsfInMap.count; ++i) {
                    dist_sb::set(storage::eeprom.crsfInMap.position + i, crsf_in_pa::values()[i]);
                }
            }
            else if (storage::eeprom.crsfInMap.stream == Stream::VControls) {
                for(uint8_t i = 0; i < storage::eeprom.crsfInMap.count; ++i) {
                    dist_hw::set(storage::eeprom.crsfInMap.position + i, crsf_in_pa::values()[i]);
                }
            }
        }
    }
    static inline void updateBluetooth() {
        if (storage::eeprom.bluetooth > 0) {
            if (storage::eeprom.bluetoothMap.stream == Stream::Trainer) {
                for(uint8_t i = 0; i < storage::eeprom.bluetoothMap.count; ++i) {
                    dist_sb::set(storage::eeprom.bluetoothMap.position + i, bt::values()[i]);
                }
            }
            else if (storage::eeprom.bluetoothMap.stream == Stream::VControls) {
                for(uint8_t i = 0; i < storage::eeprom.bluetoothMap.count; ++i) {
                    dist_hw::set(storage::eeprom.bluetoothMap.position + i, bt::values()[i]);
                }
            }
        }
    }
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mTelemTick;
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
