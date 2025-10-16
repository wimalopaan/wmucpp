/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

using namespace std::literals::chrono_literals;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using storage = devs::storage;
    using crsf = devs::crsf;
    using crsf_out = crsf::output;
    using crsf_pa = crsf::input;
    using led = devs::ledBlinker;
    using tp = devs::tp;

    using debug = devs::debug;
    using crsfCallback = crsf::callback;

    enum class State : uint8_t {Undefined, Init,
                                RunWithTelemetry,
                                NotConnected};

    enum class Event : uint8_t {None, ConnectionLost, DirectConnected, ReceiverConnected};

    static inline constexpr External::Tick<systemTimer> packagesCheckTicks{300ms};
    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};

    static inline void update(const bool eepromMode = true) {
        IO::outl<debug>("# update");
        // crsfCallback::callbacks(eepromMode);
    }
    static inline void init() {
        devs::init();
        if constexpr(!std::is_same_v<debug, void>) {
            debug::init();
        }
    }
    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void periodic() {
        // tp::toggle();
        crsf::periodic();
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
    }
    static inline void ratePeriodic() {
        crsf::ratePeriodic();
        led::ratePeriodic();
        // (++mPackagesCheckTick).on(packagesCheckTicks, []{
        //     const uint16_t ch_p = crsf_pa::template channelPackages<true>();
        //     const uint16_t l_p = crsf_pa::template linkPackages<true>();
        //     if (ch_p > 0) {
        //         if  (l_p == 0) {
        //             event(Event::DirectConnected);
        //         }
        //         else {
        //             event(Event::ReceiverConnected);
        //         }
        //     }
        //     else {
        //         event(Event::ConnectionLost);
        //     }
        // });

        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                mState = State::RunWithTelemetry;
            });
            break;
        case State::RunWithTelemetry:
            mStateTick.on(debugTicks, []{
                IO::outl<debug>("# ch0: ", crsf_pa::value(0), " psize: ", crsfCallback::numberOfParameters(), " cp: ", crsf_pa::template channelPackages<false>(), " lp: ", crsf_pa::template linkPackages<false>());
            });
            if (mEvent.is(Event::ConnectionLost)) {
                IO::outl<debug>("# E CL");
                mState = State::NotConnected;
            }
            break;
        case State::NotConnected:
            if (mEvent.is(Event::ReceiverConnected) || mEvent.is(Event::DirectConnected)) {
                    mState = State::RunWithTelemetry;
            }
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                IO::outl<debug>("# Init eep magic: ", storage::eeprom.magic);
                break;
            case State::RunWithTelemetry:
                IO::outl<debug>("# Run WT");
                crsf::baud(420'000);
                crsf_out::enableReply(true);
                led::count(1);
                led::event(led::Event::Fast);
                break;
            case State::NotConnected:
                IO::outl<debug>("# Run NC");
                led::count(1);
                led::event(led::Event::Fast);
                break;
            }
        }
    }
    private:
    static inline etl::Event<Event> mEvent;
    static inline External::Tick<systemTimer> mPackagesCheckTick;
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState = State::Undefined;
};

