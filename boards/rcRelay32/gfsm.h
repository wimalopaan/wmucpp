/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "rc/rc_2.h"
#include "etl/event.h"
#include <tick.h>

using namespace std::literals::chrono_literals;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using debug = devs::debug;
    using storage = devs::storage;
    using systemTimer = devs::systemTimer;
    using led = devs::ledBlinker;
	using crsf = devs::crsf;
    using crsf_in = devs::crsf_in;
    using crsf_cb = devs::crsf_cb;
    using relay = devs::relay;
    using rbuffer = relay::messageBuffer;

    enum class State : uint8_t {Undefined, Init, Run, UnConnected};
    enum class Event : uint8_t {None, ConnectionLost, DirectConnected, ReceiverConnected};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> packagesCheckTicks{300ms};

    static inline void updateFromEeprom() {
        crsf_cb::callbacks(true);
        crsf_cb::update();
    }
    static inline void init() {
        devs::init();
        updateFromEeprom();
    }
    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void periodic() {
		devs::periodic();
    }
    static inline void ratePeriodic() {
		devs::ratePeriodic();
        checkPackages();

        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mEvent.on(Event::ReceiverConnected, []{
                mState = State::Run;
                }).thenOn(Event::ConnectionLost, []{
                    mState = State::UnConnected;
            });
            break;
        case State::UnConnected:
            mEvent.on(Event::ReceiverConnected, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            mEvent.on(Event::ConnectionLost, []{
                mState = State::UnConnected;
            });
			(++mDebugTick).on(debugTicks, []{
                IO::outl<debug>("# p", crsf_in::channelPackages());
			});
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                IO::outl<debug>("# Init eep magic: ", storage::eeprom.magic);
                led::event(led::Event::Steady);
                break;
            case State::UnConnected:
                IO::outl<debug>("# UnCon");
                if (storage::eeprom.failsafe_mode == 0) {
                    relay::enable(false);
                }
                led::count(1);
                led::event(led::Event::Fast);
                break;
            case State::Run:
                IO::outl<debug>("# Run");
                relay::enable(true);
				led::count(2);
				led::event(led::Event::Slow);
                break;
            }
        }
    }
    private:
    static inline void checkPackages() {
        (++mPackagesCheckTick).on(packagesCheckTicks, []{
            const uint16_t ch_p = crsf_in::template channelPackages<true>();
            const uint16_t l_p = crsf_in::template linkPackages<true>();
            if ((ch_p > 0)) {
                if  ((l_p == 0)) {
                    event(Event::DirectConnected);
                }
                else {
                    event(Event::ReceiverConnected);
                }
            }
            else {
                event(Event::ConnectionLost);
            }
            const uint16_t fe = relay::uart::frameErrors();
            if (fe > 0) {
                IO::outl<debug>("# send baudrate");
                sendSetBaudrate();
            }
        });
    }
    static inline void sendSetBaudrate() {
        rbuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::Command, [&](auto& d){
            d.push_back(RC::Protokoll::Crsf::V4::Address::Broadcast);
            d.push_back(RC::Protokoll::Crsf::V4::Address::RX);
            d.push_back(RC::Protokoll::Crsf::V4::CommandType::general);
            d.push_back(RC::Protokoll::Crsf::V4::GeneralCommand::speed);
            d.push_back(uint8_t(0));
            d.push_back(RC::Protokoll::Crsf::V4::baudrateHandset);
        });
    }
    static inline etl::Event<Event> mEvent;
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline External::Tick<systemTimer> mPackagesCheckTick;
    static inline State mState = State::Undefined;
};
