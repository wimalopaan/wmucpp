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
    using btn = devs::btn;
	using crsf = devs::crsf;
    using crsf_cb = devs::crsf_cb;
    using relay = devs::relay;

    enum class State : uint8_t {Undefined, Init, Run};
    enum class Event : uint8_t {None, ButtonPress};

    static inline constexpr External::Tick<systemTimer> initTicks{100ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> telemTicks{10ms};

    static inline void updateFromEeprom() {
        crsf_cb::callbacks(true);
        crsf_cb::update();
    }

    static inline void init() {
        devs::init();
    }
    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void periodic() {
		devs::periodic();
    }
    static inline void ratePeriodic() {
		devs::ratePeriodic();
        checkButton();

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
                mState = State::Run;
            });
            break;
        case State::Run:
			update();
			(++mDebugTick).on(debugTicks, []{
				// IO::outl<debug>("# p", crsf::input::channelPackages());
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
                break;
            case State::Run:
                IO::outl<debug>("# Run");
				led::count(2);
				led::event(led::Event::Slow);
                crsf::init();
                crsf::baud(RC::Protokoll::Crsf::V4::baudrate);
                relay::init();
                relay::baud(RC::Protokoll::Crsf::V4::baudrateHandset);
                relay::activateSource(true);
				relay::activateLinkStats(false);
				relay::activateChannels(false);
                break;
            }
        }
    }
    private:
	static inline void save() {
        if (const auto [ok, err] = Mcu::Stm32::savecfg(storage::eeprom, storage::eeprom_flash); ok) {
            IO::outl<debug>("# EEPROM OK");
        }
        else {
            IO::outl<debug>("# EEPROM NOK: ", err);
        }
    }
	static inline void update() {
		mStateTick.on(telemTicks, []{
		});		
	}
    static inline void checkButton() {
        if constexpr(!std::is_same_v<btn, void>) {
            if (const auto e = btn::event(); e == btn::Press::Long) {
                event(Event::ButtonPress);
            }
        }
    }
    static inline etl::Event<Event> mEvent;
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState = State::Undefined;
};
