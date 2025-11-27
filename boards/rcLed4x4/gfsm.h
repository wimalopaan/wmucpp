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
#ifdef USE_BUTTON
    using btn = devs::btn;
#endif
    using pca = devs::pca9745;
    using debug = devs::debug;
    using crsfCallback = crsf::callback;
	
	using adc = devs::adc;
	using telemetry = devs::telemetry;

    enum class State : uint8_t {Undefined, Init,
                                RunNoTelemetry, RunWithTelemetry,
                                NotConnected};

    enum class Event : uint8_t {None, ConnectionLost, DirectConnected, ReceiverConnected};

    static inline constexpr External::Tick<systemTimer> packagesCheckTicks{300ms};
    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};

    static inline void update(const bool eepromMode = true) {
        IO::outl<debug>("# update");
        crsfCallback::callbacks(eepromMode);

        // pca::ledIRef(0, 16);
        // pca::ledPwm(0, 16);
        // pca::ledControl(0, 1);

        // pca::groupIRef(0, 255);
        // pca::groupHoldOnTime(0, 4);
        // pca::groupHoldOffTime(0, 4);
        // pca::groupRampUp(0, true);
        // pca::groupRampDown(0, true);
        // pca::groupRampRate(0, 1);
        // pca::groupStepTime(0, 0);
        // pca::ledGradationMode(0, true);
        // pca::groupContinous(0, true);
        // pca::groupStart(0);
        // pca::exponentialBrightness(true);
    }
    static inline void prop(const uint8_t channel, const uint8_t duty) {
        IO::outl<debug>("# prop: ", channel, " duty: ", duty);
    }
    static inline void set(const uint8_t sw) {
        IO::outl<debug>("# set: ", sw);
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
        crsf::periodic();
        pca::periodic();
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
    }
    static inline void ratePeriodic() {
        crsf::ratePeriodic();
        led::ratePeriodic();
        pca::ratePeriodic();
#ifdef USE_BUTTON
        btn::ratePeriodic();
#endif
        (++mPackagesCheckTick).on(packagesCheckTicks, []{
            const uint16_t ch_p = crsf_pa::template channelPackages<true>();
            const uint16_t l_p = crsf_pa::template linkPackages<true>();
            if (ch_p > 0) {
                if  (l_p == 0) {
                    event(Event::DirectConnected);
                }
                else {
                    event(Event::ReceiverConnected);
                }
            }
            else {
                event(Event::ConnectionLost);
            }
        });

        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
#ifdef HW_LED2
			mState = State::RunWithTelemetry;
#else
			mStateTick.on(initTicks, []{
                if (storage::eeprom.telemetry) {
                    mState = State::RunWithTelemetry;
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            });
#endif
            break;
        case State::RunNoTelemetry:
            mStateTick.on(debugTicks, []{
                IO::outl<debug>("# ch0: ", crsf_pa::value(0), " cp: ", crsf_pa::template channelPackages<false>(), " lp: ", crsf_pa::template linkPackages<false>());
                // IO::outl<debug>("# ch0: ", crsf_pa::value(0));
            });
#ifdef USE_BUTTON
            if (const auto e = btn::event(); e == btn::Press::Long) {
                mState = State::RunWithTelemetry;
            }
#endif
            if (mEvent.is(Event::ConnectionLost)) {
                mState = State::NotConnected;
            }
            break;
        case State::RunWithTelemetry:
			mStateTick.on(debugTicks, []{
				IO::outl<debug>("# ch0: ", crsf_pa::value(0));
				if constexpr(!std::is_same_v<adc, void>) {
					const int16_t t = Mcu::Stm::V4::adc2Temp((adc::values()[0] * devs::Vref_n) / devs::Vcal_n);
					const uint16_t v = adc::values()[1] * (devs::r1 + devs::r2) * devs::Vref_n / (devs::Vref_d * 4096 * devs::r1 / 10); // 100mV
					telemetry::voltage(v * 100);
					telemetry::temp(t * 10);
				}
				telemetry::next();
			});
#ifdef USE_BUTTON
            if (const auto e = btn::event(); e == btn::Press::Long) {
                if (storage::eeprom.telemetry) {
                    led::count(2);
                    led::event(led::Event::Slow);
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            }
#endif
            if (mEvent.is(Event::ConnectionLost)) {
                IO::outl<debug>("# E CL");
                mState = State::NotConnected;
            }
            break;
        case State::NotConnected:
            if (mEvent.is(Event::ReceiverConnected) || mEvent.is(Event::DirectConnected)) {
#ifdef HW_LED2
				mState = State::RunWithTelemetry;
#else
				
				if (storage::eeprom.telemetry) {
                    mState = State::RunWithTelemetry;
                }
                else {
                    mState = State::RunNoTelemetry;
                }
#endif
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
            case State::RunNoTelemetry:
                IO::outl<debug>("# Run NT");
                IO::outl<debug>("# adr: ", storage::eeprom.address1);
                crsf_out::enableReply(false);
                led::count(1);
                led::event(led::Event::Slow);
                break;
            case State::RunWithTelemetry:
                IO::outl<debug>("# Run WT");
                IO::outl<debug>("# adr: ", storage::eeprom.address1);
                crsf_out::enableReply(true);
#ifdef HW_LED2
				led::count(2);
				led::event(led::Event::Slow);
#else
				if (storage::eeprom.telemetry) {
                    led::count(2);
                    led::event(led::Event::Slow);
                }
                else {
                    led::event(led::Event::Steady);
                }
#endif
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

