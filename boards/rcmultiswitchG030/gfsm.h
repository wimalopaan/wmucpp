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
    using debug = devs::debug;
    using storage = devs::storage;
    using systemTimer = devs::systemTimer;
    using crsf = devs::crsf;
    using crsf_out = crsf::output;
    using crsf_pa = crsf::input;
    using crsfCallback = crsf::callback;
    using led = devs::ledBlinker;
    using btn = devs::btn;
    using adc = devs::adc;
    using telemetry = devs::telemetry;
    using in0 = devs::in0;
    using in1 = devs::in1;

    using bsws = devs::bsws;

    enum class State : uint8_t {Undefined, Init, CheckBaudrate,
                                RunNoTelemetry, RunWithTelemetry,
                                NotConnected};

    enum class Event : uint8_t {None,
                                ConnectionLost, DirectConnected, ReceiverConnected,
                                ButtonPress
                               };

    static inline constexpr External::Tick<systemTimer> packagesCheckTicks{300ms};
    static inline constexpr External::Tick<systemTimer> initTicks{100ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> baudCheckTicks{1000ms};
    static inline constexpr External::Tick<systemTimer> telemTicks{100ms};

    static inline void update(const bool eepromMode = true) {
        crsfCallback::callbacks(eepromMode);
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
        using periodics = Meta::List<crsf, debug>;
        Meta::visit<periodics>([](const auto w){
            decltype(w)::type::periodic();
        });
    }
    static inline void ratePeriodic() {
        using ratePeriodics = Meta::concat<bsws, Meta::List<crsf, led, btn, telemetry>>;
        Meta::visit<ratePeriodics>([](const auto w){
            decltype(w)::type::ratePeriodic();
        });

        checkPackages();
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
                if (storage::eeprom.telemetry) {
                    mState = State::RunWithTelemetry;
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            });
            break;
        case State::CheckBaudrate:
            mEvent.on(Event::ReceiverConnected, []{
                if (storage::eeprom.telemetry) {
                    mState = State::RunWithTelemetry;
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            }).thenOn(Event::DirectConnected, []{
                if (storage::eeprom.telemetry) {
                    mState = State::RunWithTelemetry;
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            });
            mStateTick.on(baudCheckTicks, []{
                nextBaudrate();
            });
            break;
        case State::RunNoTelemetry:
            mStateTick.on(debugTicks, []{
                IO::outl<debug>("# ch0: ", crsf_pa::value(0), " cp: ", crsf_pa::template channelPackages<false>(), " lp: ", crsf_pa::template linkPackages<false>());
            });
            mEvent.on(Event::ButtonPress, []{
                mState = State::RunWithTelemetry;
            }).thenOn(Event::ConnectionLost, []{
                mState = State::NotConnected;
            });
            break;
        case State::RunWithTelemetry:
            if constexpr(!std::is_same_v<telemetry, void>) {
                mStateTick.on(debugTicks, []{
                    if constexpr(!std::is_same_v<adc, void>) {
                        const int16_t t = Mcu::Stm::V4::adc2Temp((adc::values()[0] * devs::Vref_n) / devs::Vcal_n);
                        const uint16_t v = adc::values()[1] * (devs::r1 + devs::r2) * devs::Vref_n / (devs::Vref_d * 4096 * devs::r1 / 10); // 100mV
                        telemetry::voltage(v * 100);
                        telemetry::temp(t * 10);
                    }
                    uint8_t s = 0;
                    if constexpr(!std::is_same_v<in0, void>) {
                        if (!in0::read()) {
                            s |= 0b0000'0001;
                        }
                    }
                    if constexpr(!std::is_same_v<in1, void>) {
                        if (!in1::read()) {
                            s |= 0b0000'0010;
                        }
                    }
                    IO::outl<debug>("# status: ", s);
                    telemetry::status(s);
                    telemetry::next();
                });
            }
            if (mEvent.is(Event::ButtonPress)) {
                if (storage::eeprom.telemetry) {
                    led::count(2);
                    led::event(led::Event::Slow);
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            }
            if (mEvent.is(Event::ConnectionLost)) {
                IO::outl<debug>("# E CL");
                mState = State::NotConnected;
            }
            break;
        case State::NotConnected:
            if (mEvent.is(Event::ReceiverConnected) || mEvent.is(Event::DirectConnected)) {
                if (storage::eeprom.telemetry) {
                    mState = State::RunWithTelemetry;
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            }
            else if (mEvent.is(Event::ConnectionLost)) {
                mState = State::CheckBaudrate;
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
            case State::CheckBaudrate:
                IO::outl<debug>("# Ck Baud");
                led::count(1);
                led::event(led::Event::Medium);
                nextBaudrate();
                break;
            case State::RunNoTelemetry:
                IO::outl<debug>("# Run NT");
                IO::outl<debug>("# adr: ", storage::eeprom.addresses[EEProm::AdrIndex::Switch]);
                crsf_out::enableReply(false);
                led::count(1);
                led::event(led::Event::Slow);
                break;
            case State::RunWithTelemetry:
                IO::outl<debug>("# Run WT");
                IO::outl<debug>("# adr: ", storage::eeprom.addresses[EEProm::AdrIndex::Switch]);
                if constexpr(!std::is_same_v<adc, void>) {
                    adc::start();
                }
                crsf_out::enableReply(true);
                if (storage::eeprom.telemetry) {
                    led::count(2);
                    led::event(led::Event::Slow);
                }
                else {
                    led::event(led::Event::Steady);
                }
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
    static inline void nextBaudrate() {
        static bool halfDuplex = false;
        if (crsf::nextBaudrate()) {
            halfDuplex = !halfDuplex;
            crsf::setHalfDuplex(halfDuplex);
        }
    }
    static inline void checkPackages() {
        (++mPackagesCheckTick).on(packagesCheckTicks, []{
            const uint16_t ch_p = crsf_pa::template channelPackages<true>();
            const uint16_t l_p = crsf_pa::template linkPackages<true>();
            const uint16_t p_p = crsf_pa::template pingPackages<true>();
            if ((ch_p > 0) || (p_p > 0) || (l_p > 0)) {
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
    }
    static inline void checkButton() {
        if constexpr(!std::is_same_v<btn, void>) {
            if (const auto e = btn::event(); e == btn::Press::Long) {
                event(Event::ButtonPress);
            }
        }
    }
    static inline etl::Event<Event> mEvent;
    static inline External::Tick<systemTimer> mPackagesCheckTick;
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState = State::Undefined;
};
