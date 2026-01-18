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

namespace Gfsm {
    template<typename Devices>
    struct WithTelemetry {
        using devs = Devices;
        using systemTimer = devs::systemTimer;
        using terminal = devs::terminal;
        using adc = devs::adcController;
        using adc_i_t = adc::index_type;
        using protocoll_adapter = devs::protocoll_adapter;
        using blinker = devs::blinker;
        using adr0 = devs::adr0;
        using adr1 = devs::adr1;
        using telemetry = devs::input::telemetry;
        using decoder = devs::input::commandDecoder;

        using in0 = devs::in0;

        enum class State : uint8_t {Undefined, Init, Connected, NotConnected};
        enum class Event : uint8_t {None, Connect, Unconnect};

        static inline void init() {
            devs::init();
        }
        static inline void periodic() {
            devs::periodic();
        }
        static inline void ratePeriodic() {
            devs::ratePeriodic();
            ++mStateTicks;
            (++mDebugTicks).on(debugTicks, [] static {
                                   if constexpr(!std::is_same_v<adc, void>) {
                                       etl::outl<terminal>("adc0: "_pgm, adc::value(adc_i_t{0}),
                                       "adc1: "_pgm, adc::value(adc_i_t{1}));
                                   }
                               });
            (++mCheckTicks).on(checkTicks, [] static {
                                   const uint8_t cp = protocoll_adapter::packages();
                                   if (cp > 0) {
                                       mEvent = Event::Connect;
                                   }
                                   else {
                                       mEvent = Event::Unconnect;
                                   }
                               });

            const auto oldState = mState;
            switch(mState) {
            case State::Undefined:
                mState = State::Init;
                break;
            case State::Init:
                mEvent.on(Event::Connect, []{
                    mState = State::Connected;
                }).thenOn(Event::Unconnect, []{
                    mState = State::NotConnected;
                });
                break;
            case State::Connected:
                mEvent.on(Event::Unconnect, []{
                    mState = State::NotConnected;
                });
                mStateTicks.on(telemetryTicks, []{
                    telemetry::voltage(adc::value(adc_i_t{0}));
                    telemetry::temperature(adc::value(adc_i_t{1}) / 10);
                    const uint8_t s = (in0::isActive() ? 0b0000'0001 : 0b0000'0000);
                    telemetry::status(s);
                });
                break;
            case State::NotConnected:
                mEvent.on(Event::Connect, []{
                    mState = State::Connected;
                });
                break;
            }
            if (oldState != mState) {
                mStateTicks.reset();
                switch(mState) {
                case State::Undefined:
                    break;
                case State::Init:
                    etl::outl<terminal>("Init"_pgm);
                    blinker::steady();
                {
                    const uint8_t a0 = adr0::isActive();
                    const uint8_t a1 = adr0::isActive();
                    const uint8_t address = DEFAULT_ADDRESS + (2 * a1 + a0) * 2;
                    decoder::address(address);
                    telemetry::address(address);
                    etl::outl<terminal>("Adr: "_pgm, decoder::address());
                }
                    break;
                case State::Connected:
                    etl::outl<terminal>("Connected"_pgm);
                    blinker::blink(typename blinker::count_type{2});
                    break;
                case State::NotConnected:
                    etl::outl<terminal>("NotConnected"_pgm);
                    blinker::blink(typename blinker::count_type{1});
                    break;
                }
            }
        }
        private:
        static constexpr External::Tick<systemTimer> debugTicks{500_ms};
        static constexpr External::Tick<systemTimer> telemetryTicks{100_ms};
        static constexpr External::Tick<systemTimer> checkTicks{3000_ms};
        inline static External::Tick<systemTimer> mStateTicks;
        inline static External::Tick<systemTimer> mCheckTicks;
        inline static External::Tick<systemTimer> mDebugTicks;
        inline static State mState = State::Undefined;
        inline static etl::Event<Event> mEvent{};
    };

    template<typename Devices>
    struct NoTelemetry {
        using devs = Devices;
        using systemTimer = devs::systemTimer;
        using terminal = devs::terminal;
        using protocoll_adapter = devs::protocoll_adapter;
        using blinker = devs::blinker;
        using adr0 = devs::adr0;
        using adr1 = devs::adr1;
        using decoder = devs::input::commandDecoder;

        enum class State : uint8_t {Undefined, Init, Connected, NotConnected};
        enum class Event : uint8_t {None, Connect, Unconnect};

        static inline void init() {
            devs::init();
        }
        static inline void periodic() {
            devs::periodic();
        }
        static inline void ratePeriodic() {
            devs::ratePeriodic();
            ++mStateTicks;
            (++mDebugTicks).on(debugTicks, [] static {
                               });
            (++mCheckTicks).on(checkTicks, [] static {
                                   const uint8_t cp = protocoll_adapter::packages();
                                   if (cp > 0) {
                                       mEvent = Event::Connect;
                                   }
                                   else {
                                       mEvent = Event::Unconnect;
                                   }
                               });

            const auto oldState = mState;
            switch(mState) {
            case State::Undefined:
                mState = State::Init;
                break;
            case State::Init:
                mEvent.on(Event::Connect, []{
                    mState = State::Connected;
                }).thenOn(Event::Unconnect, []{
                    mState = State::NotConnected;
                });
                break;
            case State::Connected:
                mEvent.on(Event::Unconnect, []{
                    mState = State::NotConnected;
                });
                break;
            case State::NotConnected:
                mEvent.on(Event::Connect, []{
                    mState = State::Connected;
                });
                break;
            }
            if (oldState != mState) {
                mStateTicks.reset();
                switch(mState) {
                case State::Undefined:
                    break;
                case State::Init:
                    etl::outl<terminal>("Init"_pgm);
                    blinker::steady();
                {
                    const uint8_t a0 = adr0::isActive();
                    const uint8_t a1 = adr0::isActive();
                    const uint8_t address = DEFAULT_ADDRESS + (2 * a1 + a0) * 2;
                    decoder::address(address);
                    etl::outl<terminal>("Adr: "_pgm, decoder::address());
                }
                    break;
                case State::Connected:
                    etl::outl<terminal>("Connected"_pgm);
                    blinker::blink(typename blinker::count_type{2});
                    break;
                case State::NotConnected:
                    etl::outl<terminal>("NotConnected"_pgm);
                    blinker::blink(typename blinker::count_type{1});
                    break;
                }
            }
        }
        private:
        static constexpr External::Tick<systemTimer> debugTicks{500_ms};
        static constexpr External::Tick<systemTimer> checkTicks{3000_ms};
        inline static External::Tick<systemTimer> mStateTicks;
        inline static External::Tick<systemTimer> mCheckTicks;
        inline static External::Tick<systemTimer> mDebugTicks;
        inline static State mState = State::Undefined;
        inline static etl::Event<Event> mEvent{};
    };
}
