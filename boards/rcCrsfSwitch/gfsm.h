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
#include "tick.h"
#include "rc/crsf_2.h"

using namespace std::literals::chrono_literals;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using debug = devs::debug;
    using storage = devs::storage;

    using led = devs::ledBlinker;

    using crsf_in = devs::crsf_in;

    using crsf_hd1 = devs::crsf_hd1;
    using crsf_hd2 = devs::crsf_hd2;
    using crsf_hd3 = devs::crsf_hd3;
    using crsf_hd5 = devs::crsf_hd5;

    using crsf_ifaces = devs::crsf_ifaces;

    enum class Event : uint8_t {None, ConnectionLost, DirectConnected, ReceiverConnected};

    enum class State : uint8_t {Undefined, Init,
                                RunConnected, RunUnconnected, DirectMode,
                                CheckBaudrate,
                                UpdateRouting};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> telemetryTicks{20ms};
    static inline constexpr External::Tick<systemTimer> updateTicks{20ms};
    static inline constexpr External::Tick<systemTimer> directTicks{1000ms};
    static inline constexpr External::Tick<systemTimer> packagesCheckTicks{300ms};
    static inline constexpr External::Tick<systemTimer> baudCheckTicks{1000ms};
    static inline constexpr External::Tick<systemTimer> pingTicks{500ms};

    static inline auto rcPacket = []{
        std::array<uint8_t, 26> d{};
        std::array<uint16_t, 16> a{};
        std::fill(std::begin(a), std::end(a), RC::Protokoll::Crsf::V4::mid);
        RC::Protokoll::Crsf::V4::pack(a, d);
        return d;
    }();
    static inline void sendRCFrame() {
        Meta::visit<crsf_ifaces>([](auto P) static {decltype(P)::type::forwardPacket(&rcPacket[0], rcPacket.size());});
    }

    static inline std::array<uint8_t, 6> pingPacket {0xc8, 0x04, 0x28, 0x00, 0xea, 0x54};
    static inline uint8_t pingCounter = 0;
    static inline constexpr uint8_t maxPingCount = 5;

    static inline void sendPings() {
        Meta::visit<crsf_ifaces>([](auto P) static {decltype(P)::type::forwardPacket(&pingPacket[0], pingPacket.size());});
    }
    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void init() {
        devs::init();
    }

    using periodics = Meta::concat<crsf_ifaces, Meta::List<debug, crsf_in, led>>;

    static inline void periodic() {
        Meta::visit<periodics>([](auto P) static {decltype(P)::type::periodic(); });
    }
    static inline void ratePeriodic() {
        Meta::visit<periodics>([](auto P) static {decltype(P)::type::ratePeriodic(); });

        (++mPackagesCheckTick).on(packagesCheckTicks, []{
            const uint16_t ch_p = crsf_in::input::template channelPackages<true>();
            const uint16_t l_p = crsf_in::input::template linkPackages<true>();
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
        });

        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                pingCounter = 0;
                mState = State::UpdateRouting;
            });
            break;
        case State::UpdateRouting:
            if (pingCounter > maxPingCount) {
                mState = State::CheckBaudrate;
            }
            (++mPingTick).on(pingTicks, []{
                sendPings();
                ++pingCounter;
            });
            (++mUpdateTick).on(updateTicks, []{
                sendRCFrame();
            });
            break;
        case State::CheckBaudrate:
            mEvent.on(Event::ReceiverConnected, []{
                mState = State::RunConnected;
            }).thenOn(Event::ReceiverConnected, []{
                mState = State::DirectMode;
            });
            mStateTick.on(baudCheckTicks, []{
                nextBaudrate();
            });
            (++mUpdateTick).on(updateTicks, []{
                sendRCFrame();
            });
            break;
        case State::RunUnconnected:
            mEvent.on(Event::ReceiverConnected, []{
                mState = State::RunConnected;
            }).thenOn(Event::DirectConnected, []{
                mState = State::DirectMode;
            }).thenOn(Event::ConnectionLost, []{
                mState = State::CheckBaudrate;
            });
            (++mUpdateTick).on(updateTicks, []{
                // channelCallback::update();
            });
            mStateTick.on(debugTicks, []{
            });
            break;
        case State::RunConnected:
            mEvent.on(Event::ConnectionLost, []{
                mState = State::RunUnconnected;
            }).thenOn(Event::DirectConnected, []{
                mState = State::DirectMode;
            });
            (++mUpdateTick).on(updateTicks, []{
            });
            mStateTick.on(debugTicks, []{
            });
            break;
        case State::DirectMode:
            mEvent.on(Event::ConnectionLost, []{
                mState = State::RunUnconnected;
            }).thenOn(Event::ReceiverConnected, []{
                mState = State::RunConnected;
            });
            (++mUpdateTick).on(updateTicks, []{
            });
            (++mDirectTick).on(directTicks, []{
                crsf_in::output::setDestination(RC::Protokoll::Crsf::V4::Address::Handset);
                crsf_in::address(RC::Protokoll::Crsf::V4::Address::TX);
                crsf_in::output::sendRadioID();
                // IO::outl<debug>("# send Radio ID");
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
                led::event(led::Event::Steady);
                break;
            case State::CheckBaudrate:
                IO::outl<debug>("# Ck Baud");
                led::event(led::Event::Steady);
                nextBaudrate();
                break;
            case State::RunUnconnected:
                IO::outl<debug>("# Run Unc");
                led::event(led::Event::Fast);
                break;
            case State::RunConnected:
                IO::outl<debug>("# Run con");
                crsf_in::address(std::byte(storage::eeprom.address));
                led::count(2);
                led::event(led::Event::Slow);
                break;
            case State::DirectMode:
                IO::outl<debug>("# DMode");
                led::event(led::Event::Fast);
                break;
            case State::UpdateRouting:
                IO::outl<debug>("# Ping");
                led::event(led::Event::Medium);
                break;
            }
        }
    }
    static inline void nextBaudrate() {
        crsf_in::nextBaudrate();
    }
    static inline etl::Event<Event> mEvent;
    static inline External::Tick<systemTimer> mPackagesCheckTick;
    static inline External::Tick<systemTimer> mDirectTick;
    static inline External::Tick<systemTimer> mUpdateTick;
    static inline External::Tick<systemTimer> mTelemetryTick;
    static inline External::Tick<systemTimer> mPingTick;
    static inline External::Tick<systemTimer> mStateTick;
    static inline State mState{State::Undefined};

};

