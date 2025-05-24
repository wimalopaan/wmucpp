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
#include "uuid.h"
#include "i2c.h"
#include "tick.h"
#include "rc/crsf_2.h"

#include "compass.h"

using namespace std::literals::chrono_literals;

template<typename Devices, typename Servos, typename Escs, typename Relays, typename Auxes>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using storage = devs::storage;

    using debug = devs::debug;

    using crsf_in = devs::crsf_in;
    using crsf_in_pa = crsf_in::input;
    using crsf_in_responder = crsf_in::output;
    using crsfBuffer = devs::crsfBuffer;
    using crsf_cb = crsf_in::callback;

    using led1 = devs::ledBlinker1;
    using led2 = devs::ledBlinker2;

    using adc = devs::adc;

    using channelCallback = devs::channelCallback;

    using telemetry = devs::telem;

    using polar1 = devs::polar1;
    using polar2 = devs::polar2;

    using pulse_in = devs::pulse_in;
    using ibus_in = devs::ibus_in;
    using sumdv3_in = devs::sumdv3_in;

    using esc32ascii_1 = devs::esc32ascii_1;
    using esc32ascii_2 = devs::esc32ascii_2;

    using srv1_waveshare = devs::srv1_waveshare;
    using srv2_waveshare = devs::srv2_waveshare;

    using i2c = devs::i2c;
    using magnetometer = devs::qmc5883l;

    struct CalibClient {
        static inline void update() {
            event(Event::CompassCalibUpdate);
        }
        static inline void start() {
            event(Event::CompassCalibStart);
        }
        static inline void end() {
            event(Event::CompassCalibEnd);
        }
    };
    using compass = Compass<magnetometer, systemTimer, CalibClient>;

    using sbus_aux = devs::sbus_aux;
    using gps_aux = devs::gps_aux;

    static inline void init() {
        devs::init();
#ifdef CRSF_ADDRESS
        crsf_in::address(std::byte(CRSF_ADDRESS));
#endif
        crsf_in_responder::telemetrySlot(0);
    }

    enum class Event : uint8_t {None, ConnectionLost, DirectConnected, ReceiverConnected,
                                CompassCalibStart, CompassCalibEnd, CompassCalibUpdate};

    enum class State : uint8_t {Undefined, Init,
                                Calib,
                                CompassCalib, CompassCalibUdated,
                                I2CScan,
                                RunConnected, RunUnconnected, DirectMode, CheckBaudrate};

    static inline void event(const Event e) {
        mEvent = e;
    }

    static inline void updateFromEeprom() {
        using cb = crsf_in::callback;
        cb::callbacks(true);
    }

    static inline void periodic() {
        // devs::tp1::set();
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
        i2c::periodic();
        compass::periodic();
        crsf_in::periodic();
        Servos::periodic();
        Escs::periodic();
        Relays::periodic();
        Auxes::periodic();
        // devs::tp1::reset();
    }

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> telemetryTicks{20ms};
    static inline constexpr External::Tick<systemTimer> updateTicks{20ms};
    static inline constexpr External::Tick<systemTimer> directTicks{1000ms};

    static inline constexpr External::Tick<systemTimer> packagesCheckTicks{300ms};
    static inline constexpr External::Tick<systemTimer> baudCheckTicks{1000ms};
    static inline constexpr External::Tick<systemTimer> gpsCheckTicks{3000ms};

    static inline constexpr External::Tick<systemTimer> calibLedTicks{100ms};

    static inline void ratePeriodic() {
        led1::ratePeriodic();
        led2::ratePeriodic();
        i2c::ratePeriodic();
        compass::ratePeriodic();
        crsf_in::ratePeriodic();
        telemetry::ratePeriodic();
        Servos::ratePeriodic();
        Escs::ratePeriodic();
        Relays::ratePeriodic();
        Auxes::ratePeriodic();

        (++mPackagesCheckTick).on(packagesCheckTicks, []{
            const uint16_t ch_p = crsf_in_pa::template channelPackages<true>();
            const uint16_t l_p = crsf_in_pa::template linkPackages<true>();
            const uint16_t sb = sbus_aux::packageCount();
            // IO::outl<debug>("ch_p:", ch_p, " l_p:", l_p, " sb:", sb);
            if ((ch_p > 0) || (sb > 0)) {
                if  ((l_p == 0) && (sb == 0)) {
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
                if (adc::ready()) {
                    // mState = State::I2CScan;
                    // mState = State::Calib;
                }
                else {
                    mStateTick.reset();
                }
            });
            mState = State::I2CScan;
            break;
        case State::Calib:
            mState = State::RunUnconnected;
            break;
        case State::CheckBaudrate:
            if (const Event e = std::exchange(mEvent, Event::None); e == Event::ReceiverConnected) {
                mState = State::RunConnected;
            }
            else if (e == Event::DirectConnected) {
                mState = State::DirectMode;
            }
            mStateTick.on(baudCheckTicks, []{
                nextBaudrate();
            });
            break;
        case State::RunUnconnected:
            if (const Event e = std::exchange(mEvent, Event::None); e == Event::ReceiverConnected) {
                mState = State::RunConnected;
            }
            else if (e == Event::DirectConnected) {
                mState = State::DirectMode;
            }
            else if (e == Event::ConnectionLost) {
                mState = State::CheckBaudrate;
            }
            // (++mUpdateTick).on(updateTicks, []{
            //     channelCallback::update();
            // });
            // mStateTick.on(debugTicks, []{
            //     IO::outl<debug>("sbus aux: ", sbus_aux::value(0));
            // });
            break;
        case State::RunConnected:
            if (const Event e = std::exchange(mEvent, Event::None); e == Event::ConnectionLost) {
                mState = State::RunUnconnected;
            }
            else if (e == Event::DirectConnected) {
                mState = State::DirectMode;
            }
            else if (e == Event::ConnectionLost) {
                mState = State::CheckBaudrate;
            }
            else if (e == Event::CompassCalibStart) {
                led1::event(led1::Event::Medium);
                mState = State::CompassCalib;
            }
            (++mUpdateTick).on(updateTicks, []{
                channelCallback::update();
            });
            (++mTelemetryTick).on(telemetryTicks, []{
                telemetry::push((uint8_t)RC::Protokoll::Crsf::V4::Type::Gps, [](auto& d){
                        d.push_back((int32_t)gps_aux::ublox::mLatitude);
                        d.push_back((int32_t)gps_aux::ublox::mLongitude);
                        d.push_back((uint16_t)gps_aux::ublox::mSpeed);
                        d.push_back(int16_t((compass::a() - 180) * 100));
                        d.push_back(uint16_t(gps_aux::ublox::mAltitude / 1000 + 1000));
                        d.push_back((uint8_t)gps_aux::ublox::mSatCount);
                });
                telemetry::next();
            });
            (++mGpsCheckTick).on(gpsCheckTicks, []{
                if (gps_aux::packages() == 0) {
                    gps_aux::nextBaudrate();
                }
            });
            mStateTick.on(debugTicks, []{
                const int16_t a = compass::a();
                IO::outl<debug>("# x: ", compass::x(), " y: ", compass::y(), " z: ", compass::z(), " a: ", a);
                // IO::outl<debug>("# gps pkg: ", gps_aux::packages(), " RMC t: ", gps_aux::rmc::mTime,
                //                 " sat: ", gps_aux::gsv::mSatCount,
                //                 " lat: ", gps_aux::rmc::mLatitude, " lon: ", gps_aux::rmc::mLongitude, " date: ", gps_aux::rmc::mDate,
                //                 " speed: ", gps_aux::vtg::mSpeed, " head: ", gps_aux::vtg::mHeading / 100);
                // IO::outl<debug>("# ublox npkg: ", gps_aux::ublox::mNavPackages, " stat pkg: ", gps_aux::ublox::mStatusPackages, " flags: ", gps_aux::ublox::mFlags, " lon: ", gps_aux::ublox::mLongitude, " lat: ", gps_aux::ublox::mLatitude,
                //                 " speed: ", gps_aux::ublox::mSpeed,
                //                 " head: ", gps_aux::ublox::mHeading / 100, " headM: ", gps_aux::ublox::mHeadingM / 100);
                // IO::outl<debug>("# sbus aux: ", sbus_aux::value(0));
                // IO::outl<debug>("_end:", &_end, " _ebss:", &_ebss, " heap:", heap);
                // IO::outl<debug>("ch0: ", crsf_in_pa::value(0), " phi0: ", polar1::phi(), " amp0: ", polar1::amp(), " a0: ", Servos::actualPos(0), " t0: ", Servos::turns(0), " phi1: ", polar2::phi(), " amp1: ", polar2::amp(), " a1: ", Servos::actualPos(1), " t1: ", Servos::turns(1));
            });
            break;
        case State::DirectMode:
            if (const Event e = std::exchange(mEvent, Event::None); e == Event::ConnectionLost) {
                mState = State::RunUnconnected;
            }
            else if (e == Event::ReceiverConnected) {
                mState = State::RunUnconnected;
            }
            else if (e == Event::ConnectionLost) {
                mState = State::CheckBaudrate;
            }
            (++mUpdateTick).on(updateTicks, []{
                channelCallback::update();
            });
            (++mDirectTick).on(directTicks, []{
                crsf_in_responder::setDestination(RC::Protokoll::Crsf::V4::Address::Handset);
                crsf_in::address(RC::Protokoll::Crsf::V4::Address::TX);
                crsf_in_responder::sendRadioID();
                // IO::outl<debug>("# send Radio ID");
            });
            break;
        case State::CompassCalib:
            if (const Event e = std::exchange(mEvent, Event::None); e == Event::CompassCalibEnd) {
                mState = State::RunConnected;
            }
            else if (e == Event::CompassCalibUpdate) {
                mState = State::CompassCalibUdated;
            }
            mStateTick.on(debugTicks, []{
                compass::template debugInfo<debug>();
            });
            break;
        case State::CompassCalibUdated:
            mStateTick.on(calibLedTicks, []{
                mState = State::CompassCalib;
            });
            break;
        case State::I2CScan:
            if (i2c::isIdle()) {
                mState = State::RunUnconnected;
            }
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                IO::outl<debug>("# Undef");
                break;
            case State::Init:
                IO::outl<debug>("# Init uuid: ", uuid);
                led1::event(led1::Event::Steady);
                break;
            case State::Calib:
                IO::outl<debug>("# Calib");
                // servo1::event(servo1::Event::Calibrate);
                // servo1::event(servo1::Event::Run);
                break;
            case State::CheckBaudrate:
                IO::outl<debug>("# Ck Baud");
                led1::event(led1::Event::Steady);
                led2::event(led2::Event::Steady);
                nextBaudrate();
                break;
            case State::RunUnconnected:
                IO::outl<debug>("# Run Unc");
                channelCallback::update();
                led1::event(led1::Event::Fast);
                led2::event(led2::Event::Off);
                break;
            case State::RunConnected:
                IO::outl<debug>("# Run con");
                crsf_in::address(std::byte(storage::eeprom.address));
                led1::event(led1::Event::Slow);
                led2::event(led2::Event::Off);
                adc::start();
                break;
            case State::DirectMode:
                IO::outl<debug>("# DMode");
                led1::event(led1::Event::Fast);
                led2::event(led2::Event::Fast);
                break;
            case State::CompassCalib:
                IO::outl<debug>("# CompassCalib");
                led2::event(led2::Event::Off);
                break;
            case State::CompassCalibUdated:
                IO::outl<debug>("# CompassCalibUpdated");
                led2::event(led2::Event::Steady);
                break;
            case State::I2CScan:
                IO::outl<debug>("# I2CScan");
                if (i2c::scan([](const Mcu::Stm::I2C::Address a){
                              static uint8_t n = 0;
                              IO::outl<debug>("I2C: ", a.value);
                              crsf_cb::setI2CDev(n++, a.value);
                            })) {
                    IO::outl<debug>("# i2c scan start");
                }
                else {
                    IO::outl<debug>("# i2c scan failed");
                }
                break;
            }
        }
    }
    private:
    static inline void nextBaudrate() {
        crsf_in::nextBaudrate();
    }
    static inline const uint32_t uuid = Mcu::Stm::Uuid::get();
    static inline Event mEvent = Event::None;
    static inline External::Tick<systemTimer> mPackagesCheckTick;
    static inline External::Tick<systemTimer> mGpsCheckTick;
    static inline External::Tick<systemTimer> mDirectTick;
    static inline External::Tick<systemTimer> mUpdateTick;
    static inline External::Tick<systemTimer> mTelemetryTick;
    static inline External::Tick<systemTimer> mStateTick;
    static inline State mState{State::Undefined};
};
