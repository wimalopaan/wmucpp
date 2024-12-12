#pragma once

#include <chrono>
#include <utility>

#include "mcu/mcu.h"

#include "tick.h"
#include "rc/crsf.h"
#include "meta.h"

using namespace std::literals::chrono_literals;

template<typename Devices, typename Servos, typename Escs, typename Relays, typename Auxes>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using storage = devs::storage;

    using debug = devs::debug;

    using crsf_in = devs::crsf_in;
    using crsf_in_pa = crsf_in::adapter;
    using crsf_in_responder = crsf_in_pa::responder;
    using crsfBuffer = devs::crsfBuffer;

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

    static inline void init() {
        devs::init();
#ifdef CRSF_ADDRESS
        crsf_in_responder::address(std::byte(CRSF_ADDRESS));
#endif
        crsf_in_responder::telemetrySlot(0);
    }

    enum class Event : uint8_t {None, ConnectionLost, DirectConnected, ReceiverConnected};

    enum class State : uint8_t {Undefined, Init, Calib, RunConnected, RunUnconnected, DirectMode, CheckBaudrate};

    static inline void event(const Event e) {
        mEvent = e;
    }

    static inline void updateFromEeprom() {
        using cb = crsf_in_pa::CB;
        cb::callbacks(true);
    }

    static inline void periodic() {
        devs::tp1::set();
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
        crsf_in::periodic();
        Servos::periodic();
        Escs::periodic();
        Relays::periodic();
        Auxes::periodic();

        crsfBuffer::periodic();
        devs::tp1::reset();
    }

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> telemetryTicks{100ms};
    static inline constexpr External::Tick<systemTimer> updateTicks{20ms};
    static inline constexpr External::Tick<systemTimer> directTicks{1000ms};

    static inline constexpr External::Tick<systemTimer> packagesCheckTicks{300ms};
    static inline constexpr External::Tick<systemTimer> baudCheckTicks{1000ms};

    static inline void ratePeriodic() {
        led1::ratePeriodic();
        led2::ratePeriodic();
        crsf_in_pa::ratePeriodic([]{});
        Servos::ratePeriodic();
        Escs::ratePeriodic();
        Relays::ratePeriodic();
        Auxes::ratePeriodic();

        crsfBuffer::ratePeriodic();

        (++mPackagesCheckTick).on(packagesCheckTicks, []{
            const uint16_t ch_p = crsf_in_pa::template channelPackages<true>();
            const uint16_t l_p = crsf_in_pa::template linkPackages<true>();
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
            mState = State::Init;
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                if (adc::ready()) {
                    mState = State::Calib;
                }
                else {
                    mStateTick.reset();
                }
            });
            mState = State::RunUnconnected;
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
            (++mUpdateTick).on(updateTicks, []{
                channelCallback::update();
            });
            (++mTelemetryTick).on(telemetryTicks, []{
                telemetry::next();
            });
            mStateTick.on(debugTicks, []{
                // IO::outl<debug>("_end:", &_end, " _ebss:", &_ebss, " heap:", heap);
                IO::outl<debug>("ch0: ", crsf_in_pa::value(0), " phi: ", polar1::phi(), " amp: ", polar1::amp(), " a: ", Servos::actualPos(0), " t: ", Servos::turns(0));
                // IO::out<debug>("ibus: ec: ", ibus_in::errorCount(), " uc: ", ibus_in::uart::readCount(), " d0: ", ibus_in::uart::readBuffer()[0]);
                // for(uint8_t i = 0; i < 7; ++i) {
                //     IO::out<debug>(" ", ibus_in::value(i));
                // }
                // IO::outl<debug>(" ");
                // IO::outl<debug>(" sumdv3: cs: ");
                // IO::outl<debug>(" n p: ", crsf_in_pa::packages(), " n l:", crsf_in_pa::linkPackages());
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
                crsf_in_responder::setExtendedDestination(RC::Protokoll::Crsf::Address::Handset);
                crsf_in_responder::address(RC::Protokoll::Crsf::Address::TX);
                crsf_in_responder::sendRadioID();
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
                crsf_in_responder::address(std::byte(storage::eeprom.address));
                led1::event(led1::Event::Slow);
                led2::event(led2::Event::Off);
                adc::start();
                break;
            case State::DirectMode:
                IO::outl<debug>("# DMode");
                led1::event(led1::Event::Fast);
                led2::event(led2::Event::Fast);
                break;
            }
        }
    }

    private:
    static inline void nextBaudrate() {
        if (++mActiceBaudrateIndex >= mBaudrates.size()) {
            mActiceBaudrateIndex = 0;
        }
        IO::outl<debug>("# nextbaud: ", mBaudrates[mActiceBaudrateIndex]);
        crsf_in::baud(mBaudrates[mActiceBaudrateIndex]);
    }
    static inline uint8_t mActiceBaudrateIndex{0};
    static inline std::array<uint32_t, 2> mBaudrates{420'000, 921'000};
    static inline Event mEvent = Event::None;
    static inline External::Tick<systemTimer> mPackagesCheckTick;
    static inline External::Tick<systemTimer> mDirectTick;
    static inline External::Tick<systemTimer> mUpdateTick;
    static inline External::Tick<systemTimer> mTelemetryTick;
    static inline External::Tick<systemTimer> mStateTick;
    static inline State mState{State::Undefined};
};
