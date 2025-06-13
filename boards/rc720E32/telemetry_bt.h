#pragma once

#include "tick.h"

namespace External::Bluetooth {
    using namespace std::literals::chrono_literals;

    template<typename Config>
    struct Telemetry {
        using debug = Config::debug;
        using timer = Config::timer;
        using dev1 = Config::dev1;
        using dev2 = Config::dev2;
        using storage = Config::storage;
        using sources = Config::sources;

        static inline constexpr External::Tick<timer> waitTicks{500ms};

        enum class State : u_int8_t {Init, Wait, HeartBeat, Values};

        static inline void ratePeriodic() {
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Init:
                mStateTick.on(waitTicks, []{
                    mState = State::Wait;
                });
                break;
            case State::Wait:
                mStateTick.on(waitTicks, []{
                    mState = State::HeartBeat;
                });
                break;
            case State::HeartBeat:
                mStateTick.on(waitTicks, []{
                    mState = State::Values;
                });
                break;
            case State::Values:
                mState = State::Wait;
                break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Init:
                    break;
                case State::Wait:
                    break;
                case State::HeartBeat:
                    if (dev1::isActive()) {
                        dev1::sendValue("hb", 1);
                    }
                    else if (dev2::isActive()) {
                        dev2::sendValue("hb", 1);
                    }
                    break;
                case State::Values:
                    sendNextValue();
                    break;
                }
            }
        }
        private:
        static inline void sendNextValue() {
            static u_int8_t item = 0;
            switch(item++) {
            case 0:
                if (dev1::isActive()) {
                    dev1::sendValue("volt0", sources::voltage(0));
                }
                else if (dev2::isActive()) {
                    dev2::sendValue("volt0", sources::voltage(0));
                }
                break;
            case 1:
                if (dev1::isActive()) {
                    dev1::sendValue("volt1", sources::voltage(1));
                }
                else if (dev2::isActive()) {
                    dev2::sendValue("volt1", sources::voltage(1));
                }
                break;
            case 2:
            {
                bool alarm = false;
                if ((sources::voltage(0) > 0) && (sources::voltage(0) < storage::eeprom.bt_telem_voltage_thresh)) {
                    alarm = true;
                }
                if ((sources::voltage(1) > 0) && (sources::voltage(1) < storage::eeprom.bt_telem_voltage_thresh)) {
                    alarm = true;
                }
                if (alarm) {
                    if (dev1::isActive()) {
                        dev1::sendString("alarm 800 200");
                    }
                    else if (dev2::isActive()) {
                        dev2::sendString("alarm 800 200");
                    }
                }
            }
                item = 0;
                break;
            default:
                item = 0;
                break;
            }
        }

        static inline State mState = State::Init;
        static inline External::Tick<timer> mStateTick;
    };
}
