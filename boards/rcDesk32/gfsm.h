#pragma once

#include <chrono>
#include <utility>

#include "mcu/mcu.h"
#include "meta.h"
#include "i2c.h"
#include "tick.h"

using namespace std::literals::chrono_literals;

template <typename Devs>
struct GFSM {
    using devs = Devs;
    using debug = devs::debug;
    using systemTimer = devs::systemTimer;
    // using storage = devs::storage;
    using i2c = devs::i2c3;

    using radio = devs::radio;

    using switches = External::Switches<typename devs::pca0, typename devs::pca1>;

    using led1 = devs::ledBlinker1;
    using led2 = devs::ledBlinker2;

    enum class State : uint8_t {Undefined, Init, I2CScan, Run};

    static inline void init() {
        devs::init();
        switches::init();
        radio::init();
    }
    static inline void periodic() {
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
        i2c::periodic();
        switches::periodic();
        radio::periodic();
    }

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> switchesTicks{20ms};

    static inline void ratePeriodic() {
        led1::ratePeriodic();
        led2::ratePeriodic();

        i2c::ratePeriodic();

        radio::ratePeriodic();

        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                mState = State::I2CScan;
            });
            break;
        case State::I2CScan:
            if (i2c::isIdle()) {
                mState = State::Run;
            }
            break;
        case State::Run:
            switches::ratePeriodic();
            (++mSwitchesTick).on(switchesTicks, []{
                switches::startRead([](const uint8_t index, const uint8_t newState){
                    IO::outl<debug>("# Switch ", index, " ", newState);
                    if (index < 32) {
                        const uint8_t wIndex1 = 2 * index;
                        const uint8_t wIndex2 = 2 * index + 1;
                        const uint64_t mask1 = (1 << wIndex1);
                        const uint64_t mask2 = (1 << wIndex2);
                        const bool on1 = (newState == 0);
                        const bool on2 = (newState == 2);

                        mSwitchesState = (mSwitchesState & ~mask1) | (on1 ? mask1 : 0);
                        mSwitchesState = (mSwitchesState & ~mask2) | (on2 ? mask2 : 0);

                        radio::set(mSwitchesState);
                    }
                    // IO::outl<debug>("# Switches ", mSwitchesState);
                });
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
            case State::I2CScan:
                IO::outl<debug>("# I2C Scan");
                if (i2c::scan([](const Mcu::Stm::I2C::Address a){
                              IO::outl<debug>("I2C: ", a.value);
                            })) {
                    IO::outl<debug>("# i2c scan start");
                }
                else {
                    IO::outl<debug>("# i2c scan failed");
                }
                break;
            case State::Run:
                IO::outl<debug>("# Run");
                break;
            }
        }

    }
    private:
    static inline uint64_t mSwitchesState = 0;

    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mSwitchesTick;
    static inline State mState{State::Undefined};

};

