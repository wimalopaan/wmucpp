#pragma once

#include <cstring>

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "clock.h"
#include "timer.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "gpio.h"
#include "tick.h"

namespace External {
    namespace Generator {
        template<typename CB, typename SystemTimer>
        struct Periodic {
            static inline constexpr External::Tick<SystemTimer> shortTicks{200ms};
            static inline constexpr External::Tick<SystemTimer> longTicks{500ms};
            static inline constexpr External::Tick<SystemTimer> longPeriodTicks{5000ms};
            static inline constexpr External::Tick<SystemTimer> shortPeriodTicks{500ms};

            enum class State : uint8_t {Idle, Short, Long,
                                        PeriodicShortOn, PeriodicShortOff,
                                        PeriodicLongOn, PeriodicLongOff,
                                        Steady};
            enum class Event : uint8_t {None, Stop, Steady, Short, Long, PeriodicShort, PeriodicLong};

            static inline void ratePeriodic() {
                const auto oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::Idle:
                    if (const auto e = std::exchange(mEvent, Event::None); e == Event::Short) {
                        mState = State::Short;
                    }
                    else if (e == Event::Long) {
                        mState = State::Long;
                    }
                    else if (e == Event::PeriodicShort) {
                        mState = State::PeriodicShortOn;
                    }
                    else if (e == Event::PeriodicLong) {
                        mState = State::PeriodicLongOn;
                    }
                    else if (e == Event::Steady) {
                        mState = State::Steady;
                    }
                    break;
                case State::Steady:
                    if (const auto e = std::exchange(mEvent, Event::None); e == Event::Stop) {
                        mState = State::Idle;
                    }
                    break;
                case State::Short:
                    mStateTick.on(shortTicks, []{
                        mState = State::Idle;
                    });
                    break;
                case State::Long:
                    mStateTick.on(longTicks, []{
                        mState = State::Idle;
                    });
                    break;
                case State::PeriodicShortOn:
                    if (const auto e = std::exchange(mEvent, Event::None); e == Event::Stop) {
                        mState = State::Idle;
                    }
                    mStateTick.on(shortTicks, []{
                        mState = State::PeriodicShortOff;
                    });
                    break;
                case State::PeriodicShortOff:
                    if (const auto e = std::exchange(mEvent, Event::None); e == Event::Stop) {
                        mState = State::Idle;
                    }
                    mStateTick.on(shortPeriodTicks, []{
                        mState = State::PeriodicShortOn;
                    });
                    break;
                case State::PeriodicLongOn:
                    if (const auto e = std::exchange(mEvent, Event::None); e == Event::Stop) {
                        mState = State::Idle;
                    }
                    mStateTick.on(shortTicks, []{
                        mState = State::PeriodicLongOff;
                    });
                    break;
                case State::PeriodicLongOff:
                    if (const auto e = std::exchange(mEvent, Event::None); e == Event::Stop) {
                        mState = State::Idle;
                    }
                    mStateTick.on(longPeriodTicks, []{
                        mState = State::PeriodicLongOn;
                    });
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Idle:
                        CB::on(false);
                        break;
                    case State::Steady:
                        CB::on(true);
                        break;
                    case State::Short:
                        CB::on(true);
                        break;
                    case State::Long:
                        CB::on(true);
                        break;
                    case State::PeriodicShortOn:
                        CB::on(true);
                        break;
                    case State::PeriodicShortOff:
                        CB::on(false);
                        break;
                    case State::PeriodicLongOn:
                        CB::on(true);
                        break;
                    case State::PeriodicLongOff:
                        CB::on(false);
                        break;
                    }
                }
            }
            static inline void event(const Event e) {
                mEvent = e;
            }
            private:
            static inline Event mEvent{Event::None};
            static inline State mState{State::Idle};
            static inline External::Tick<SystemTimer> mStateTick;
        };
    }
}

