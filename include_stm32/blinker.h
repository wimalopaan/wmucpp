#pragma once

#include <cstdint>
#include <type_traits>
#include <output.h>
#include <mcu/alternate.h>

#include "tick.h"

namespace External {
    using namespace std::literals::chrono_literals;

    template<typename Pin, typename Timer, typename Debug = void>
    struct Blinker {
        enum class State : uint8_t {Off, On, IntervallOff, IntervallOn, BlinkOff};

        enum class Event : uint8_t {None, Off, Steady, Slow, Medium, Fast};

        static inline void event(const Event e) {
            IO::outl<Debug>("B E: ", (uint8_t)e);
            mEvent = e;
            switch(e) {
            case Event::Slow:
                onTicks = 100ms;
                offTicks = 5000ms;
                break;
            case Event::Medium:
                onTicks = 500ms;
                offTicks = 1200ms;
                break;
            case Event::Fast:
                onTicks = 100ms;
                offTicks = 210ms;
                break;
            default:
                break;
            }
        }

        static inline void ratePeriodic() {
            const Event e = std::exchange(mEvent, Event::None);
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Off:
                processEvent(e);
                break;
            case State::On:
                processEvent(e);
                break;
            case State::IntervallOff:
                mStateTick.on(offTicks, []{
                    mState = State::IntervallOn;
                });
                processEvent2(e);
                break;
            case State::IntervallOn:
                mStateTick.on(onTicks, []{
                    ++mBlCounter;
                    if (mBlCounter == mBlinkCount) {
                        mState = State::IntervallOff;
                        mBlCounter = 0;
                    }
                    else {
                        mState = State::BlinkOff;
                    }
                });
                processEvent2(e);
                break;
            case State::BlinkOff:
                mStateTick.on(onTicks, []{
                   mState = State::IntervallOn;
                });
                break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Off:
                    IO::outl<Debug>("B Off");
                    Pin::reset();
                    mBlCounter = 0;
                    break;
                case State::On:
                    IO::outl<Debug>("B On");
                    Pin::set();
                    break;
                case State::IntervallOff:
                    IO::outl<Debug>("B I Off");
                    Pin::reset();
                    break;
                case State::IntervallOn:
                    IO::outl<Debug>("B I on");
                    Pin::set();
                    break;
                case State::BlinkOff:
                    IO::outl<Debug>("Bl Off");
                    Pin::reset();
                    break;
                }
            }
        }
        static inline void count(const uint8_t c) {
            mBlinkCount = c;
        }
        private:
        static inline void processEvent(const Event e) {
            switch(e) {
            case Event::Off:
                mState = State::Off;
                break;
            case Event::Steady:
                mState = State::On;
                break;
            case Event::Slow:
                mState = State::IntervallOn;
                mBlCounter = 0;
                break;
            case Event::Medium:
                mState = State::IntervallOn;
                mBlCounter = 0;
                break;
            case Event::Fast:
                mState = State::IntervallOn;
                mBlCounter = 0;
                break;
            case Event::None:
                break;
            }
        }
        static inline void processEvent2(const Event e) {
            switch(e) {
            case Event::Off:
                mState = State::Off;
                break;
            case Event::Steady:
                mState = State::On;
                break;
            case Event::Slow:
            case Event::Medium:
            case Event::Fast:
            case Event::None:
                break;
            }
        }
        static inline Event mEvent{Event::None};
        static inline State mState{State::Off};
        static inline External::Tick<Timer> offTicks{500ms};
        static inline External::Tick<Timer> onTicks{500ms};
        static inline External::Tick<Timer> mStateTick;
        static inline uint8_t mBlinkCount = 1;
        static inline uint8_t mBlCounter = 0;
    };

    template<typename Pin, typename Timer, typename Pwm = void, typename Debug = void>
    struct BlinkerWithPwm {
        enum class State : uint8_t {Off, On, IntervallOff, IntervallOn};

        enum class Event : uint8_t {None, Off, On};

        static inline void on(const uint8_t on) {
            if (on != 0) {
                event(Event::On);
            }
            else {
                event(Event::Off);
            }
        }
        static inline void event(const Event e) {
            mEvent = e;
        }

        static inline void blink(const uint8_t b) {
            IO::outl<Debug>("Pin: ", Pin::number, " Blink: ", b);
            mBlink = b;
        }

        static inline void on_dezi(const uint8_t b) {
            IO::outl<Debug>("Pin: ", Pin::number, " Ion: ", b);
            onTicks = External::Tick<Timer>::fromRaw(b * 200);
        }

        static inline void off_dezi(const uint8_t b) {
            IO::outl<Debug>("Pin: ", Pin::number, " Ioff: ", b);
            offTicks = External::Tick<Timer>::fromRaw(b * 200);
        }

        static inline void pwm(const uint8_t p) {
            IO::outl<Debug>("Pin: ", Pin::number, " pwm: ", p);
            mUsePwm = p;
            if ((mState == State::On) || (mState == State::IntervallOn)) {
                on();
            }
        }
        static inline void expo(const uint8_t) {
        }

        static inline void duty(const uint8_t d) {
            if constexpr(!std::is_same_v<Pwm, void>) {
                IO::outl<Debug>("Pin: ", Pin::number, " duty: ", d);
                if constexpr(!std::is_same_v<Pwm, void>) {
                    using pol_t = Pwm::polarity_t;
                    if constexpr(std::is_same_v<pol_t, Mcu::Stm::AlternateFunctions::Negativ>) {
                        Pwm::duty(100 - d);
                    }
                    else {
                        Pwm::duty(d);
                    }
                }
            }
        }

        static inline void set() {
            if (mUsePwm && ((mState == State::On) || (mState == State::IntervallOn))) {
                Pin::set();
            }
        }
        static inline void reset() {
            if (mUsePwm && ((mState == State::On) || (mState == State::IntervallOn))) {
                Pin::reset();
            }
        }

        static inline void ratePeriodic() {
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Off:
                if (const Event e = std::exchange(mEvent, Event::None); e == Event::On) {
                    if (mBlink) {
                        mState = State::IntervallOn;
                    }
                    else {
                        mState = State::On;
                    }
                }
                break;
            case State::On:
                if (const Event e = std::exchange(mEvent, Event::None); e == Event::Off) {
                    mState = State::Off;
                }
                if (mBlink) {
                    mState = State::IntervallOn;
                }
                break;
            case State::IntervallOff:
                if (const Event e = std::exchange(mEvent, Event::None); e == Event::Off) {
                    mState = State::Off;
                }
                mStateTick.on(offTicks, []{
                    if (mBlink) {
                        mState = State::IntervallOn;
                    }
                    else {
                        mState = State::On;
                    }
                });
                break;
            case State::IntervallOn:
                if (const Event e = std::exchange(mEvent, Event::None); e == Event::Off) {
                    mState = State::Off;
                }
                mStateTick.on(onTicks, []{
                    if (mBlink) {
                        mState = State::IntervallOff;
                    }
                    else {
                        mState = State::On;
                    }
                });
                break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Off:
                    IO::outl<Debug>("Pin: ", Pin::number, " Off");
                    Pin::reset();
                    Pin::template dir<Mcu::Output>();
                    break;
                case State::On:
                    IO::outl<Debug>("Pin: ", Pin::number, " On");
                    on();
                    break;
                case State::IntervallOff:
                    IO::outl<Debug>("Pin: ", Pin::number, " IntOff");
                    Pin::reset();
                    Pin::template dir<Mcu::Output>();
                    break;
                case State::IntervallOn:
                    IO::outl<Debug>("Pin: ", Pin::number, " IntOn");
                    on();
                    break;
                }
            }
        }
        private:
        static inline void on() {
            if (mUsePwm) {
                if constexpr(!std::is_same_v<Pwm, void>) {
                    if constexpr(Pwm::hasOutput) {
                        using pol_t = Pwm::polarity_t;
                        static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<Pin, Pwm, Mcu::Stm::AlternateFunctions::CC<Pwm::channel, pol_t>>;
                        Pin::afunction(af);
                        IO::outl<Debug>("Pin: ", Pin::number, " af: ", af);
                    }
                    else {
                        Pin::set();
                        Pin::template dir<Mcu::Output>();
                    }
                }
                else {
                    Pin::set();
                    Pin::template dir<Mcu::Output>();
                }
            }
            else {
                Pin::set();
                Pin::template dir<Mcu::Output>();
            }
        }
        static inline Event mEvent{Event::None};
        static inline State mState{State::Off};
        static inline bool mUsePwm{false};
        static inline bool mBlink{false};
        static inline External::Tick<Timer> onTicks{100ms};
        static inline External::Tick<Timer> offTicks{100ms};
        static inline External::Tick<Timer> mStateTick;
    };

}
