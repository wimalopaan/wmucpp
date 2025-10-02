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

#include <cstdint>
#include <type_traits>
#include <output.h>
#include <mcu/alternate.h>

#include <etl/event.h>
#include "tick.h"

namespace External {
    using namespace std::literals::chrono_literals;

    namespace Morse {
        struct Entry {
            char letter;
            uint8_t length;
            uint8_t pattern; // LSB first, 0: dit; 1: dah
        };
        static inline constexpr std::array<Entry, 45> lut = {{
            { 'A', 2, 0b01},
            { 'B', 4, 0b1000 },
            { 'C', 4, 0b1010},
            { 'D', 3, 0b100},
            { 'E', 1, 0b0 },
            { 'F', 4, 0b0010 },
            { 'G', 3, 0b110 },
            { 'H', 4, 0b0000 },
            { 'I', 2, 0b00 },
            { 'J', 4, 0b0111 },
            { 'K', 3, 0b101 },
            { 'L', 4, 0b0100 },
            { 'M', 2, 0b11 },
            { 'N', 2, 0b10 },
            { 'O', 3, 0b111 },
            { 'P', 4, 0b0110 },
            { 'Q', 4, 0b1101 },
            { 'R', 3, 0b010 },
            { 'S', 3, 0b000 },
            { 'T', 1, 0b1 },
            { 'U', 3, 0b001 },
            { 'V', 4, 0b0001 },
            { 'W', 3, 0b011 },
            { 'X', 4, 0b1001 },
            { 'Y', 4, 0b1011 },
            { 'Z', 4, 0b1100 },
            { '1', 5, 0b01111 },
            { '2', 5, 0b00111 },
            { '3', 5, 0b00011 },
            { '4', 5, 0b00001 },
            { '5', 5, 0b00000 },
            { '6', 5, 0b10000 },
            { '7', 5, 0b11000 },
            { '8', 5, 0b11100 },
            { '9', 5, 0b11110 },
            { '0', 5, 0b11111 },
            { '.', 6, 0b010101 },
            { ',', 6, 0b110011 },
            { ':', 6, 0b111000 },
            { ';', 6, 0b101010 },
            { '?', 6, 0b001100 },
            { '!', 6, 0b101011 },
            { '-', 6, 0b100001 },
            { '=', 6, 0b10001 },
            { '+', 6, 0b01010 }
        }};

        template<typename Pin, typename Config, typename Pwm = void>
        struct BlinkerWithPwm {
            using Timer = Config::timer;
            using Debug = Config::debug;

            static inline constexpr auto& text = Config::text;

            enum class State : uint8_t {Off, On, IntervallOff, IntervallOn, IntervallGap,
                                        PwmMode,
                                        MorseStart, MorseNext, MorseDah, MorseDit, MorseGap, MorseIGap};
            enum class Event : uint8_t {None, Off, On, PwmModeOn, PwmModeOff};
            enum class Blink : uint8_t {Steady, Blink, Morse};

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
                IO::outl<Debug>("# Pin: ", Pin::number, " Blink: ", b);
                if (b == 0) {
                    mBlinkMode = Blink::Steady;
                }
                else if (b == 1) {
                    mBlinkMode = Blink::Blink;
                }
                else if (b == 2) {
                    mBlinkMode = Blink::Morse;
                }
            }
            static inline void morse_dit_dezi(const uint8_t b) {
                IO::outl<Debug>("# Pin: ", Pin::number, " dit: ", b);
                morseDitTicks = External::Tick<Timer>::fromRaw(b * 200);
            }
            static inline void morse_dah_dezi(const uint8_t b) {
                IO::outl<Debug>("# Pin: ", Pin::number, " dah: ", b);
                morseDahTicks = External::Tick<Timer>::fromRaw(b * 200);
            }
            static inline void morse_gap_dezi(const uint8_t b) {
                IO::outl<Debug>("# Pin: ", Pin::number, " gap: ", b);
                morseGapTicks = External::Tick<Timer>::fromRaw(b * 200);
            }
            static inline void morse_igap_dezi(const uint8_t b) {
                IO::outl<Debug>("# Pin: ", Pin::number, " igap: ", b);
                morseIGapTicks = External::Tick<Timer>::fromRaw(b * 200);
            }
            static inline void on_dezi(const uint8_t b) {
                IO::outl<Debug>("# Pin: ", Pin::number, " Ion: ", b);
                onTicks = External::Tick<Timer>::fromRaw(b * 100);
            }
            static inline void off_dezi(const uint8_t b) {
                IO::outl<Debug>("# Pin: ", Pin::number, " Ioff: ", b);
                offTicks = External::Tick<Timer>::fromRaw(b * 100);
            }
            static inline void flash_count(const uint8_t c) {
                if (c > 0) {
                    IO::outl<Debug>("# Pin: ", Pin::number, " count: ", c);
                    flashCount = c;
                }
                else {
                    flashCount = 1;
                }
            }
            static inline void pwm(const uint8_t p) {
                IO::outl<Debug>("# Pin: ", Pin::number, " pwm: ", p);
                mEvent = Event::None;
                mUsePwm = p;
                if (p == 0) {
                    event(Event::PwmModeOff);
                }
                else if (p == 1) {
                    if ((mState == State::On) || (mState == State::IntervallOn)) {
                        on();
                    }
                }
                else if (p == 2) {
                    event(Event::PwmModeOn);
                }
                ratePeriodic();
            }
            static inline void expo(const uint8_t) {}
            static inline void duty(const uint8_t d) {
                if constexpr(!std::is_same_v<Pwm, void>) {
                    IO::outl<Debug>("# Pin: ", Pin::number, " duty: ", d, " state: ", (uint8_t)mState);
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
                if (mUsePwm && ((mState == State::On) || (mState == State::IntervallOn) || (mState == State::PwmMode))) {
                    Pin::set();
                }
            }
            static inline void reset() {
                if (mUsePwm && ((mState == State::On) || (mState == State::IntervallOn) || (mState == State::PwmMode))) {
                    Pin::reset();
                }
            }
            static inline void ratePeriodic() {
                static struct {
                    uint8_t actualPattern;
                    uint8_t charIndex = 0;
                    uint8_t signsLeft = 0;
                } morseState;

                const auto oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::Off:
                    if (mEvent.is(Event::On)) {
                        if (mBlinkMode == Blink::Blink) {
                            remaingFlashCount = flashCount;
                            mState = State::IntervallOn;
                        }
                        else if (mBlinkMode == Blink::Morse) {
                            morseState.charIndex = 0;
                            mState = State::MorseStart;
                        }
                        else if (mBlinkMode == Blink::Steady) {
                            mState = State::On;
                        }
                    }
                    else if (mEvent.is(Event::PwmModeOn)) {
                        mState = State::PwmMode;
                    }
                    break;
                case State::MorseStart:
                    if (text[morseState.charIndex] == '\0') {
                        mState = State::Off;
                    }
                    else {
                        mState = State::MorseNext;
                    }
                    break;
                case State::MorseNext:
                    if (morseState.actualPattern & 0b1) {
                        mState = State::MorseDah;
                    }
                    else {
                        mState = State::MorseDit;
                    }
                    break;
                case State::MorseDah:
                    mStateTick.on(morseDahTicks, [] static {
                        mState = State::MorseGap;
                    });
                    break;
                case State::MorseDit:
                    mStateTick.on(morseDitTicks, [] static {
                        mState = State::MorseGap;
                    });
                    break;
                case State::MorseGap:
                    mStateTick.on(morseGapTicks, [] static {
                      if (--morseState.signsLeft == 0) {
                          ++morseState.charIndex;
                          if ((text[morseState.charIndex] == '\0') || (morseState.charIndex >= text.size())) {
                              mState = State::Off;
                          }
                          else {
                                mState = State::MorseIGap;
                        }
                      }
                      else {
                          morseState.actualPattern >>= 1;
                          mState = State::MorseNext;
                      }
                      });
                    break;
                case State::MorseIGap:
                    mStateTick.on(morseIGapTicks, [] static {
                                     mState = State::MorseStart;
                                  });
                    break;
                case State::On:
                    mEvent.on(Event::Off, [] static {mState = State::Off;});
                    if (mBlinkMode == Blink::Blink) {
                        remaingFlashCount = flashCount;
                        mState = State::IntervallOn;
                    }
                    break;
                case State::IntervallOff:
                    mEvent.on(Event::Off, [] static {mState = State::Off;});
                    mStateTick.on(offTicks, []{
                        if (mBlinkMode == Blink::Blink) {
                            remaingFlashCount = flashCount;
                            mState = State::IntervallOn;
                        }
                        else {
                            mState = State::On;
                        }
                    });
                    break;
                case State::IntervallOn:
                    mEvent.on(Event::Off, [] static {mState = State::Off;});
                    mStateTick.on(onTicks, [] static {
                        if (remaingFlashCount > 0) {
                            --remaingFlashCount;
                        }
                        if (mBlinkMode == Blink::Blink) {
                            if (remaingFlashCount == 0) {
                                mState = State::IntervallOff;
                            }
                            else {
                                mState = State::IntervallGap;
                            }
                        }
                        else {
                            mState = State::On;
                        }
                    });
                    break;
                case State::IntervallGap:
                    mEvent.on(Event::Off, [] static {mState = State::Off;});
                    mStateTick.on(onTicks, [] static {
                        if (mBlinkMode == Blink::Blink) {
                            mState = State::IntervallOn;
                        }
                        else {
                            mState = State::On;
                        }
                    });
                    break;
                case State::PwmMode:
                    mEvent.on(Event::PwmModeOff, [] static {
                                  mUsePwm = false;
                                  mState = State::Off;
                              });
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Off:
                        IO::outl<Debug>("# Pin: ", Pin::number, " Off");
                        Pin::reset();
                        Pin::template dir<Mcu::Output>();
                        break;
                    case State::MorseStart:
                    {
                        IO::outl<Debug>("# Pin: ", Pin::number, " MoStart");
                        const auto [pattern, length] = morseLookup(text[morseState.charIndex]);
                        morseState.actualPattern = pattern;
                        morseState.signsLeft = length;
                        Pin::reset();
                        Pin::template dir<Mcu::Output>();
                    }
                        break;
                    case State::MorseNext:
                        IO::outl<Debug>("# Pin: ", Pin::number, " MoNext");
                        break;
                    case State::MorseDah:
                        IO::outl<Debug>("# Pin: ", Pin::number, " MoDah");
                        on();
                        break;
                    case State::MorseDit:
                        IO::outl<Debug>("# Pin: ", Pin::number, " MoDit");
                        on();
                        break;
                    case State::MorseGap:
                        IO::outl<Debug>("# Pin: ", Pin::number, " MoGap");
                        Pin::reset();
                        Pin::template dir<Mcu::Output>();
                        break;
                    case State::MorseIGap:
                        IO::outl<Debug>("# Pin: ", Pin::number, " MoIGap");
                        Pin::reset();
                        Pin::template dir<Mcu::Output>();
                        break;
                    case State::On:
                        IO::outl<Debug>("# Pin: ", Pin::number, " On");
                        on();
                        break;
                    case State::IntervallOff:
                    case State::IntervallGap:
                        IO::outl<Debug>("# Pin: ", Pin::number, " IntOff");
                        Pin::reset();
                        Pin::template dir<Mcu::Output>();
                        break;
                    case State::IntervallOn:
                        IO::outl<Debug>("# Pin: ", Pin::number, " IntOn");
                        on();
                        break;
                    case State::PwmMode:
                        IO::outl<Debug>("# Pin: ", Pin::number, " PwmMode");
                        mUsePwm = true;
                        duty(0);
                        on();
                        break;
                    }
                }
            }
            private:

            static inline std::pair<uint8_t, uint8_t> morseLookup(const char c) {
                for(const Entry& e : lut) {
                    if (std::toupper(c) == e.letter) {
                        return {e.pattern, e.length};
                    }
                }
                return {0, 0};
            }

            static inline void on() {
                if (mUsePwm) {
                    if constexpr(!std::is_same_v<Pwm, void>) {
                        if constexpr(Pwm::hasOutput) {
                            using pol_t = Pwm::polarity_t;
                            static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<Pin, Pwm, Mcu::Stm::AlternateFunctions::CC<Pwm::channel, pol_t>>;
                            Pin::afunction(af);
                            IO::outl<Debug>("# Pin: ", Pin::number, " af: ", af);
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
            static inline etl::Event<Event> mEvent;
            static inline State mState{State::Off};
            static inline bool mUsePwm{false};
            static inline Blink mBlinkMode{Blink::Steady};
            static inline External::Tick<Timer> morseDitTicks{200ms};
            static inline External::Tick<Timer> morseDahTicks{600ms};
            static inline External::Tick<Timer> morseGapTicks{200ms};
            static inline External::Tick<Timer> morseIGapTicks{200ms};
            static inline External::Tick<Timer> onTicks{100ms};
            static inline External::Tick<Timer> offTicks{100ms};
            static inline uint8_t flashCount = 1;
            static inline uint8_t remaingFlashCount = flashCount;
            static inline External::Tick<Timer> mStateTick;
        };
    }

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
        static inline void periodic() {}

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
        enum class State : uint8_t {Off, On, IntervallOff, IntervallOn, PwmMode};

        enum class Event : uint8_t {None, Off, On, PwmModeOn, PwmModeOff};

        static inline void on(const uint8_t on) {
            if (on != 0) {
                event(Event::On);
            }
            else {
                event(Event::Off);
            }
        }
        static inline void event(const Event e) {
            if (mEvent == Event::None) {
                mEvent = e;
            }
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
            mEvent = Event::None;
            mUsePwm = p;
            if (p == 0) {
                event(Event::PwmModeOff);
            }
            else if (p == 1) {
                if ((mState == State::On) || (mState == State::IntervallOn)) {
                    on();
                }
            }
            else if (p == 2) {
                event(Event::PwmModeOn);
            }
            ratePeriodic();
        }
        static inline void expo(const uint8_t) {
        }
        static inline void duty(const uint8_t d) {
            if constexpr(!std::is_same_v<Pwm, void>) {
                IO::outl<Debug>("Pin: ", Pin::number, " duty: ", d, " state: ", (uint8_t)mState);
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
            if (mUsePwm && ((mState == State::On) || (mState == State::IntervallOn) || (mState == State::PwmMode))) {
                Pin::set();
            }
        }
        static inline void reset() {
            if (mUsePwm && ((mState == State::On) || (mState == State::IntervallOn) || (mState == State::PwmMode))) {
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
                else if (e == Event::PwmModeOn) {
                    mState = State::PwmMode;
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
            case State::PwmMode:
                if (const Event e = std::exchange(mEvent, Event::None); e == Event::PwmModeOff) {
                    mUsePwm = false;
                    mState = State::Off;
                }
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
                case State::PwmMode:
                    IO::outl<Debug>("Pin: ", Pin::number, " PwmMode");
                    mUsePwm = true;
                    duty(0);
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
