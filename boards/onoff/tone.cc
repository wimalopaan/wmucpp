/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

// Clock = 8MHz / 8 = 1MHz, EEProm 
// sudo avrdude -p attiny85 -P usb -c avrisp2 -U lfuse:w:0x62:m -U hfuse:w:0xd7:m -U efuse:w:0xff:m

// sudo avrdude -p attiny85 -P usb -c avrisp2 -U flash:w:main.elf

#define NDEBUG

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/sleep.h"
#include "mcu/avr/groups.h"
#include "hal/alarmtimer.h"
#include "util/disable.h"
#include "util/types.h"

template<auto T>
struct static_print {
    std::integral_constant<uint16_t, T> v;
    using type = typename decltype(v)::_;
};

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer  = AlarmTimer<systemTimer, UseEvents<false>>;

using toneTimer = AVR::Timer8Bit<1>;

using mosfetPin = AVR::ActiveHigh<AVR::Pin<PortB, 4>, AVR::Output>;
using buttonPin = AVR::ActiveLow<AVR::Pin<PortB, 2>, AVR::Input>;
using buzzerPin1 = AVR::Pin<PortB, 1>;
using buzzerPin0 = AVR::Pin<PortB, 0>;
using testPin   = AVR::Pin<PortB, 3>;

struct Parameter {
    inline static constexpr auto intervall = 100_ms;
    inline static constexpr uint8_t ticksPerSecond = 1000_ms / intervall;
};

using sleep = AVR::Sleep<>;

template<typename Timer, typename P>
struct Beeper {
    enum class Mode : uint8_t {Off, Periodic, OneShot};
    enum class Tone : uint8_t {Tone, ScaleUp, ScaleDown};
    
    // 4100
    
    inline static constexpr auto fTone     = 4300_Hz;
    inline static constexpr auto fToneHigh = 4100_Hz;
    inline static constexpr auto fToneLow  = 3900_Hz;

    inline static constexpr auto formFactor = 2;
    
    inline static constexpr auto beepDuration  = 200_ms;
    inline static constexpr auto beepTicks  = beepDuration / P::intervall;
    
    using value_type = typename Util::TypeForValue_t<beepTicks>;
    
    inline static constexpr auto beepIntervall = 1000_ms;
    inline static constexpr auto beepIntervallTicks  = beepIntervall/ P::intervall;
    static_assert(beepIntervallTicks < std::numeric_limits<value_type>::max());

    inline static constexpr auto numberOfPhases = beepIntervallTicks / beepTicks;
    
//    static_print<beepIntervallTicks> x;
    
    inline static constexpr auto tl = AVR::Util::calculate<Timer>(fToneLow);
    static_assert(tl, "falscher wert für p");

    inline static void init() {
        Timer::template prescale<tl.prescaler>();
        Timer::mode(AVR::TimerMode::CTCNoInt);
        normal();
    }
    
    inline static void tick() {
        uint8_t phase = ++mTicks / beepTicks;
        if (mMode != Mode::Off) {
            if (mTone == Tone::Tone) {
                if (phase == 0) {
                    buzzerPin1::dir<AVR::Output>();
                    buzzerPin0::dir<AVR::Output>();
                }
                else {
                    buzzerPin1::dir<AVR::Input>();
                    buzzerPin0::dir<AVR::Input>();
                    if (mMode == Mode::OneShot) {
                        mMode = Mode::Off;
                    }
                }
            }
            else if (mTone == Tone::ScaleUp) {
                if (phase == 0) {
                    low();
                    buzzerPin1::dir<AVR::Output>();
                    buzzerPin0::dir<AVR::Output>();
                }
                else if (phase == 1) {
                    normal();
                }
                else if (phase == 2) {
                    high();
                }
                else {
                    buzzerPin1::dir<AVR::Input>();
                    buzzerPin0::dir<AVR::Input>();
                }
            }
            else if (mTone == Tone::ScaleDown) {
                if (phase == 0) {
                    high();
                    buzzerPin1::dir<AVR::Output>();
                    buzzerPin0::dir<AVR::Output>();
                }
                else if (phase == 1) {
                    normal();
                }
                else if (phase == 2) {
                    low();
                }
                else {
                    buzzerPin1::dir<AVR::Input>();
                    buzzerPin0::dir<AVR::Input>();
                }
            }
        }
        else {
            buzzerPin1::dir<AVR::Input>();
            buzzerPin0::dir<AVR::Input>();
        }
    }
    inline static void low() {
        Timer::template ocrc<tl.ocr>();
        Timer::template ocra<tl.ocr / formFactor>();
    }            
    inline static void high() {
        constexpr auto ocr = (tl.ocr * fToneLow) / fToneHigh;
        static_assert(ocr <= std::numeric_limits<value_type>::max());
        Timer::template ocrc<ocr>();
        Timer::template ocra<ocr / formFactor>();
    }            
    inline static void normal() {
        constexpr auto ocr = (tl.ocr * fToneLow) / fTone;
        static_assert(ocr <= std::numeric_limits<value_type>::max());
        Timer::template ocrc<ocr>();
        Timer::template ocra<ocr / formFactor>();
    }            
    inline static void on() {
        mTicks = 0;
        mMode = Mode::Periodic;
    }
    inline static void oneShot() {
        mTicks = 0;
        mMode = Mode::OneShot;
    }
    inline static void off() {
        mTicks = 0;
        mMode = Mode::Off;
    }
    inline static void mode(Mode m) {
        mTicks = 0;
        mMode = m;
    }
    inline static void tone(Tone t) {
        mTone = t;
    }
private:
    inline static Mode mMode = Mode::Off;
    inline static Tone mTone = Tone::Tone;
    inline static uint_ranged_circular<value_type, 0, beepIntervallTicks> mTicks;
};

using beeper = Beeper<toneTimer, Parameter>;

int main() {
    mosfetPin::init();
    buttonPin::init();
    
    testPin::dir<AVR::Output>();
    testPin::off();
    
    systemTimer::setup<Config::Timer::frequency>(AVR::TimerMode::CTCNoInt);
    auto secondsTimer = alarmTimer::create(Parameter::intervall, AlarmFlags::Periodic);
    
    beeper::init();
    
    beeper::normal();
    beeper::on();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        while(true) {
            systemTimer::periodic<systemTimer::flags_type::ocf0a>([&](){
                alarmTimer::periodic([&](uint7_t timer){
                    if (timer == *secondsTimer) {
                        testPin::toggle();
                        beeper::tick();
                    }                
                });
            });
        }
    }
}


#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {
        led::toggle();
    }
}
#endif
