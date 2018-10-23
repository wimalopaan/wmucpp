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

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer  = AlarmTimer<systemTimer, UseEvents<false>>;

using toneTimer = AVR::Timer8Bit<1>;

using mosfetPin = AVR::ActiveHigh<AVR::Pin<PortB, 4>, AVR::Output>;
using buttonPin = AVR::ActiveLow<AVR::Pin<PortB, 2>, AVR::Input>;
using buzzerPin = AVR::Pin<PortB, 1>;
using testPin   = AVR::Pin<PortB, 3>;

struct Parameter {
    inline static constexpr auto intervall = 100_ms;
    inline static constexpr uint8_t ticksPerSecond = 1000_ms / intervall;
};

using sleep = AVR::Sleep<>;

template<typename Timer, typename P>
struct Beeper {
    inline static constexpr auto fTone     = 2000_Hz;
    inline static constexpr auto fToneHigh = 2500_Hz;
    inline static constexpr auto fToneLow  = 1000_Hz;

    inline static constexpr auto formFactor = 8;
    
    inline static constexpr auto beepDuration  = 200_ms;
    inline static constexpr auto beepTicks  = beepDuration / P::intervall;
    
    using value_type = typename Util::TypeForValue_t<beepTicks>;
    
    inline static constexpr auto beepIntervall = 1000_ms;
    inline static constexpr auto beepIntervallTicks  = beepIntervall/ P::intervall;
    static_assert(beepIntervallTicks < std::numeric_limits<value_type>::max());
    std::integral_constant<value_type, beepIntervallTicks> x;
//    decltype(x)::_;
    
    inline static constexpr auto tl = AVR::Util::calculate<Timer>(fToneLow);
    static_assert(tl, "falscher wert für p");

//    std::integral_constant<value_type, tl.ocr> x;
//    std::integral_constant<value_type, tl.prescaler> y;
//    decltype(x)::_;
//    decltype(y)::_;
    
    inline static void init() {
        Timer::template prescale<tl.prescaler>();
        Timer::mode(AVR::TimerMode::CTCNoInt);
        normal();
    }
    
    inline static void tick() {
        if (isOn) {
            if (++ticks < beepTicks) {
                buzzerPin::dir<AVR::Output>();
            }
            else {
                buzzerPin::dir<AVR::Input>();
            }
        }
        else {
            buzzerPin::dir<AVR::Input>();
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
        if (!isOn) {
            ticks = 0;
        }
        isOn = true;
    }
    inline static void off() {
        isOn = false;
    }
private:
//    value_type::_;
    inline static bool isOn = false;
//    inline static uint8_t ticks = 0;
    inline static uint_ranged_circular<value_type, 0, beepIntervallTicks> ticks;
};

using beeper = Beeper<toneTimer, Parameter>;

template<typename P, typename Beeper>
struct FSM {
    inline static constexpr auto startPressDuration = 1000_ms;
    inline static constexpr auto  startPressTicks = startPressDuration / P::intervall;
    
    inline static constexpr auto sleepStartDuration = 3000_ms;
    inline static constexpr auto sleepStartTicks = sleepStartDuration / P::intervall;

    using value_type = typename Util::TypeForValue_t<sleepStartTicks>;
    
    enum class State : uint8_t {Off, WaitForOn, PreOn, On, WaitForOff, PreOff};
    
    inline static void tick() {
        State newState = state;
        switch(state) {
        case State::Off:
            if (buttonPin::isActive()) {
                ++pressTicks;
                if (pressTicks > startPressTicks) {
                    newState = State::WaitForOn;
                }
            }
            else {
                pressTicks = 0;
                ++sleepTicks;
                if (sleepTicks > sleepStartTicks) {
                    sleep::down();
                    sleepTicks = 0;
                }
            }
            break;
        case State::WaitForOn:
            if (buttonPin::isActive()) {
                ++pressTicks;
                if (pressTicks > (3 * startPressTicks)) {
                    newState = State::PreOn;
                }
            }
            else {
                newState = State::Off;
            }
            break;
        case State::PreOn:
            if (buttonPin::isActive()) {
                ++pressTicks;
                if (pressTicks > startPressTicks) {
                    newState = State::On;
                }
            }
            else {
                newState = State::Off;
            }
            break;
        case State::On:
            if (buttonPin::isActive()) {
                ++pressTicks;
                if (pressTicks > startPressTicks) {
                    newState = State::WaitForOff;
                }
            }
            else {
                pressTicks = 0;
            }
            break;
        case State::WaitForOff:
            if (buttonPin::isActive()) {
                ++pressTicks;
                if (pressTicks > (3 * startPressTicks)) {
                    newState = State::PreOff;
                }
            }
            else {
                newState = State::On;
            }
            break;
        case State::PreOff:
            if (buttonPin::isActive()) {
                ++pressTicks;
                if (pressTicks > startPressTicks) {
                    newState = State::Off;
                }
            }
            else {
                newState = State::On;
            }
            break;
        default:
            break;
        }
        if (newState != state) {
            pressTicks = 0;
            switch(state = newState) {
            case State::Off:
                Beeper::off();
                mosfetPin::inactivate();
                break;
            case State::WaitForOn:
            case State::WaitForOff:
                Beeper::normal();
                Beeper::on();
                break;
            case State::PreOn:
                Beeper::high();
                Beeper::on();
                break;
            case State::On:
                Beeper::off();
                mosfetPin::activate();
                break;
            case State::PreOff:
                Beeper::low();
                Beeper::on();
                break;
            default:
                break;
            }
        }
    }
    inline static State state = State::Off;
    inline static value_type pressTicks = 0;
    inline static value_type sleepTicks = 0;
};

using fsm = FSM<Parameter, beeper>;

struct Button : public IsrBaseHandler<AVR::ISR::Int<0>> {
    inline static void isr() {}
};

using isrRegistrar = IsrRegistrar<Button>;

int main() {
    isrRegistrar::init();
    
    mosfetPin::init();
    buttonPin::init();
    
    testPin::dir<AVR::Output>();
    testPin::off();
    
    systemTimer::setup<Config::Timer::frequency>(AVR::TimerMode::CTCNoInt);
    auto secondsTimer = alarmTimer::create(Parameter::intervall, AlarmFlags::Periodic);
    
    beeper::init();

    // todo: aufsteigende Tonfolge
    
    constexpr auto interrupts = AVR::getBaseAddr<AVR::ATTiny85::Interrupt>;
    interrupts()->gifr.reset<AVR::ATTiny85::Interrupt::GIFlags::intf>();
    interrupts()->gimsk.set<AVR::ATTiny85::Interrupt::GIMask::ie>();
    
    sleep::init<sleep::PowerDown>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        sleep::down();        
    
        beeper::on();
        
        while(true) {
            systemTimer::periodic<systemTimer::flags_type::ocf0a>([&](){
                alarmTimer::periodic([&](uint7_t timer){
                    if (timer == *secondsTimer) {
                        fsm::tick();
                        beeper::tick();
                    }                
                });
            });
        }
        
    }
}

ISR(INT0_vect) {
    isrRegistrar::isr<AVR::ISR::Int<0>>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {
        led::toggle();
    }
}
#endif
