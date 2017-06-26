/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#define NDEBUG

#include "ledbuttons.h"
#include "../include/anano.h"

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "hal/constantrate.h"
#include "units/duration.h"
#include "console.h"

using namespace AVR;

template<typename Button, typename Led>
class ButtonLed {
public:
    typedef Button button;    
    typedef Led led;    
    static void init() {
        Led::template dir<Output>();
        Button::template dir<Input>();
        Button::pullup();
    }
    static void update() {
        if (Button::isHigh()) {
            if (mStartTime == 0_ms) {
                Led::off();
            }
            else {
                --mStartTime;
            }
        }
        else {
            mStartTime = mDelay;
            Led::on();
        }
    }
    template<typename Terminal>
    static void print() {
        std::outl<Terminal>("TasterPin: "_pgm, Button::isHigh(), " LedPin: "_pgm, Led::get(), " Startzeit: "_pgm, mStartTime);
    }

private:
    inline static constexpr auto mDelay = 1000_ms;
    inline static auto mStartTime = 0_ms;
};

// todo: parameter resolution vom timer
template<typename... BTs>
class ButtonLedsController {
public:
    static void init() {
        (BTs::init(), ...);
    }
    static void periodic() {
        (BTs::update(), ...);
    }
    template<typename Terminal>
    static void print() {
        (BTs::template print<Terminal>(), ...);
    }  
};

template<typename ...> struct makeController;
template<typename LedPort, typename ButtonPort, size_t... Is>
struct makeController<LedPort, ButtonPort, std::index_sequence<Is...>> {
    typedef ButtonLedsController<ButtonLed<Pin<LedPort, Is>, Pin<ButtonPort, Is> >... > type;
};

using controller = typename makeController<PortB, PortC, std::make_index_sequence<2>>::type;

using systemTimer = AVR::Timer8Bit<0>;

using terminalDevice = Usart<0>;
using terminal = std::basic_ostream<terminalDevice>;

using systemConstantRate = PeriodicGroup<1, AVR::ISR::Timer<0>::CompareA, controller>;
using isrRegistrar = IsrRegistrar<systemConstantRate, terminalDevice::RxHandler, terminalDevice::TxHandler>;

static constexpr auto f = 1000_Hz;

int main() {
    controller::init();
    terminalDevice::init<9600>();

    systemTimer::setup<f>(TimerMode::CTC);

//    std::outl<terminal>("ANano Led Buttons"_pgm);
    
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            systemConstantRate::periodic();
            if (auto c = terminalDevice::get()) {
                if (*c == std::byte{'p'}) {
                    controller::print<terminal>();    
                }
            }
        }
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}

ISR(USART_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

