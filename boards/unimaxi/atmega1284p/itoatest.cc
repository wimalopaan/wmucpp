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

// 20MHz extern
// sudo avrdude -p atmega1284P -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd1:m -U efuse:w:0xfc:m

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/usart.h"
#include "hal/softspimaster.h"
#include "hal/bufferedstream.h"
#include "hal/alarmtimer.h"
#include "hal/constantrate.h"
#include "external/ws2812.h"
#include "util/util.h"
#include "units/duration.h"
#include "console.h"

auto date = PGMSTRING(__DATE__);

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using SoftSPIData = AVR::Pin<PortB, 1>;
using SoftSPIClock = AVR::Pin<PortB, 0>;
using SoftSPISS = AVR::Pin<PortC, 3>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;
//using terminal = SSpi0;
using terminalDevice = AVR::Usart<1>;
using terminal = std::basic_ostream<terminalDevice>;
//using bufferedTerminal = BufferedStream<SSpi0, 512>;

// Timer0

// Timer1

// Timer2

// Timer3

// Uart 0

using statusLed = WS2812<1, AVR::Pin<PortC, 6>>;
typedef statusLed::color_type StatusColor;

using constantRateTimer = AVR::Timer16Bit<3>;
constexpr const auto constantRatePeriod = 1000_us;

struct CTHandler : public IsrBaseHandler<AVR::ISR::Timer<3>::CompareA> {
    static void isr() {
        ++mCounter;
    }
    inline static volatile uint16_t mCounter = 0;
};

using isrRegistrar = IsrRegistrar<CTHandler, 
terminalDevice::RxHandler, terminalDevice::TxHandler>;

struct UsartFeHandler: public EventHandler<EventType::UsartFe> {
    static bool process(uint8_t n) {
        std::outl<terminal>("Usart Fe  "_pgm,  n);
        return true;
    }
};
struct UsartDorHandler: public EventHandler<EventType::UsartDor> {
    static bool process(uint8_t n) {
        std::outl<terminal>("Usart Dor "_pgm, n);
        return true;
    }
};
struct UsartUpeHandler: public EventHandler<EventType::UsartUpe> {
    static bool process(uint8_t n) {
        std::outl<terminal>("Usart Upe "_pgm, n);
        return true;
    }
};

struct Measure {
    uint64_t value = 0;
    std::milliseconds time;
    uint8_t type = 0;
};

constexpr uint8_t Base = 10;

int main() {
    isrRegistrar::init();
    terminalDevice::init<19200>();
    //    statusLed::off();
    
    constexpr std::hertz constantRateFrequency = 1 / constantRatePeriod;
    constexpr auto tsd = AVR::Util::calculate<constantRateTimer>(constantRateFrequency);
    static_assert(tsd, "wrong parameter");
    constantRateTimer::prescale<tsd.prescaler>();
    constantRateTimer::ocra<tsd.ocr>();
    constantRateTimer::mode(AVR::TimerMode::CTC);
    
    {
        Scoped<EnableInterrupt> interruptEnabler;
        
        for(auto d : Util::detail::Convert<2,10>::lookupTable) {
            for(auto c : d) {
                std::out<terminal>(c);
            }
        }
        
        const uint16_t iterations = 900;
                
        std::array<Measure, 20> times;
        
        uint64_t value = 1;
        std::array<char, Util::numberOfDigits<uint64_t>() + 1> data;
        
        for(uint8_t p = 1; p < times.size; ++p) {
            if (value <= std::numeric_limits<uint8_t>::max()) {
                uint8_t lv = value;
                uint32_t start = CTHandler::mCounter;
                for(uint16_t n = 0; n < iterations; ++n) {
                    Util::V2::itoa<Base>(lv, data);
                }
                uint32_t end = CTHandler::mCounter;            
                times[p].value = value;
                times[p].time = std::milliseconds{static_cast<uint16_t>(end - start)};
                times[p].type = 1;
            }
            else if (value <= std::numeric_limits<uint16_t>::max()) {
                uint16_t lv = value;
                uint32_t start = CTHandler::mCounter;
                for(uint16_t n = 0; n < iterations; ++n) {
                    Util::V2::itoa<Base>(lv, data);
                }
                uint32_t end = CTHandler::mCounter;            
                times[p].value = value;
                times[p].time = std::milliseconds{static_cast<uint16_t>(end - start)};
                times[p].type = 2;
            }
            else if (value <= std::numeric_limits<uint32_t>::max()) {
                uint32_t lv = value;
                uint32_t start = CTHandler::mCounter;
                for(uint16_t n = 0; n < iterations; ++n) {
                    Util::V2::itoa<Base>(lv, data);
                }
                uint32_t end = CTHandler::mCounter;            
                times[p].value = value;
                times[p].time = std::milliseconds{static_cast<uint16_t>(end - start)};
                times[p].type = 3;
            }
            else if (value <= std::numeric_limits<uint64_t>::max()) {
                uint64_t lv = value;
                uint32_t start = CTHandler::mCounter;
                for(uint16_t n = 0; n < iterations; ++n) {
                    Util::V2::itoa<Base>(lv, data);
                }
                uint32_t end = CTHandler::mCounter;            
                times[p].value = value;
                times[p].time = std::milliseconds{static_cast<uint16_t>(end - start)};
                times[p].type = 4;
            }
            value *= 10;
        }
        
        for(uint8_t i = 0; i < times.size; ++i) {
            std::outl<terminal>(i, " : V : "_pgm, times[i].value, ',', times[i].time);
        }        
        
        using allEventHandler = EventHandlerGroup<UsartDorHandler, UsartFeHandler, UsartUpeHandler>;
        
        EventManager::run3<allEventHandler>([](){
            if (EventManager::unprocessedEvent()) {
                StatusColor c{255};
                statusLed::set(c);
            }
            if (EventManager::leakedEvent()) {
                StatusColor c = {Red{255}, Green{0}, Blue{255}};
                statusLed::set(c);
            }
        });
    }
}

//ISR(TIMER0_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
//}

ISR(TIMER3_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<3>::CompareA>();
}
//ISR(USART0_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
//}
//ISR(USART0_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
//}

ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif
