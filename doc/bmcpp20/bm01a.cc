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

#include <stdint.h>
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "hal/constantrate.h"
#include "std/array.h"
#include "std/concepts.h"
#include "util/disable.h"
#include "util/fixedpoint.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

#include "../3rdparty/itoa/itoa.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

//using terminalDevice = AVR::Usart<1>;
//using terminal = std::basic_ostream<terminalDevice>;

namespace UtilN {
namespace detail {

} // detail

} // UtilN

using constantRateTimer = AVR::Timer16Bit<1>;
constexpr const auto constantRatePeriod = 1000_us;

struct CTHandler : public IsrBaseHandler<AVR::ISR::Timer<1>::CompareA> {
    static void isr() {
        ++mCounter;
    }
    inline static volatile uint16_t mCounter = 0;
};

using isrRegistrar = IsrRegistrar<CTHandler>;

struct Measure {
    uint64_t value = 0;
    std::milliseconds time;
    uint8_t type = 0;
};

std::array<char, Util::numberOfDigits<uint64_t>() + 1> string;

int main() {
    isrRegistrar::init();
    terminalDevice::init<19200>();
    Scoped<EnableInterrupt> interruptEnabler;
    
    constexpr std::hertz constantRateFrequency = 1 / constantRatePeriod;
    constexpr auto tsd = AVR::Util::calculate<constantRateTimer>(constantRateFrequency);
    static_assert(tsd, "wrong parameter");
    constantRateTimer::prescale<tsd.prescaler>();
    constantRateTimer::ocra<tsd.ocr>();
    constantRateTimer::mode(AVR::TimerMode::CTC);
    
    constexpr uint8_t Base = 10;

//    for(auto d: UtilN::detail::Convert<uint8_t, 2, Base>::lookupTable) {
//        for(auto c : d) {
//            std::out<terminal>(c);
//        }
//        std::outl<terminal>();
//    }
    
    {
        uint64_t value = 65536;
        Util::V2::itoa<Base>(value, string);
        std::outl<terminal>(string);
    }
    {
        int64_t value = -65536;
        Util::V2::itoa<Base>(value, string);
        std::outl<terminal>(string);
    }
    

    
    const uint16_t iterations = 900;
    
    std::array<Measure, 20> times;
    
    uint64_t value = 1;
    std::array<char, Util::numberOfDigits<uint64_t, Base>() + 1> data;
    
    for(uint8_t p = 1; p < times.size; ++p) {
        if (value <= std::numeric_limits<uint8_t>::max()) {
            uint8_t lv = value;
            uint32_t start = CTHandler::mCounter;
            for(uint16_t n = 0; n < iterations; ++n) {
                _3rdParty::itoa_(lv, &data[0]);
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
                _3rdParty::itoa_(lv, &data[0]);
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
                _3rdParty::itoa_(lv, &data[0]);
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
                _3rdParty::itoa_(lv, &data[0]);
            }
            uint32_t end = CTHandler::mCounter;            
            times[p].value = value;
            times[p].time = std::milliseconds{static_cast<uint16_t>(end - start)};
            times[p].type = 4;
        }
        value *= 10;
    }
    
    for(uint8_t i = 0; i < times.size; ++i) {
        std::outl<terminal>(i, " : V : "_pgm, ',', times[i].type,  ',', times[i].value, ',', times[i].time);
    }        

    value = 1;
    
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
        std::outl<terminal>(i, " : V : "_pgm, times[i].type, ',', times[i].value, ',', times[i].time);
    }        

    value = 1;
    
    for(uint8_t p = 1; p < times.size; ++p) {
        if (value <= std::numeric_limits<uint8_t>::max()) {
            uint8_t lv = value;
            uint32_t start = CTHandler::mCounter;
            for(uint16_t n = 0; n < iterations; ++n) {
                itoa(lv, &data[0], 10);
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
                itoa(lv, &data[0], 10);
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
                itoa(lv, &data[0], 10);
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
                itoa(lv, &data[0], 10);
            }
            uint32_t end = CTHandler::mCounter;            
            times[p].value = value;
            times[p].time = std::milliseconds{static_cast<uint16_t>(end - start)};
            times[p].type = 4;
        }
        value *= 10;
    }
    
    for(uint8_t i = 0; i < times.size; ++i) {
        std::outl<terminal>(i, " : V : "_pgm, ',', times[i].type,  ',', times[i].value, ',', times[i].time);
    }        

    value = 1;    
    for(uint8_t p = 1; p < times.size; ++p) {
        auto lv = value;    
        itoa(lv, &data[0], 10);
        
        std::outl<terminal>(p, ',', lv, ',', data);
        
        value *= 10;
    }
    
    while(true) {}
}

#ifndef NDEBUG

void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    std::outl<terminal>("assert"_pgm);
    while(true) {}
}
#endif

ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
