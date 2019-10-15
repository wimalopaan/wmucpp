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

// sudo avrdude -p attiny85 -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

#define NDEBUG

#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "hal/constantrate.h"
#include "std/array"
#include "std/concepts"
#include "util/disable.h"
#include "util/bits.h"
#include "units/percent.h"
#include "container/stringbuffer.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
//using terminalDevice = SWUsart<0>;
using terminal = std::basic_ostream<terminalDevice>;

using systemTimer = AVR::Timer8Bit<1>;

//using isrRegistrar = IsrRegistrar<terminalDevice::TransmitBitHandler>;

template<typename MCUTimer>
constexpr AVR::Util::TimerSetupData<typename MCUTimer::value_type> calculate2(const std::hertz& ftimer) {
//    static_assert(MCUTimer::hasOcrA || MCUTimer::hasOcrB, "need ocra or ocrb");

    using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
    auto p = AVR::Util::prescalerValues(pBits::values);
    
    for(const auto& p : ::Util::sort(p)) { // aufsteigend
        std::outl<terminal>(p);
        if (p > 0) {
            const auto tv = (Config::fMcu / ftimer) / p;
            if ((tv > 0) && (tv < std::numeric_limits<typename MCUTimer::value_type>::max())) {
                const bool exact = ((Config::fMcu.value / p) % tv) == 0;
                return {p, static_cast<typename MCUTimer::value_type>(tv), Config::fMcu / tv / uint32_t(p), exact};
            }
        }
    }
    return {};
}


int main() {
    terminalDevice::init<19200>();
    constexpr auto tsd = AVR::Util::calculate<AVR::Timer8Bit<0>>(std::hertz{19200});
    
//    isrRegistrar::init();
    
//    led::init();
//    led::set(Constant::cGreen);
    
    std::outl<terminal>(tsd.ocr);
    std::outl<terminal>(tsd.prescaler);
    std::outl<terminal>(tsd.f);
    
//    {
//        Scoped<EnableInterrupt> ei;
//        std::outl<terminal>(Constant::title);
//        while (true) {
            
//        }
//    }
}
//ISR(TIMER0_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
//}

