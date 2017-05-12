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

#include "itoa.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

//using terminalDevice = AVR::Usart<1>;
//using terminal = std::basic_ostream<terminalDevice>;

namespace UtilN {
namespace detail {

template<uint8_t Digits = 2, uint8_t Base = 10> 
struct Convert {
    static_assert(Base <= 16, "wrong Base");
    static_assert(Digits <= 4, "wrong Digits");
    
    typedef uint16_t dimension_type;
    constexpr inline static dimension_type dimension = Base * Base;

    static constexpr char toChar(uint8_t d) {
        static_assert(Base <= 16, "wrong Base");
        if constexpr(Base > 10) {
            if (d < 10) {
                return '0' + d;
            }        
            else {
                return 'a' + d - 10;
            }
        }
        else {
            return '0' + d;
        }
    }
    
    constexpr static inline auto lookupTable = [](){
        std::array<std::array<char, Digits>, dimension> data;
        for(dimension_type i = 0; i < dimension; ++i) {
            auto value = i;
            for(int8_t d = Digits - 1; d >= 0; --d) {
                auto r = value % Base;
                data[i][d] = toChar(r);
                value /= Base;
            }
        }
        return data;
    }();
    
    template<typename T>
    constexpr static inline uint8_t maxPower = [](){
        uint64_t v = 1;
        for(uint8_t i = 0; i < 64; ++i) {
            if (v >= std::numeric_limits<T>::max() / Base) {
                return i;
            }
            v *= Base;
        }
    }();
    
    template<typename T>
    constexpr static inline auto powers = [](){
        std::array<uint64_t, maxPower<T> + 1> data;
        uint64_t v = 1;
        for(auto& p : data) {
            p = v;
            v *= Base;
        }
        return data;
    }();
    
    template<uint8_t B, uint8_t E, typename T>
    constexpr static uint8_t digits_r(T v) {
        constexpr uint8_t mid = (B + E) / 2;
        if constexpr(mid == B) {
            return mid + 1;
        }
        
        if (v < powers<T>[mid]) {
            return digits_r<B, mid>(v);
        }
        else {
            return digits_r<mid, E>(v);
        }
    }
    template<typename T>
    constexpr static uint8_t digits(T v) {
        return digits_r<0, powers<T>.size - 1>(v);
    }
};

template<uint8_t Base, std::Unsigned T>
void itoa(T value, uint8_t length, char* data) {
    auto next = length - 1;
    constexpr auto modul = detail::Convert<2, Base>::dimension;
    while(value >= modul) {
        auto const d = value % modul;
        data[next--] = detail::Convert<2, Base>::lookupTable[d][1];
        data[next--] = detail::Convert<2, Base>::lookupTable[d][0];
        value /= modul;
    }
    if (value < Base) {
        data[next] = detail::Convert<2, Base>::toChar(value);
    }
    else {
        auto const d = (uint8_t)value;
        data[next--] = detail::Convert<2, Base>::lookupTable[d][1];
        data[next] = detail::Convert<2, Base>::lookupTable[d][0];
    }
}

} // detail

template<uint8_t Base = 10, uint8_t offset = 0, std::Unsigned T = uint8_t, uint16_t L = 0>
void itoa(const T& value, std::array<char, L>& data) {
    static_assert(L >= Util::numberOfDigits<T, Base>(), "wrong length");
    auto length = detail::Convert<2, Base>::digits(value);
    if constexpr(std::is_same<T, uint64_t>::value) {
        using fragmentType = typename Util::fragmentType<T>::type;
        constexpr auto maximumPower = detail::Convert<2, Base>::template maxPower<fragmentType>;
        if (length > maximumPower) {
            uint32_t v1 = value / detail::Convert<2, Base>::template powers<fragmentType>[maximumPower];
            uint32_t v2 = value - v1;
            detail::itoa<Base>(v1 , length, &data[0] + offset);
            detail::itoa<Base>(v2 , length - maximumPower, &data[0] + offset);
        }
        else {
            detail::itoa<Base>(value, length, &data[0] + offset);
        }
    }
    else {
        detail::itoa<Base>(value, length, &data[0] + offset);
    }
}

template<uint8_t Base = 10, std::Signed T = uint8_t, uint16_t L = 0>
void itoa(const T& value, std::array<char, L>& data) {
    static_assert(L >= Util::numberOfDigits<T, Base>(), "wrong length");
    typedef typename UnsignedFor<T>::type uType;
    if (value < 0) {
        data[0] = '-';
        itoa<Base, 1>(static_cast<uType>(-value), data);
    }
    else {
        itoa<Base>(static_cast<uType>(value), data);
    }
}

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
        UtilN::itoa<Base>(value, string);
        std::outl<terminal>(string);
    }
    {
        int64_t value = -65536;
        UtilN::itoa<Base>(value, string);
        std::outl<terminal>(string);
    }
    

    
    const uint16_t iterations = 900;
    
    std::array<Measure, 20> times;
    
    uint64_t value = 1;
    std::array<char, Util::numberOfDigits<uint64_t, Base>() + 1> data;
    
#if 1
    
    for(uint8_t p = 1; p < times.size; ++p) {
        if (value <= std::numeric_limits<uint8_t>::max()) {
            uint8_t lv = value;
            uint32_t start = CTHandler::mCounter;
            for(uint16_t n = 0; n < iterations; ++n) {
                itoa_(lv, &data[0]);
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
                itoa_(lv, &data[0]);
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
                itoa_(lv, &data[0]);
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
                itoa_(lv, &data[0]);
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
#endif
    value = 1;
    
    for(uint8_t p = 1; p < times.size; ++p) {
        if (value <= std::numeric_limits<uint8_t>::max()) {
            uint8_t lv = value;
            uint32_t start = CTHandler::mCounter;
            for(uint16_t n = 0; n < iterations; ++n) {
                UtilN::itoa<Base>(lv, data);
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
                UtilN::itoa<Base>(lv, data);
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
                UtilN::itoa<Base>(lv, data);
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
                UtilN::itoa<Base>(lv, data);
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
