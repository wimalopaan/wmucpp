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

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

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

std::array<char, Util::numberOfDigits<uint64_t>() + 1> string;
constexpr uint8_t Base = 10;

int main() {
    Scoped<EnableInterrupt> interruptEnabler;
    uint64_t value = 1234;
    UtilN::itoa<Base>(value, string);
    std::outl<terminal>(string);

    while(true) {}
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif

