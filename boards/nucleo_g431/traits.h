#pragma once

#include <limits>

namespace Etl {
    template<typename T>
    requires std::is_unsigned_v<T>
    inline constexpr bool isPowerof2(T v) {
        return v && ((v & (v - 1)) == 0);
    }

    template<typename T, uint8_t Base = 10>
    requires std::is_integral_v<T>
    inline consteval uint8_t numberOfDigits() {
        T v = std::numeric_limits<T>::max();
        uint8_t number = 0;
        while(v > 0) {
            v /= Base;
            ++number;
        }
        if (number == 0) {
            number = 1;
        }
        if constexpr(std::is_signed<T>::value) {
            number += 1;
        }
        return number;
    }
}

