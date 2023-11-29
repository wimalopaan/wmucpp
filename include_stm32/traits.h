#pragma once

#include <limits>
#include <array>

namespace etl {
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
    
    
    template<typename T> 
    constexpr bool isInIntervallOf(auto l, auto h) {
        return (l >= std::numeric_limits<T>::min()) && (h <= std::numeric_limits<T>::max());
    }
    
    template<auto L, auto H>
    consteval auto typeForIntervall() {
        if constexpr ((L >= 0) && (H >= 0)) {
            if constexpr (isInIntervallOf<uint8_t>(L, H)) {
                return uint8_t{};
            }
            else if constexpr (isInIntervallOf<uint16_t>(L, H)) {
                return uint16_t{};
            }
            else if constexpr (isInIntervallOf<uint32_t>(L, H)) {
                return uint32_t{};
            }
            else if constexpr (isInIntervallOf<uint64_t>(L, H)) {
                return uint64_t{};
            }
        }
        else {
            if constexpr (isInIntervallOf<int8_t>(L, H)) {
                return int8_t{};
            }
            else if constexpr (isInIntervallOf<int16_t>(L, H)) {
                return int16_t{};
            }
            else if constexpr (isInIntervallOf<int32_t>(L, H)) {
                return int32_t{};
            }
            else if constexpr (isInIntervallOf<int64_t>(L, H)) {
                return int64_t{};
            }
        }
    }
    
    template<auto L, auto H>
    struct typeFromIntervall {
        using type = decltype(typeForIntervall<L, H>());
    };
    template<auto L, auto H>
    using typeForIntervall_t = typeFromIntervall<L, H>::type;
    
    static_assert(std::is_same_v<typeForIntervall_t<0, 128>, uint8_t>);
    static_assert(std::is_same_v<typeForIntervall_t<1, 128>, uint8_t>);
    static_assert(std::is_same_v<typeForIntervall_t<1, 255>, uint8_t>);
    static_assert(std::is_same_v<typeForIntervall_t<-128, 127>, int8_t>);
    static_assert(std::is_same_v<typeForIntervall_t<-128, 128>, int16_t>);
    
}

