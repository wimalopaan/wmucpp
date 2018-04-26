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

#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <type_traits>
#include "std/bitmask.h"

namespace Util {
    
    template<typename T>
    struct UnsignedFor;
    
    template<>
    struct UnsignedFor<int8_t> {
        typedef uint8_t type;
    };
    template<>
    struct UnsignedFor<int16_t> {
        typedef uint16_t type;
    };
    template<>
    struct UnsignedFor<int32_t> {
        typedef uint32_t type;
    };
    template<>
    struct UnsignedFor<int64_t> {
        typedef uint64_t type;
    };
    template<>
    struct UnsignedFor<uint8_t> {
        typedef uint8_t type;
    };
    template<>
    struct UnsignedFor<uint16_t> {
        typedef uint16_t type;
    };
    template<>
    struct UnsignedFor<uint32_t> {
        typedef uint32_t type;
    };
    template<>
    struct UnsignedFor<uint64_t> {
        typedef uint64_t type;
    };
    
    
    template<typename T>
    constexpr uint8_t numberOfBits() {
        return sizeof(T) * 8;
    }
    template<typename Bit, typename T>
    constexpr bool isSet(T v) {
        if constexpr(std::is_same<T, std::byte>::value) {
            return std::to_integer<uint8_t>(Bit::template Value<T>::value) & std::to_integer<uint8_t>(v);
        }
        else {
            return Bit::template Value<T>::value & v;
        }
    }
    template<typename Bit, typename T>
    constexpr bool set(T& v) {
        return v |= Bit::template Value<T>::value;
    }
    
    struct MSB {
        template<typename T>
        struct Value {
            static constexpr const T value{T(1) << (numberOfBits<T>() - 1)};
        };
    };  
    struct LSB {
        template<typename T>
        struct Value {
            static constexpr const T value = 1;
        };
    };  
    template<typename T>
    constexpr uint8_t numberOfOnes(T x) {
        return (x != T{0}) ? T(x & T{0x01}) + numberOfOnes(T(x>>1u)) : 0u;
    }
    
    template<typename T>
    struct enclosingType;
    template<>
    struct enclosingType<uint8_t> {
        typedef uint16_t type;
    };
    template<>
    struct enclosingType<uint16_t> {
        typedef uint32_t type;
    };
    template<>
    struct enclosingType<uint32_t> {
        typedef uint64_t type;
    };
    template<>
    struct enclosingType<int16_t> {
        typedef int32_t type;
    };
    template<>
    struct enclosingType<int8_t> {
        typedef int16_t type;
    };
    
    template<typename T>
    struct fragmentType;
    
    template<>
    struct fragmentType<uint16_t> {
        typedef uint8_t type;
        static constexpr const uint8_t shift = 8;
    };
    
    template<>
    struct fragmentType<uint32_t> {
        typedef uint16_t type;
        static constexpr const uint8_t shift = 16;
    };
    template<>
    struct fragmentType<uint64_t> {
        typedef uint32_t type;
        static constexpr const uint8_t shift = 32;
    };
    
    template<typename T>
    constexpr auto upperHalf(const T& v) -> typename fragmentType<T>::type {
        return v >> fragmentType<T>::shift;
    }
    
    template<typename T>
    constexpr auto lowerHalf(const T& v) -> typename fragmentType<T>::type {
        return v;
    }
    
    template<uint64_t V>
    struct TypeForValue {
        using type = typename std::conditional<(V > std::numeric_limits<uint32_t>::max()), uint64_t, 
                              typename std::conditional<(V > std::numeric_limits<uint16_t>::max()), uint32_t,
                                typename std::conditional<(V > std::numeric_limits<uint8_t>::max()), uint16_t, uint8_t>::type>::type>::type;
    };

    template<uint64_t Bits>
    struct TypeForBits {
        using type = typename std::conditional<(Bits <= 8), uint8_t, 
                         typename std::conditional<(Bits <= 16), uint16_t, 
                             typename std::conditional<(Bits <= 32), uint32_t,
                                  typename std::conditional<(Bits <= 64), uint64_t, void>::type>::type>::type>::type;
    };
    
    template<typename T>
    constexpr uint8_t minimumBitsForValue(const T& v) {
        for(uint8_t n = 1; n <= std::numeric_limits<uint8_t>::max(); ++n) {
            T max = T((1 << n) - 1);
            if (v <= max) {
                return n;
            }
        }
        return 0;
    }
    
} // Util


template<typename E>
constexpr bool isset(E flags) {
    typedef typename std::underlying_type<E>::type underlying;
    return static_cast<underlying>(flags) != 0;
}

