#pragma once

/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <limits>
#include <type_traits>
#include <algorithm>

#include "concepts.h"
#include "meta.h"

namespace etl {
    template<typename T>
    consteval bool has_arithmetic_right_shift() {
        T v1{std::numeric_limits<T>::min()};
        return (v1 >>= 2) == (std::numeric_limits<T>::min() / 4);
    }
    template<typename T>
    constexpr bool has_arithmetic_right_shift_v = has_arithmetic_right_shift<T>();
    
    template<typename T>
    inline constexpr uint8_t numberOfBits() {
        return sizeof(T) * 8;
    }

    namespace detail {
        template<auto Bits>
        struct typeForBits {
            using type = typename std::conditional<(Bits <= 8), uint8_t, 
                             typename std::conditional<(Bits <= 16), uint16_t, 
                                 typename std::conditional<(Bits <= 32), uint32_t,
                                      typename std::conditional<(Bits <= 64), uint64_t, void>::type>::type>::type>::type;
        };
        template<auto Bits>
        struct signedTypeForBits {
            using type = typename std::conditional<(Bits <= 8), int8_t, 
                             typename std::conditional<(Bits <= 16), int16_t, 
                                 typename std::conditional<(Bits <= 32), int32_t,
                                      typename std::conditional<(Bits <= 64), int64_t, void>::type>::type>::type>::type;
        };
        template<auto V>
        struct typeForValue {
            using type = typename std::conditional<(V > std::numeric_limits<uint32_t>::max()), uint64_t, 
                                  typename std::conditional<(V > std::numeric_limits<uint16_t>::max()), uint32_t,
                                    typename std::conditional<(V > std::numeric_limits<uint8_t>::max()), uint16_t, uint8_t>::type>::type>::type;
        };
    
        template<typename T> struct enclosingType;

        template<> struct enclosingType<uint8_t> {
            typedef uint16_t type;
        };
        template<> struct enclosingType<uint16_t> {
            typedef uint32_t type;
        };
        template<> struct enclosingType<uint32_t> {
            typedef uint64_t type;
        };
        template<> struct enclosingType<int8_t> {
            typedef int16_t type;
        };
        template<> struct enclosingType<int16_t> {
            typedef int32_t type;
        };
        template<> struct enclosingType<int32_t> {
            typedef int64_t type;
        };
        
        template<typename T>struct fragmentType;
        
        template<> struct fragmentType<uint16_t> {
            typedef uint8_t type;
            static constexpr const uint8_t shift = 8;
        };
        
        template<> struct fragmentType<uint32_t> {
            typedef uint16_t type;
            static constexpr const uint8_t shift = 16;
        };
        template<> struct fragmentType<uint64_t> {
            typedef uint32_t type;
            static constexpr const uint8_t shift = 32;
        };

        template<typename T, typename U>
        struct containing_type;
        
        template<typename T, typename U>
        requires (std::is_signed_v<T> && std::is_signed_v<U>)
        struct containing_type<T, U> {
            constexpr static inline uint8_t bits = std::max(etl::numberOfBits<T>(), etl::numberOfBits<U>());
            using type = typename signedTypeForBits<bits>::type;        
        };
        template<typename T, typename U>
        requires (std::is_unsigned_v<T> && std::is_unsigned_v<U>)
        struct containing_type<T, U> {
            constexpr static inline uint8_t bits = std::max(etl::numberOfBits<T>(), etl::numberOfBits<U>());
            using type = typename typeForBits<bits>::type;        
        };
    }
    
    
    template<uint64_t Bits>
    using typeForBits_t = typename detail::typeForBits<Bits>::type;  

    template<uint64_t Bits>
    using signedTypeForBits_t = typename detail::signedTypeForBits<Bits>::type;  
    
    template<auto V>
    using typeForValue_t = typename detail::typeForValue<V>::type;
    
    template<typename T>
    using enclosingType_t = typename detail::enclosingType<T>::type;

    template<typename T>
    using enclosing_t = typename detail::enclosingType<std::remove_cv_t<T>>::type;
    
    template<typename T>
    using fragmentType_t = typename detail::fragmentType<T>::type;    

    template<typename T>
    using fragment_t = typename detail::fragmentType<T>::type;    

    template<typename T, typename U>
    using containing_type_t = typename detail::containing_type<T, U>::type;

    
    namespace detail {
        template<class Trait, typename = void>
        struct propogate_cv_to_type {};
        
        template<class Trait>
        struct propogate_cv_to_type<Trait, std::void_t<typename Trait::value_type>> { 
            using type = typename Trait::value_type; 
        }; 
        
        template<class Trait>
        struct propogate_cv_to_type<const Trait, std::void_t<typename Trait::value_type>> { 
            using type = const typename Trait::value_type; 
        }; 
        
        template<class Trait>
        struct propogate_cv_to_type<Trait volatile, std::void_t<typename Trait::value_type>> { 
            using type = volatile typename Trait::value_type; 
        }; 
        
        template<class Trait>
        struct propogate_cv_to_type<Trait const volatile, std::void_t<typename Trait::value_type>> { 
            using type = const volatile typename Trait::value_type; 
        }; 
    }
    
    template<typename T>
    using propagate_cv_value_type_t = typename detail::propogate_cv_to_type<T>::type;
    
    using namespace etl::Concepts;
    
    template<Integral T, uint8_t Base = 10>
    inline consteval /*constexpr*/ uint8_t numberOfDigits() {
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

    template<typename T, uint8_t Base = 10>
    requires (T::valid_bits > 0)
    inline consteval /*constexpr */ uint8_t numberOfDigits() {
        double v = 1.0;
        for(uint8_t b = 0; b < T::valid_bits; ++b) {
            v /= 2.0;
        }
        uint8_t number = 0;
        while((v - (uint64_t)v) > 0.0) {
            number += 1;
            v *= 10.0;
        }
        return number;
    }
    
    template<Unsigned T>
    inline constexpr bool isPowerof2(T v) {
        return v && ((v & (v - 1)) == 0);
    }
    
    template<typename Bit, typename T>
    inline constexpr bool isSet(T v) {
        if constexpr(std::is_same<T, std::byte>::value) {
            return std::to_integer<uint8_t>(Bit::template Value<T>::value) & std::to_integer<uint8_t>(v);
        }
        else {
            return Bit::template Value<T>::value & v;
        }
    }

    template<typename Bit, typename T>
    inline constexpr bool set(T& v) {
        return v |= Bit::template Value<T>::value;
    }
    
    struct MSB {
        template<typename T>
        struct Value {
            inline static constexpr const T value{T(1) << (numberOfBits<T>() - 1)};
        };
    };  
    struct LSB {
        template<typename T>
        struct Value {
            inline static constexpr const T value = 1;
        };
    };  
    template<typename T>
    consteval /*constexpr*/ uint8_t numberOfOnes(T x) {
        return (x != T{0}) ? T(x & T{0x01}) + numberOfOnes(T(x>>1u)) : 0u;
    }
    template<typename T>
    /*consteval */constexpr uint8_t minimumBitsForValue(const T& v) {
        for(uint8_t n = 1; n <= std::numeric_limits<uint8_t>::max(); ++n) {
            T max = T((1 << n) - 1);
            if (v <= max) {
                return n;
            }
        }
        return 0;
    }
}
