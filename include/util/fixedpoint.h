/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdint.h>

#include "std/traits.h"

template<typename T>
struct Unsigned;

template<>
struct Unsigned<int8_t> {
    typedef uint8_t type;
};
template<>
struct Unsigned<int16_t> {
    typedef uint16_t type;
};
template<>
struct Unsigned<int32_t> {
    typedef uint32_t type;
};
template<>
struct Unsigned<uint8_t> {
    typedef uint8_t type;
};
template<>
struct Unsigned<uint16_t> {
    typedef uint16_t type;
};
template<>
struct Unsigned<uint32_t> {
    typedef uint32_t type;
};

template<typename T>
struct Fraction final {
    static_assert(std::is_unsigned<T>::value, "T must be unsigned type");
    typedef T value_type;
    
    constexpr explicit Fraction(T v) : value(v) {} 
    const T value = 0;
};


template<typename Type, uint8_t fractionalBits>
class FixedPoint final {
    template<typename Stream> friend Stream& operator<<(Stream& o, const FixedPoint<Type, fractionalBits>& f);

public:
    typedef Type value_type;
    typedef typename Unsigned<Type>::type unsigned_type;
    static constexpr unsigned_type fractional_mask = (1 << fractionalBits) - 1;
    static constexpr unsigned_type integral_mask = ~((1 << fractionalBits) - 1);
    static constexpr uint8_t fractional_bits = fractionalBits;
    static constexpr uint8_t integer_bits = sizeof(Type) * 8 - fractionalBits;
    static constexpr value_type one = 1 << fractional_bits;
    
    constexpr explicit FixedPoint(double v) : mValue(v * one) {}
    constexpr FixedPoint() = default;
    
    static FixedPoint<Type, fractionalBits> fromRaw(unsigned_type raw) {
        return FixedPoint<Type, fractionalBits>{raw};
    }
    
    unsigned_type integerAbs() const {
        if (mValue < 0) {
            return -integer();
        }
        return integer();
    }
    unsigned_type fractionalAbs() const {
        if (mValue < 0) {
            return -(fractional() | integral_mask);
        }
        return fractional();
    }
    Type integer() const {
        return mValue / one;
    }
    unsigned_type fractional() const {
        return (unsigned_type(mValue) & fractional_mask);
    }
    Fraction<unsigned_type> fraction() const {
        return Fraction<unsigned_type>{fractionalAbs() << integer_bits};
    }
    Type raw() const {
        return mValue;
    }
private:
    FixedPoint(unsigned_type v) : mValue(v){}
    const Type mValue = 0;
};

constexpr FixedPoint<int16_t, 4> operator"" _fp(long double v) {
    return FixedPoint<int16_t, 4>{(double)v};
}

