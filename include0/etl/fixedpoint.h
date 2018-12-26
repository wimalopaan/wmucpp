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

#pragma once

#include <cstdint>
#include <type_traits>
#include <array>

#include "concepts.h"
#include "output.h"
    
namespace etl {
    using namespace std;
    
    template<typename T, uint8_t Bits>
    struct Fraction final {
        static_assert(std::is_unsigned<T>::value, "T must be unsigned type");
        static_assert(numberOfBits<T>() >= Bits, "type too small");
        typedef T value_type;
        inline static constexpr uint8_t valid_bits = Bits;
        constexpr explicit Fraction(T v) : value(v) {} 
        const T value = 0;
    };

    template<typename Type, uint8_t fractionalBits>
    class FixedPoint final {
        template<typename Stream> friend Stream& operator<<(Stream& o, const FixedPoint<Type, fractionalBits>& f);
    public:
        typedef Type value_type;
        typedef make_unsigned_t<Type> unsigned_type;
        typedef typeForBits_t<fractionalBits> fractional_type;
        static constexpr unsigned_type fractional_mask = (1 << fractionalBits) - 1;
        static constexpr unsigned_type integral_mask = ~((1 << fractionalBits) - 1);
        static constexpr uint8_t fractional_bits = fractionalBits;
        static constexpr uint8_t integer_bits = sizeof(Type) * 8 - fractionalBits;
        static constexpr value_type one = 1u << fractional_bits;
        typedef typeForBits_t<integer_bits> integral_type;
        
        inline constexpr explicit FixedPoint(double v) : mValue(v * one) {}
    
        inline constexpr FixedPoint() = default;
        
        inline static FixedPoint<Type, fractionalBits> fromRaw(unsigned_type raw) {
            return FixedPoint<Type, fractionalBits>{raw};
        }
        
        inline unsigned_type integerAbs() const {
            if (mValue < 0) {
                return -integer();
            }
            return integer();
        }
        inline fractional_type fractionalAbs() const {
            return fractional();
        }
        inline integral_type integer() const {
            return mValue / one;
        }
        inline fractional_type fractional() const {
            return (fractional_type(mValue) & fractional_mask);
        }
        inline Fraction<fractional_type, fractionalBits> fraction() const {
            return Fraction<fractional_type, fractionalBits>(fractional() << ((sizeof(fractional_type) * 8) - fractional_bits));
        }
        inline const Type& raw() const {
            return mValue;
        }
        inline FixedPoint& operator/=(Type d) {
            mValue /= d;
            return *this;
        }
        inline FixedPoint& operator-=(const FixedPoint& d) {
            mValue -= d.mValue;
            return *this;
        }
        inline FixedPoint& operator+=(const FixedPoint& d) {
            mValue += d.mValue;
            return *this;
        }
        // todo: op++
        // todo: op--
    private:
        explicit FixedPoint(unsigned_type v) : mValue(v){}
        Type mValue = 0;
    };
    
    // todo: UDL-op for FP<uint16_t, 8>
    
    inline constexpr FixedPoint<int16_t, 4> operator"" _fp(long double v) {
        return FixedPoint<int16_t, 4>{(double)v};
    }
    
    template<typename T, auto Bits>
    inline FixedPoint<T, Bits> operator-(FixedPoint<T, Bits> lhs, const FixedPoint<T, Bits>& rhs) {
        return lhs -= rhs;
    }
    
    template<typename T, auto Bits, typename D>
    inline FixedPoint<T, Bits> operator/(FixedPoint<T, Bits> lhs, const D& rhs) {
        return lhs /= rhs;
    }
    
    template<typename T, uint8_t FB>
    inline FixedPoint<T, FB> operator*(FixedPoint<T, FB> lhs, FixedPoint<T, FB> rhs) {
        enclosingType_t<T> p = lhs.raw() * rhs.raw();
        return FixedPoint<T, FB>::fromRaw(p >> FB);
    }
    
    namespace detail {
        template<AVR::Concepts::Stream Stream, typename T, uint8_t Bits>
        inline void out_impl(const Fraction<T, Bits>& f) {
            array<Char, 1 + numberOfDigits<Fraction<T, Bits>>()> buffer; // dot + sentinel
            ftoa(f, buffer);
            out_impl<Stream>(buffer);
        }
        
        template<AVR::Concepts::Stream Stream, Signed T, auto Bits>
        inline void out_impl(const FixedPoint<T, Bits>& f) {
            if (f.raw() < 0) {
                out_impl<Stream>(Char{'-'});
            }
            out_impl<Stream>(f.integerAbs());
            out_impl<Stream>(f.fraction());
        }
    
        template<AVR::Concepts::Stream Stream, Unsigned T, uint8_t Bits>
        inline void out_impl(const FixedPoint<T, Bits>& f) {
            out_impl<Stream>(f.integerAbs());
            out_impl<Stream>(f.fraction());
        }
        
    } // detail

}

