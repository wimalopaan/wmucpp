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
    
    template<Unsigned T, uint8_t Bits>
    struct Fraction final {
        static_assert(std::is_unsigned<T>::value, "T must be unsigned type");
        static_assert(etl::numberOfBits<T>() >= Bits, "type too small");
        using value_type = T;
        inline static constexpr uint8_t valid_bits = Bits;
        constexpr explicit Fraction(T v) : value(v) {} 
        const value_type value = 0;
    };

    template<typename Type, uint8_t fractionalBits>
    struct FixedPoint final {
        using value_type = Type;
        using unsigned_type = make_unsigned_t<Type>;
        using fractional_type = typeForBits_t<fractionalBits>;
        inline static constexpr unsigned_type fractional_mask = (1 << fractionalBits) - 1;
        inline static constexpr unsigned_type integral_mask = ~((1 << fractionalBits) - 1);
        inline static constexpr uint8_t fractional_bits = fractionalBits;
        inline static constexpr uint8_t integer_bits = sizeof(Type) * 8 - fractionalBits;
        inline static constexpr value_type one = 1u << fractional_bits;
        
        inline constexpr FixedPoint(const FixedPoint& o) : mValue{o.mValue} {}

        inline constexpr FixedPoint(volatile const FixedPoint& o) : mValue{o.mValue} {}
        
        inline constexpr explicit FixedPoint(double v) : mValue(v * one) {}
    
        inline constexpr FixedPoint() = default;
        
        inline constexpr void operator=(FixedPoint rhs) volatile {
            mValue = rhs.mValue;
        }
        
        inline static constexpr FixedPoint<Type, fractionalBits> fromRaw(value_type raw) {
            return FixedPoint<value_type, fractionalBits>{raw};
        }
        
        inline constexpr unsigned_type integerAbs() const {
            if (mValue < 0) {
                return -integer();
            }
            return integer();
        }
        inline constexpr fractional_type fractionalAbs() const {
            return fractional();
        }
        inline constexpr value_type integer() const {
            return mValue / one;
        }
        inline value_type integer() const volatile {
            return mValue / one;
        }
        inline constexpr fractional_type fractional() const {
            return (fractional_type(mValue) & fractional_mask);
        }
        inline fractional_type fractional() const volatile {
            return (fractional_type(mValue) & fractional_mask);
        }
        inline constexpr Fraction<fractional_type, fractionalBits> fraction() const {
            return Fraction<fractional_type, fractionalBits>(fractional() << ((sizeof(fractional_type) * 8) - fractional_bits));
        }
        inline constexpr const value_type& raw() const {
            return mValue;
        }
        inline constexpr FixedPoint& operator/=(Type d) {
            mValue /= d;
            return *this;
        }
        inline constexpr FixedPoint& operator-=(const FixedPoint& d) {
            mValue -= d.mValue;
            return *this;
        }
        inline constexpr FixedPoint& operator+=(const FixedPoint& d) {
            mValue += d.mValue;
            return *this;
        }
        // todo: op++
        // todo: op--
    private:
        explicit constexpr FixedPoint(value_type v) : mValue(v){}
        value_type mValue{0};
    };

    inline constexpr FixedPoint<int16_t, 4> operator"" _fp(long double v) {
        return FixedPoint<int16_t, 4>{(double)v};
    }
    inline constexpr FixedPoint<int16_t, 4> operator"" _Qs11_4(long double v) {
        return FixedPoint<int16_t, 4>{(double)v};
    }
    inline constexpr FixedPoint<uint16_t, 8> operator"" _Qu8_8(long double v) {
        return FixedPoint<uint16_t, 8>{(double)v};
    }
    inline constexpr FixedPoint<int16_t, 12> operator"" _Qs3_12(long double v) {
        return FixedPoint<int16_t, 12>{(double)v};
    }
    
    template<typename T, auto Bits>
    inline constexpr FixedPoint<T, Bits> operator-(FixedPoint<T, Bits> lhs, FixedPoint<T, Bits> rhs) {
        return lhs -= rhs;
    }
    
    template<typename T, auto Bits, typename D>
    inline constexpr FixedPoint<T, Bits> operator/(FixedPoint<T, Bits> lhs, const D& rhs) {
        return lhs /= rhs;
    }
    
    template<typename T, uint8_t FB1, uint8_t FB2>
    inline constexpr FixedPoint<T, FB1> operator*(FixedPoint<T, FB1> lhs, FixedPoint<T, FB2> rhs) {
        enclosingType_t<T> p = static_cast<enclosingType_t<T>>(lhs.raw()) * rhs.raw();
        return FixedPoint<T, FB1>::fromRaw(p >> FB2);
    }
    
    namespace detail {
        template<etl::Concepts::Stream Stream, typename T, uint8_t Bits>
        inline void out_impl(const Fraction<T, Bits>& f) {
            array<Char, 1 + numberOfDigits<Fraction<T, Bits>>()> buffer; // dot + sentinel
            ftoa(f, buffer);
            out_impl<Stream>(buffer);
        }
        
        template<etl::Concepts::Stream Stream, Signed T, auto Bits>
        inline void out_impl(const FixedPoint<T, Bits>& f) {
            if (f.raw() < 0) {
                out_impl<Stream>(Char{'-'});
            }
            out_impl<Stream>(f.integerAbs());
            out_impl<Stream>(f.fraction());
        }
    
        template<etl::Concepts::Stream Stream, Unsigned T, uint8_t Bits>
        inline void out_impl(const FixedPoint<T, Bits>& f) {
            out_impl<Stream>(f.integerAbs());
            out_impl<Stream>(f.fraction());
        }
        
    } // detail

}

