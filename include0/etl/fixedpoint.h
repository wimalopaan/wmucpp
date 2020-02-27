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
        const value_type value{};
    };

    namespace detail {
        struct Overflow;
        struct Saturate;
    }
    
    template<typename Type, uint8_t fractionalBits, typename OvflPol>
    struct FixedPoint final {
        struct Internal{};
        using value_type = Type;
        using unsigned_type = make_unsigned_t<Type>;
        using fractional_type = typeForBits_t<fractionalBits>;
        inline static constexpr unsigned_type fractional_mask = (1 << fractionalBits) - 1;
        inline static constexpr unsigned_type integral_mask = ~((1 << fractionalBits) - 1);
        inline static constexpr uint8_t fractional_bits = fractionalBits;
        inline static constexpr uint8_t integer_bits = sizeof(Type) * 8 - fractionalBits;
        inline static constexpr value_type one = unsigned_type{1} << fractional_bits;

        using integer_type = std::conditional_t<std::is_signed_v<value_type>, 
                                                std::make_signed_t<etl::typeForBits_t<integer_bits>>, 
                                                etl::typeForBits_t<integer_bits>>;

        inline constexpr explicit FixedPoint(const integer_type v) : mValue{value_type{v} << fractionalBits} {}
                
        inline constexpr FixedPoint(const FixedPoint& o) : mValue{o.mValue} {}

        inline constexpr FixedPoint(volatile const FixedPoint& o) : mValue{o.mValue} {}
        
        inline /*constexpr */ consteval explicit FixedPoint(double v) : mValue(v * one) {}
    
        inline constexpr FixedPoint() = default;

        static inline constexpr FixedPoint max() {
            return FixedPoint::fromRaw(std::numeric_limits<value_type>::max());
        }
        static inline constexpr FixedPoint min() {
            return FixedPoint::fromRaw(std::numeric_limits<value_type>::min());
        }
        
        inline constexpr void operator=(const FixedPoint rhs) volatile {
            mValue = rhs.mValue;
        }

        inline constexpr bool operator==(const FixedPoint rhs) const {
            return mValue == rhs.mValue;
        }
        inline constexpr bool operator!=(const FixedPoint rhs) const {
            return !(*this == rhs);
        }
        
        inline static constexpr FixedPoint fromRaw(const value_type raw) {
            return FixedPoint{raw, Internal{}};
        }

        inline constexpr unsigned_type integerAbs() const {
            if (mValue < 0) {
                return -integer();
            }
            return integer();
        }
        inline constexpr unsigned_type integerAbs() const volatile {
            if (mValue < 0) {
                return -integer();
            }
            return integer();
        }
        inline constexpr fractional_type fractionalAbs() const {
            return fractional();
        }
        inline constexpr value_type integer() const {
            if constexpr(etl::has_arithmetic_right_shift_v<enclosingType_t<Type>>) {
                return mValue >> fractionalBits;                
            }
            else {
                return mValue / one;
            }
        }
        inline value_type integer() const volatile {
            if constexpr(etl::has_arithmetic_right_shift_v<enclosingType_t<Type>>) {
                return mValue >> fractionalBits;                
            }
            else {
                return mValue / one;
            }
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
        inline constexpr Fraction<fractional_type, fractionalBits> fraction() const volatile {
            return Fraction<fractional_type, fractionalBits>(fractional() << ((sizeof(fractional_type) * 8) - fractional_bits));
        }
        inline constexpr const value_type raw() const {
            return mValue;
        }
        inline constexpr FixedPoint& operator/=(const Type d) {
            mValue /= d;
            return *this;
        }
        inline constexpr FixedPoint& operator*=(const Type d) {
            mValue *= d;
            return *this;
        }

        inline constexpr FixedPoint operator/(const FixedPoint rhs) const {
            const enclosingType_t<Type> p = enclosingType_t<value_type>{mValue} << fractionalBits;
            value_type p1 = p / rhs.mValue;
            return FixedPoint{p1, Internal{}};
        }   
        
        inline constexpr FixedPoint operator*(const FixedPoint rhs) const {
            const enclosingType_t<value_type> p = enclosingType_t<value_type>{mValue} * rhs.mValue;
            value_type p1;
            if constexpr(etl::has_arithmetic_right_shift_v<enclosingType_t<value_type>>) {
                p1 = p >> fractionalBits; 
            }
            else {
                p1 = p / (make_unsigned_t<value_type>{1} << fractionalBits);
            }
            if (p & (fractional_type{1} << (fractionalBits - 1))) {
                     p1 += value_type{1};
            }
            return FixedPoint{p1, Internal{}};
        }        
        
        inline constexpr void operator*=(const Type d) volatile {
            mValue = mValue * d;
        }
        inline constexpr FixedPoint& operator-=(const FixedPoint d) {
            mValue -= d.mValue;
            return *this;
        }
        inline constexpr FixedPoint& operator+=(const FixedPoint d) {
            mValue += d.mValue;
            return *this;
        }
        inline constexpr void operator-=(const FixedPoint d) volatile {
            mValue = mValue - d.mValue;
        }
        inline constexpr void operator+=(const FixedPoint d) volatile {
            mValue = mValue + d.mValue;
        }
    private:
        explicit constexpr FixedPoint(value_type v, Internal) : mValue{v}{}
        value_type mValue{};
    };

    inline /*constexpr*/ consteval FixedPoint<int16_t, 4> operator"" _fp(const long double v) {
        return FixedPoint<int16_t, 4>{(double)v};
    }
    inline /*constexpr*/ consteval FixedPoint<int16_t, 4> operator"" _Qs11_4(const long double v) {
        return FixedPoint<int16_t, 4>{(double)v};
    }
    inline /*constexpr*/ consteval FixedPoint<uint16_t, 8> operator"" _Qu8_8(const long double v) {
        return FixedPoint<uint16_t, 8>{(double)v};
    }
    inline /*constexpr*/ consteval FixedPoint<int16_t, 12> operator"" _Qs3_12(const long double v) {
        return FixedPoint<int16_t, 12>{(double)v};
    }
    inline /*constexpr*/ consteval FixedPoint<uint8_t, 7> operator"" _Qu1_7(const long double v) {
        return FixedPoint<uint8_t, 7>{(double)v};
    }

    template<typename T, auto Bits>
    inline constexpr bool operator>(const FixedPoint<T, Bits> lhs, const FixedPoint<T, Bits> rhs) {
        return lhs.raw() > rhs.raw();
    }
    template<typename T, auto Bits>
    inline constexpr bool operator<(const FixedPoint<T, Bits> lhs, const FixedPoint<T, Bits> rhs) {
        return lhs.raw() < rhs.raw();
    }
    
    template<typename T, auto Bits>
    inline constexpr FixedPoint<T, Bits> operator-(FixedPoint<T, Bits> lhs, const FixedPoint<T, Bits> rhs) {
        return lhs -= rhs;
    }
    template<typename T, auto Bits>
    inline constexpr FixedPoint<T, Bits> operator+(FixedPoint<T, Bits> lhs, const FixedPoint<T, Bits> rhs) {
        return lhs += rhs;
    }
    
    template<typename T, auto Bits, typename D>
    inline constexpr FixedPoint<T, Bits> operator/(FixedPoint<T, Bits> lhs, const D rhs) {
        return lhs /= rhs;
    }

    template<typename T, auto Bits, typename D>
    inline constexpr FixedPoint<T, Bits> operator*(FixedPoint<T, Bits> lhs, const D rhs) {
        return lhs *= rhs;
    }

    template<typename T, auto Bits, typename D>
    inline constexpr D operator*(const D lhs, const FixedPoint<T, Bits> rhs) {
        return (etl::enclosing_t<D>{lhs} * rhs.raw()) >> Bits;
    }

    namespace detail {
        template<etl::Concepts::Stream Stream, typename T, uint8_t Bits>
        inline void out_impl(const Fraction<T, Bits>& f) {
            array<Char, 1 + numberOfDigits<Fraction<T, Bits>>()> buffer; // dot + sentinel
            ftoa(f, buffer);
            out_impl<Stream>(buffer);
        }
        
        template<etl::Concepts::Stream Stream, etl::Concepts::Signed T, uint8_t Bits>
        inline void out_impl(const FixedPoint<T, Bits>& f) {
            if (f.raw() < 0) {
                out_impl<Stream>(Char{'-'});
            }
            out_impl<Stream>(f.integerAbs());
            out_impl<Stream>(f.fraction());
        }
    
        template<etl::Concepts::Stream Stream, etl::Concepts::Unsigned T, uint8_t Bits>
        inline void out_impl(const FixedPoint<T, Bits>& f) {
            out_impl<Stream>(f.integerAbs());
            out_impl<Stream>(f.fraction());
        }

        template<etl::Concepts::Stream Stream, etl::Concepts::Unsigned T, uint8_t Bits>
        inline void out_impl(const volatile FixedPoint<T, Bits>& f) {
            out_impl<Stream>(f.integerAbs());
            out_impl<Stream>(f.fraction());
        }

        namespace tests {
            consteval bool tests_qs78() {
                using fp_t = etl::FixedPoint<int16_t, 8>; 
                using fp_int_t = fp_t::integer_type;
                
                if((fp_t{101.0} * fp_t{1.25}) != fp_t{101.0 * 1.25}) return false;
                if((fp_t{101.0} * fp_t{1.125}) != fp_t{101.0 * 1.125}) return false;
                if((fp_t{101.0} * fp_t{1.0625}) != fp_t{101.0 * 1.0625}) return false;
                if((fp_t{101.0} * fp_t{1.03125}) != fp_t{101.0 * 1.03125}) return false;
                if((fp_t{101.0} * fp_t{1.015625}) != fp_t{101.0 * 1.015625}) return false;
                if((fp_t{101.0} * fp_t{1.0078125}) != fp_t{101.0 * 1.0078125}) return false;
                if((fp_t{101.0} * fp_t{1.00390625}) != fp_t{101.0 * 1.00390625}) return false;
                if((fp_t{101.0} * fp_t{1.0 - 0.00390625}) != fp_t{101.0 * (1.0 - 0.00390625)}) return false;
                
                if((fp_t{-101.0} * fp_t{1.25}) != fp_t{-101.0 * 1.25}) return false;
                if((fp_t{101.0} * fp_t{-1.25}) != fp_t{-101.0 * 1.25}) return false;
                
                if (fp_t{1.0} != fp_t{fp_int_t{1}}) return false;
                if (fp_t{1.0}.integer() != 1) return false;
                
                if((fp_t{1.0} + fp_t{1.0}) != fp_t{1.0 + 1.0}) return false;
                if((fp_t{0.125} + fp_t{0.125}) != fp_t{0.125 + 0.125}) return false;
                
                if((fp_t{0.125} + fp_t{0.75}) != fp_t{0.125 + 0.75}) return false;

                if((fp_t{2.5} / fp_t{2.0}) != fp_t{2.5 / 2.0}) return false;
                if((fp_t{2.125} / fp_t{2.0}) != fp_t{2.125 / 2.0}) return false;
                
                return true;
            }
            static_assert(tests_qs78());

            consteval bool tests_qu88() {
                using fp_t = etl::FixedPoint<uint16_t, 8>; 
                using fp_int_t = fp_t::integer_type;
                
                if((fp_t{101.0} * fp_t{1.25}) != fp_t{101.0 * 1.25}) return false;
                if((fp_t{101.0} * fp_t{1.125}) != fp_t{101.0 * 1.125}) return false;
                if((fp_t{101.0} * fp_t{1.0625}) != fp_t{101.0 * 1.0625}) return false;
                if((fp_t{101.0} * fp_t{1.03125}) != fp_t{101.0 * 1.03125}) return false;
                if((fp_t{101.0} * fp_t{1.015625}) != fp_t{101.0 * 1.015625}) return false;
                if((fp_t{101.0} * fp_t{1.0078125}) != fp_t{101.0 * 1.0078125}) return false;
                if((fp_t{101.0} * fp_t{1.00390625}) != fp_t{101.0 * 1.00390625}) return false;
                if((fp_t{101.0} * fp_t{1.0 - 0.00390625}) != fp_t{101.0 * (1.0 - 0.00390625)}) return false;
                
                if (fp_t{1.0} != fp_t{fp_int_t{1}}) return false;
                if (fp_t{1.0}.integer() != 1) return false;
                
                if((fp_t{1.0} + fp_t{1.0}) != fp_t{1.0 + 1.0}) return false;
                if((fp_t{0.125} + fp_t{0.125}) != fp_t{0.125 + 0.125}) return false;
                
                if((fp_t{0.125} + fp_t{0.75}) != fp_t{0.125 + 0.75}) return false;

                if((fp_t{2.5} / fp_t{2.0}) != fp_t{2.5 / 2.0}) return false;
                if((fp_t{2.125} / fp_t{2.0}) != fp_t{2.125 / 2.0}) return false;
                
                return true;
            }
            static_assert(tests_qu88());

            consteval bool tests_qu79() {
                using fp_t = etl::FixedPoint<uint16_t, 9>; 
                using fp_int_t = fp_t::integer_type;
                
                if((fp_t{101.0} * fp_t{1.001953125}) != fp_t{101.0 * 1.001953125}) return false;
                if((fp_t{101.0} * fp_t{1.0 - 0.00390625}) != fp_t{101.0 * (1.0 - 0.00390625)}) return false;
                
                if (fp_t{1.0} != fp_t{fp_int_t{1}}) return false;
                if (fp_t{1.0}.integer() != 1) return false;
                
                return true;
            }
            static_assert(tests_qu79());
        
            consteval bool tests_qu97() {
                using fp_t = etl::FixedPoint<uint16_t, 7>; 
                using fp_int_t = fp_t::integer_type;
                
                if((fp_t{101.0} * fp_t{1.25}) != fp_t{101.0 * 1.25}) return false;
                if((fp_t{101.0} * fp_t{1.125}) != fp_t{101.0 * 1.125}) return false;
                if((fp_t{101.0} * fp_t{1.0625}) != fp_t{101.0 * 1.0625}) return false;
                if((fp_t{101.0} * fp_t{1.03125}) != fp_t{101.0 * 1.03125}) return false;
                if((fp_t{101.0} * fp_t{1.015625}) != fp_t{101.0 * 1.015625}) return false;

                if (fp_t{1.0} != fp_t{fp_int_t{1}}) return false;
                if (fp_t{1.0}.integer() != 1) return false;
                
                return true;
            }
            static_assert(tests_qu97());
            
            consteval bool tests_udl() {
                const auto f1 = 1.25_Qu8_8;
                const auto f2 = 256.25_fp;
                if (f1.integer() != 1) return false;
                if (f2.integer() != 256) return false;
                return true;
            }
            static_assert(tests_udl());
        }
    } // detail
}

