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
#include <type_traits>
#include <algorithm>
#include <ratio>
#include <compare>

#include "type_traits.h"
#include "concepts.h"
#include "char.h"

#include "ranged.h"

namespace etl {
    using namespace std;
    
    template<bool V>
    struct NamedFlag : integral_constant<bool, V> {};
    
    template<auto c>
    struct NamedConstant : integral_constant<decltype(c), c> {};
    
    template<typename T = uint8_t, typename R = std::byte>
    struct Parity {
        void operator+=(const T& v) {
            mValue += v;
        }
        void operator+=(const R& v) {
            mValue += T(v);
        }
        R operator<<=(const R& v) {
            mValue += T(v);
            return v;
        }
        R value() const {
            return R{mValue};
        }
        operator R() {
            return value();
        }
        void clear() {
            mValue = 0;
        }
    private:
        T mValue = 0;
    };
    
    template<auto Bits>
    class bitsN_t final {
    public:
        using value_type = typeForBits_t<Bits>;
        inline static constexpr auto size = Bits;
        inline static constexpr value_type mask = ((1 << Bits) - 1);
        inline constexpr bitsN_t(const volatile bitsN_t& o) : mValue{o.mValue} {}
        inline constexpr bitsN_t() = default;
        inline constexpr explicit bitsN_t(value_type v) : mValue(v & mask) {}
        inline constexpr explicit bitsN_t(std::byte v) : mValue(std::to_integer<value_type>(v) & mask) {}
        inline constexpr explicit operator value_type() const {
            return mValue;
        }
    private:
        value_type mValue{};
    };
    
    template<auto Bits>
    class uintN_t final {
    public:
        using value_type = typeForBits_t<Bits>;
        inline static constexpr value_type mask = ((1 << Bits) - 1);
        inline explicit uintN_t(value_type v = 0) : mValue(v & mask) {}
        inline constexpr operator value_type() const {
            return mValue;
        }
        inline constexpr uintN_t& operator++() {
            ++mValue;
            mValue &= mask;
            return *this;
        }
    private:
        value_type mValue{};
    };
    
    template<Unsigned T>
    class uint_NaN final {
        inline static constexpr T NaN = std::numeric_limits<T>::max();
    public:
        using value_type = T;
        
        inline explicit constexpr uint_NaN(T v) : mValue(v) {
            assert(mValue != NaN);
        }
        inline constexpr uint_NaN() : mValue(NaN) {}
        
        inline void setNaN() volatile {
            mValue = NaN;
        }
        inline void operator=(T v) volatile {
//            assert(v != NaN);
            mValue = v;
        }
        inline uint_NaN& operator=(T v){
//            assert(v != NaN);
            mValue = v;
            return *this;
        }
        inline explicit operator bool() volatile const {
            return mValue != NaN;
        }
        inline constexpr explicit operator bool() const {
            return mValue != NaN;
        }
        inline constexpr T toInt() const {
            assert(mValue != NaN);
            return mValue;
        }
        inline volatile T& operator*() volatile {
//            assert(mValue != NaN); // wegen Zuweisung
            return mValue;
        }
        inline T& operator*() {
//            assert(mValue != NaN); // wegen Zuweisung
            return mValue;
        }
        inline const T& operator*() const {
//            assert(mValue != NaN);
            return mValue;
        }
        inline void operator++() volatile {
            mValue = std::min(T(mValue + 1), T(NaN - 1));
            ++mValue;
        }
        inline void operator++() {
            mValue = std::min(T(mValue + 1), T(NaN - 1));
        }
        inline void operator--() volatile {
            if (mValue > 0) {
                --mValue;
            }
        }
        inline void operator--() {
            if (mValue > 0) {
                --mValue;
            }
        }
        inline constexpr bool operator==(uint_NaN rhs) const volatile {
            if (*this && rhs) {
                return mValue == rhs.mValue;
            }
            return false;
        }
        inline constexpr bool operator<=(uint_NaN rhs) const volatile {
            if (*this && rhs) {
                return mValue <= rhs.mValue;
            }
            return false;
        }
//    private: // strcutural
        T mValue{0};
    };
    
    
    
    template<typename T>
    struct combinedType final {
        using type = etl::enclosingType_t<T>;
        inline static constexpr const uint8_t shift = numberOfBits<T>();
    };
    
    template<typename T>
    inline typename combinedType<T>::type combinedValue(volatile const pair<T, T>& p) {
        return (p.first << combinedType<T>::shift) + p.second;
    }
    
    template<typename T>
    inline typename combinedType<T>::type combinedValue(const pair<T, T>& p) {
        return (p.first << combinedType<T>::shift) + p.second;
    }
    
    template<typename Representation, typename Scale = std::ratio<1,1>>
    struct ScaledInteger;
    
    template<typename ValueType, typename Scale, auto Min, auto Max>
    struct ScaledInteger<etl::uint_ranged<ValueType, Min, Max>, Scale> {
        using rep_type = etl::uint_ranged<ValueType, Min, Max>;
        using value_type = ValueType;
        using scale_type = Scale;
        using enc_type = etl::enclosing_t<value_type>;
        
//        enc_type::_;

        constexpr ScaledInteger(const value_type& v) : value{v} {}

        constexpr ScaledInteger(const ScaledInteger& v) : value{v.value} {}
        
        inline void operator++() volatile {
            ++value;            
        }
        inline void operator--() volatile {
            --value;            
        }
        inline bool operator>(value_type rhs) const volatile {
            return value > rhs;
        }
        inline void operator=(const ScaledInteger& rhs) volatile {
            value = rhs.value;
        }
        inline bool operator>(value_type rhs) const {
            return value > rhs;
        }
        inline void operator=(const ScaledInteger& rhs) {
            value = rhs.value;
        }
        
        inline value_type toInt() const {
            return (enc_type{value} * scale_type::nom) / scale_type::denom;
        }
        inline value_type toInt() const volatile {
            return (enc_type{value} * scale_type::nom) / scale_type::denom;
        }
//    private:
        rep_type value{0};
    };
    
    template<typename R, typename S, typename I, auto Min, auto Max>
    auto operator*(const I& f, const ScaledInteger<uint_ranged<R, Min, Max>, S>& s) -> R {
        using enc_type = ScaledInteger<uint_ranged<R, Min, Max>, S>::enc_type;
        return (enc_type{s.value} * f * S::nom) / S::denom;
    }
    template<typename R, typename S, typename I, auto Min, auto Max>
    auto operator*(const I& f, const volatile ScaledInteger<uint_ranged<R, Min, Max>, S>& s) -> R {
        using enc_type = ScaledInteger<uint_ranged<R, Min, Max>, S>::enc_type;
        return (enc_type{s.value} * f * S::nom) / S::denom;
    }
    
}

namespace std {
    template<>
    struct numeric_limits<etl::uint_NaN<uint8_t>> {
        using type = etl::uint_NaN<uint8_t>;
        inline static constexpr uint8_t max() {return UINT8_MAX - 1;}
        inline static constexpr uint8_t min() {return 0;}
    };
    
    template<>
    struct enable_bitmask_operators<etl::Char> : std::true_type {};
}
