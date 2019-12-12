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

#include "type_traits.h"
#include "concepts.h"

#include "char.h"

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
//        R operator<<=(const T& v) {
//            mValue += v;
//            return v;
//        }
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
//            assert(mValue != NaN);
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
            assert(mValue != NaN);
            return mValue;
        }
        inline void operator++() volatile {
            mValue = std::min(T(mValue + 1), T(NaN - 1));
            ++mValue;
        }
        inline uint_NaN& operator++() {
            mValue = std::min(T(mValue + 1), T(NaN - 1));
            return *this;
        }
        inline void operator--() volatile {
            if (mValue > 0) {
                --mValue;
            }
        }
        inline uint_NaN& operator--() {
            if (mValue > 0) {
                --mValue;
            }
            return *this;
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
    private:
        T mValue = 0;
    };
    
 
    template<Unsigned T = uint8_t, T LowerBound = 0, T UpperBound = std::numeric_limits<T>::max()>
    requires(LowerBound <= UpperBound)
    class uint_ranged final {
    public:
        inline static constexpr T Lower = LowerBound;
        inline static constexpr T Upper = UpperBound;
        using value_type = T;
        
        inline constexpr uint_ranged(T v = LowerBound) : mValue(v) {
            assert(v >= LowerBound);
            assert(v <= UpperBound);
            if (v < LowerBound) {
                mValue = LowerBound;
            }
            else if (v > UpperBound) {
                mValue = UpperBound;
            }
        }

//        inline constexpr uint_ranged(const volatile uint_ranged& o) : mValue(o.mValue) {}

        inline constexpr explicit operator bool() const {
            return (mValue >= LowerBound) && (mValue <= UpperBound);
        }
        
        inline constexpr bool isTop() const {
            return mValue == Upper;
        }
        
        inline constexpr bool isBottom() const {
            return mValue == Lower;
        }
        
//        inline constexpr bool operator==(T rhs) const {
//            return mValue == rhs;
//        }
//        inline constexpr bool operator>=(T rhs) const {
//            return mValue >= rhs;
//        }
//        inline constexpr bool operator>(T rhs) const {
//            return mValue > rhs;
//        }
//        inline constexpr bool operator<(T rhs) const {
//            return mValue < rhs;
//        }
//        inline constexpr bool operator>(T rhs) const volatile {
//            return mValue > rhs;
//        }
        
//        inline auto operator<=>(const uint_ranged&) const = default;
        
        inline uint_ranged& operator--() {
            if (mValue > LowerBound) {
                --mValue;
            }
            return *this;
        }
        inline void operator++() volatile {
            if (mValue < UpperBound) {
                mValue = mValue + 1;
            }
        }
        inline uint_ranged& operator++() {
            if (mValue < UpperBound) {
                ++mValue;
            }
            return *this;
        }
        inline void operator--() volatile {
            if (mValue > LowerBound) {
                mValue = mValue - 1;
            }
        }
        
        inline uint_ranged& operator+=(T rhs) {
            if ((mValue + rhs) <= UpperBound) {
                mValue += rhs;
            }
            return *this;
        }
        inline void operator+=(T rhs) volatile {
            if ((mValue + rhs) <= UpperBound) {
                mValue = mValue + rhs;
            }
        }
        inline uint_ranged& operator-=(T rhs) {
            if (mValue >= (LowerBound + rhs)) {
                mValue -= rhs;
            }
            return *this;
        }
        inline void operator-=(T rhs) volatile {
            if (mValue >= (LowerBound + rhs)) {
                mValue = mValue - rhs;
            }
        }
        
        inline constexpr uint_ranged& operator=(const uint_ranged&) = default;

        inline constexpr uint_ranged& operator=(T rhs) {
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            mValue = rhs;
            return *this;
        }
        inline constexpr void operator=(T rhs) volatile {
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            mValue = rhs;
        }
        constexpr operator T() const {
            return mValue;
        }
        constexpr operator T() volatile const {
            return mValue;
        }
        inline constexpr T toInt() const {
            return mValue;
        }
        inline constexpr T toInt() volatile const {
            return mValue;
        }
    private:
        T mValue{LowerBound};
    };

    template<Unsigned T = uint8_t, T LowerBound = 0, T UpperBound = std::numeric_limits<T>::max() - 1>
    class uint_ranged_NaN final {
    public:
        inline static constexpr T Lower = LowerBound;
        inline static constexpr T Upper = UpperBound;
        inline static constexpr T Mid = (UpperBound + LowerBound) / 2;
        inline static constexpr T NaN   = std::numeric_limits<T>::max();
        static_assert(Upper != NaN);

        using value_type = T;
        
        inline constexpr uint_ranged_NaN() = default;
        
        inline constexpr uint_ranged_NaN(T v) : mValue(v) {
            assert(v >= LowerBound);
            assert(v <= UpperBound);
            if (v < LowerBound) {
                mValue = LowerBound;
            }
            else if (v > UpperBound) {
                mValue = UpperBound;
            }
        }

        inline constexpr explicit operator bool() const {
            return mValue != NaN;
        }

        template<typename R>
        inline constexpr uint_ranged_NaN operator+(R rhs) const {
            if ((mValue + rhs) > Upper) {
                return uint_ranged_NaN{Upper};
            }
            else if ((mValue + rhs) < Lower) {
                return uint_ranged_NaN{Lower};
            }
            return uint_ranged_NaN(mValue + rhs);
        }
        
//        inline constexpr bool operator>(T rhs) const {
//            return mValue > rhs;
//        }
        inline uint_ranged_NaN& operator--() {
            if (mValue > LowerBound) {
                --mValue;
            }
            return *this;
        }
        inline uint_ranged_NaN& operator++() {
            if (mValue < UpperBound) {
                ++mValue;
            }
            return *this;
        }
//        inline constexpr bool operator==(T rhs) const {
//            return mValue == rhs;
//        }
        inline constexpr uint_ranged_NaN& operator=(T rhs) {
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            mValue = std::clamp(rhs, LowerBound, UpperBound);
            return *this;
        }
        
        inline constexpr T toInt() const {
            return mValue;
        }
        constexpr operator T() const {
            return mValue;
        }
        
        template<typename TO>
        inline constexpr TO mapTo() const {
            return (etl::enclosing_t<T>(mValue - Lower) * std::numeric_limits<TO>::max()) / (Upper - Lower);
        }
        
        inline constexpr uint_ranged_NaN<T, LowerBound, UpperBound> invert() const {
            if (*this) {
                return uint_ranged_NaN<T, LowerBound, UpperBound>((UpperBound - mValue) + LowerBound);
            }
            return *this;
        }
        inline constexpr void operator+=(T value) {
            mValue = std::min(UpperBound, mValue + value);
        }
        inline constexpr void operator/=(T d) {
            mValue = std::clamp(mValue / d, LowerBound, UpperBound);
        }
    private:
        T mValue{NaN};
    };

    template<Signed T = int8_t, T LowerBound = std::numeric_limits<T>::min(), T UpperBound = std::numeric_limits<T>::max() - 1>
    class int_ranged_NaN final {
    public:
        inline static constexpr T Lower = LowerBound;
        inline static constexpr T Upper = UpperBound;
        inline static constexpr T NaN   = std::numeric_limits<T>::max();
        using value_type = T;
        
        static_assert(Upper != NaN);
        
//        using type = T;
        
        inline constexpr int_ranged_NaN() = default;
        
        inline constexpr int_ranged_NaN(T v) : mValue(v) {
            assert(v >= LowerBound);
            assert(v <= UpperBound);
        }

//        inline constexpr uint_ranged_NaN(etl::fragmentType_t<T> higherPart, etl::fragmentType_t<T> lowerPart) :
//            uint_ranged_NaN((static_cast<T>(higherPart) << etl::numberOfBits<etl::fragmentType_t<T>>()) + lowerPart)
//        {}
        
        inline constexpr explicit operator bool() const {
            return mValue != NaN;
        }
        
        inline constexpr bool operator>(T rhs) const {
            return mValue > rhs;
        }
        inline constexpr bool operator>=(T rhs) const {
            return mValue >= rhs;
        }
        inline constexpr bool operator<(T rhs) const {
            return mValue < rhs;
        }
        inline constexpr bool operator<=(T rhs) const {
            return mValue <= rhs;
        }

        inline int_ranged_NaN& operator--() {
            if (mValue > LowerBound) {
                --mValue;
            }
            return *this;
        }
        inline int_ranged_NaN& operator++() {
            if (mValue < UpperBound) {
                ++mValue;
            }
            return *this;
        }
        inline constexpr bool operator==(T rhs) const {
            return mValue == rhs;
        }
        inline constexpr int_ranged_NaN& operator=(T rhs) {
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            mValue = std::clamp(rhs, LowerBound, UpperBound);
            return *this;
        }
        
        inline constexpr T toInt() const {
            return mValue;
        }
        
//        template<typename TO>
//        inline constexpr TO mapTo() const {
//            return (etl::enclosing_t<T>(mValue - Lower) * std::numeric_limits<TO>::max()) / (Upper - Lower);
//        }
        
//        inline constexpr uint_ranged_NaN<T, LowerBound, UpperBound> invert() const {
//            if (*this) {
//                return uint_ranged_NaN<T, LowerBound, UpperBound>((UpperBound - mValue) + LowerBound);
//            }
//            return *this;
//        }
        inline constexpr void operator+=(T value) {
            mValue = std::min(UpperBound, mValue + value);
        }
        inline constexpr void operator/=(T value) {
            mValue /= value;
        }
    private:
        T mValue{NaN};
    };
    
    template<Unsigned T = uint8_t, T LowerBound = 0, T UpperBound = std::numeric_limits<T>::max()>
    class uint_ranged_circular final {
        static_assert(LowerBound <= UpperBound);
    public:
        inline static constexpr T Lower = LowerBound;
        inline static constexpr T Upper = UpperBound;
        using value_type = T;
        
        inline constexpr uint_ranged_circular() = default;
        
        inline constexpr explicit uint_ranged_circular(T v) : mValue(v) {
            assert(v >= LowerBound);
            assert(v <= UpperBound);
        }

        inline constexpr uint_ranged_circular(const uint_ranged_circular& rhs) = default;
        
        inline constexpr void operator=(const uint_ranged_circular& rhs) volatile {
              mValue = rhs.mValue;
        }
        
        inline constexpr bool operator>(T rhs) const {
            return mValue > rhs;
        }
        inline uint_ranged_circular& operator--() {
            if (mValue > LowerBound) {
                --mValue;
            }
            else {
                mValue = UpperBound;
            }
            return *this;
        }
        inline uint_ranged_circular& operator++() {
            if (mValue < UpperBound) {
                ++mValue;
            }
            else {
                mValue = LowerBound;
            }
            return *this;
        }

        template<T Shift>
        inline uint_ranged_circular leftShift() volatile {
            static_assert(Shift >= 0);
            static_assert(Shift <= Upper);
            if (Shift <= mValue) {
                return uint_ranged_circular(mValue - Shift);
            }
            else {
                return uint_ranged_circular(mValue + (Upper + 1 - T{Shift}));
            }
        }
        
        inline void increment() {
            if (mValue == Upper) {
                mValue = Lower;
            }
            else {
                mValue = mValue + 1;
            }
        }
        inline void increment() volatile {
            if (mValue == Upper) {
                mValue = 0;
            }
            else {
                mValue = mValue + 1;
            }
        }
        
        inline uint_ranged_circular operator+(value_type rhs) {
            return uint_ranged_circular((mValue + rhs) % (UpperBound + 1));
        }
        inline uint_ranged_circular operator+(value_type rhs) volatile {
            return uint_ranged_circular((mValue + rhs) % (UpperBound + 1));
        }
        inline uint_ranged_circular operator-(value_type rhs) {
            return uint_ranged_circular((mValue + (UpperBound + 1 - rhs)) % (UpperBound + 1));
        }
        inline uint_ranged_circular operator-(value_type rhs) volatile {
            return uint_ranged_circular((mValue + (UpperBound + 1 - rhs)) % (UpperBound + 1));
        }
        
        inline bool operator==(value_type rhs) {
            return mValue == rhs;
        }

        inline bool operator==(value_type rhs) volatile {
            return mValue == rhs;
        }
        
        inline bool operator!=(value_type rhs) {
            return mValue != rhs;
        }

        inline void operator++() volatile {
            if (mValue < UpperBound) {
                ++mValue;
            }
            else {
                mValue = LowerBound;
            }
        }
        inline void  operator--() volatile {
            if (mValue > LowerBound) {
                --mValue;
            }
            else {
                mValue = UpperBound;
            }
        }
        inline constexpr uint_ranged_circular& operator=(T rhs) {
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            mValue = rhs;
            return *this;
        }
        inline constexpr void operator=(T rhs) volatile {
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            mValue = rhs;
        }
        inline constexpr uint_ranged<T, LowerBound, UpperBound> toRanged() const {
            return {mValue};
        }
        inline constexpr T toInt() const {
            return mValue;
        }
        inline constexpr T toInt() const volatile {
            return mValue;
        }
    private:
        T mValue{LowerBound};
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
