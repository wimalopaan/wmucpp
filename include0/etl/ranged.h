#pragma once

#include <cstdint>
#include <algorithm>
#include <array>
#include "etl/concepts.h"

namespace etl {
    template<Unsigned T = uint8_t, T LowerBound = 0, T UpperBound = std::numeric_limits<T>::max()>
    class uint_ranged_circular;
    
    template<Unsigned T = uint8_t, T LowerBound = 0, T UpperBound = std::numeric_limits<T>::max()>
    class uint_ranged;

    namespace detail {
        template<typename A> struct index_type;
        
        template<typename T, auto S>
        struct index_type<std::array<T, S>> final {
            using size_type = std::array<T, S>::size_type;
            using type = etl::uint_ranged<size_type, 0, S - 1>;
        };
    }
    
    template<typename A>
    using index_type_t = detail::index_type<A>::type;
    
    template<bool B>
    using RangeCheck = std::integral_constant<bool, B>;
    
    template<Unsigned T, T LowerBound, T UpperBound>
    class uint_ranged final {
        static_assert(LowerBound <= UpperBound);
    public:
        inline static constexpr T Lower = LowerBound;
        inline static constexpr T Upper = UpperBound;
        using value_type = T;
     
        inline static constexpr uint_ranged upper() {
            return uint_ranged{Upper};
        }
        inline static constexpr uint_ranged lower() {
            return uint_ranged{Lower};
        }
        
        inline constexpr uint_ranged() = default; // LowerBound
        
        inline constexpr explicit uint_ranged(T v){
            assert(v >= LowerBound);
            assert(v <= UpperBound);
            mValue = std::clamp({v}, LowerBound, UpperBound);
        }
        
        template<T L, T U>
        requires((L >= LowerBound) && (U <= UpperBound))
        inline constexpr uint_ranged(const uint_ranged<T, L, U>& v) {
            mValue = std::clamp(T{v}, LowerBound, UpperBound);
        }
        
        inline constexpr uint_ranged(const uint_ranged&) = default;
        inline constexpr uint_ranged(volatile const uint_ranged& v) : mValue{v}{}
        
        inline constexpr uint_ranged(uint_ranged_circular<T, LowerBound, UpperBound> v) : mValue{v} {}
        
        inline constexpr bool isTop() const {
            return mValue == Upper;
        }
        
        inline constexpr bool isBottom() const {
            return mValue == Lower;
        }
        
        inline constexpr void setToBottom() volatile {
            mValue = LowerBound;
        }
        inline constexpr void setToTop() {
            mValue = UpperBound;
        }

        inline void operator--() {
            if (mValue > LowerBound) {
                --mValue;
            }
        }
        inline void operator++() volatile {
            if (mValue < UpperBound) {
                mValue = mValue + 1;
            }
        }
        inline uint_ranged operator++() {
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
        
        inline constexpr void operator+=(T rhs) {
            if ((mValue + rhs) <= UpperBound) {
                mValue += rhs;
            }
            else {
                mValue = UpperBound;
            }
        }
        inline constexpr void operator+=(T rhs) volatile {
            if ((mValue + rhs) <= UpperBound) {
                mValue = mValue + rhs;
            }
            else {
                mValue = UpperBound;
            }
        }
        
        inline constexpr void operator-=(T rhs) {
            if (mValue >= (LowerBound + rhs)) {
                mValue -= rhs;
            }
            else {
                mValue = LowerBound;
            }
        }
        inline constexpr void operator-=(T rhs) volatile {
            if (mValue >= (LowerBound + rhs)) {
                mValue = mValue - rhs;
            }
            else {
                mValue = LowerBound;
            }
        }
        inline constexpr void operator/=(T f) {
            value_type t{mValue / f};
            if (mValue >= LowerBound) {
                mValue = t;
            }
            else {
                mValue = LowerBound;
            }
        }
        
        inline constexpr uint_ranged& operator=(const uint_ranged& rhs) {
            mValue = rhs.mValue;
            return *this;
        };

        inline constexpr void operator=(const uint_ranged& rhs) volatile {
            mValue = rhs.mValue;
        }
        
        template<etl::Concepts::NamedFlag Check = std::integral_constant<bool, true>>
        inline constexpr void set(T rhs) volatile {
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            if constexpr(Check::value) {
                mValue = std::clamp(rhs, LowerBound, UpperBound);
            }
            else {
                mValue = rhs;
            }
        }

        [[deprecated("use set...()")]] inline constexpr uint_ranged& operator=(T rhs){
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            mValue = std::clamp(rhs, LowerBound, UpperBound);
            return *this;
        }
        [[deprecated("use set...()")]] inline constexpr void operator=(T rhs) volatile {
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            mValue = std::clamp(rhs, LowerBound, UpperBound);
        }
        inline constexpr operator T() const {
            return mValue;
        }
        inline constexpr operator T() volatile const {
            return mValue;
        }
        inline constexpr T toInt() const {
            return mValue;
        }
        inline constexpr T toInt() volatile const {
            return mValue;
        }
        //    private: // necessary to be structural
        T mValue{LowerBound};
    };
    
//    template<Unsigned T, T L, T U>
//    constexpr int_ranged<T, L, U> operator-(uint_ranged<T, L, U> lhs, const uint_ranged<T, L, U>& rhs) {
//        return {lhs.mValue - rhs.mValue};
//    }
    
//    template<Unsigned T, T L, T U>
//    constexpr uint_ranged<T, L, U> operator+(uint_ranged<T, L, U> lhs, const uint_ranged<T, L, U>& rhs) {
//        lhs += rhs;
//        return lhs;
//    }
    template<Unsigned T, T L, T U>
    constexpr uint_ranged<T, L, U> operator/(uint_ranged<T, L, U> lhs, const T& rhs) {
        lhs /= rhs;
        return lhs;
    }
    
    template<Unsigned T = uint8_t, T LowerBound = 0, T UpperBound = std::numeric_limits<T>::max() - 1>
    class uint_ranged_NaN final {
        static_assert(LowerBound <= UpperBound);
    public:
        inline static constexpr T Lower = LowerBound;
        inline static constexpr T Upper = UpperBound;
        inline static constexpr T Mid = (UpperBound + LowerBound) / 2;
        inline static constexpr T NaN   = std::numeric_limits<T>::max();
        static_assert(Upper != NaN);
        
        using value_type = T;
        
        inline constexpr uint_ranged_NaN() = default;
        
        inline constexpr uint_ranged_NaN(const T v) : mValue(v) {
//            assert(v >= LowerBound);
//            assert(v <= UpperBound);
            if (v < LowerBound) {
                mValue = LowerBound;
            }
            else if (v > UpperBound) {
                mValue = UpperBound;
            }
        }

        template<typename U>
        inline static constexpr uint_ranged_NaN absolute(const U& s) {
            if (s >= 0) {
                return uint_ranged_NaN(s);
            }
            else {
                return uint_ranged_NaN(-s);
            }
        }
        
        inline constexpr void setNaN() {
            mValue = NaN;
        }
        
        inline constexpr explicit operator bool() const {
            return mValue != NaN;
        }
        inline T& operator*() {
            return mValue;
        }
        inline const T& operator*() const {
            return mValue;
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
        inline void operator--() {
            if (mValue > LowerBound) {
                --mValue;
            }
        }
        inline void  operator++() {
            if (mValue < UpperBound) {
                ++mValue;
            }
        }
        //        inline constexpr bool operator==(T rhs) const {
        //            return mValue == rhs;
        //        }
        inline constexpr uint_ranged_NaN& operator=(T rhs) {
//            assert(rhs >= LowerBound);
//            assert(rhs <= UpperBound);
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
        inline constexpr void operator+=(const T value) {
            mValue = std::min(UpperBound, mValue + value);
        }
        inline constexpr void operator/=(const T d) {
            mValue = std::clamp(mValue / d, LowerBound, UpperBound);
        }
    private:
        T mValue{NaN};
    };
    
    template<Signed T = int8_t, T LowerBound = std::numeric_limits<T>::min(), T UpperBound = std::numeric_limits<T>::max() - 1>
    class int_ranged_NaN final {
        static_assert(LowerBound <= UpperBound);
    public:
        inline static constexpr T Lower = LowerBound;
        inline static constexpr T Upper = UpperBound;
        inline static constexpr T NaN   = std::numeric_limits<T>::max();
        using value_type = T;
        
        static_assert(Upper != NaN);
        
        //        using type = T;
        
        inline constexpr int_ranged_NaN() = default;
        
        inline constexpr int_ranged_NaN(const T v) : mValue(v) {
            assert(v >= LowerBound);
            assert(v <= UpperBound);
            if (v < LowerBound) {
                mValue = LowerBound;
            }
            else if (v > UpperBound) {
                mValue = UpperBound;
            }
        }
        
        //        inline constexpr uint_ranged_NaN(etl::fragmentType_t<T> higherPart, etl::fragmentType_t<T> lowerPart) :
        //            uint_ranged_NaN((static_cast<T>(higherPart) << etl::numberOfBits<etl::fragmentType_t<T>>()) + lowerPart)
        //        {}
        
        inline constexpr explicit operator bool() const {
            return mValue != NaN;
        }
        
        //        inline constexpr bool operator>(T rhs) const {
        //            return mValue > rhs;
        //        }
                inline constexpr bool operator>=(const T rhs) const {
                    return mValue >= rhs;
                }
        //        inline constexpr bool operator<(T rhs) const {
        //            return mValue < rhs;
        //        }
                inline constexpr bool operator<=(T rhs) const {
                    return mValue <= rhs;
                }
        
        inline void operator--() {
            if (mValue > LowerBound) {
                --mValue;
            }
        }
        inline void  operator++() {
            if (mValue < UpperBound) {
                ++mValue;
            }
        }
        inline constexpr bool operator==(const T rhs) const {
            return mValue == rhs;
        }
        inline constexpr void  operator=(const T rhs) {
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            mValue = std::clamp(rhs, LowerBound, UpperBound);
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
        //    private: // be structural
        T mValue{NaN};
    };
    
    template<Unsigned T, T LowerBound, T UpperBound >
    class uint_ranged_circular final {
        static_assert(LowerBound <= UpperBound);
        
        inline static constexpr T module{UpperBound + 1};
        inline static constexpr T module_mask = UpperBound;
        inline static constexpr bool use_mask_modulo = (LowerBound == 0) && (etl::isPowerof2(module));
        
    public:
        inline static constexpr T Lower = LowerBound;
        inline static constexpr T Upper = UpperBound;
        using value_type = T;
        
        inline constexpr uint_ranged_circular() = default;
        
        inline constexpr explicit uint_ranged_circular(T v) : mValue(v) {
            assert(v >= LowerBound);
            assert(v <= UpperBound);
            if (v < LowerBound) {
                mValue = LowerBound;
            }
            else if (v > UpperBound) {
                mValue = UpperBound;
            }
        }
        
        inline constexpr uint_ranged_circular(const uint_ranged_circular& rhs) = default;
        
        inline constexpr void operator=(const uint_ranged_circular& rhs) volatile {
            assert(rhs >= LowerBound);
            assert(rhs <= UpperBound);
            mValue = rhs.mValue;
        }
        
        inline constexpr bool isTop() const {
            return mValue == Upper;
        }
        
        inline constexpr bool isBottom() const {
            return mValue == Lower;
        }
        
        inline constexpr void setToBottom() {
            mValue = LowerBound;
        }
        
        inline constexpr uint_ranged_circular& operator--() {
            if constexpr(use_mask_modulo) {
                --mValue;
                mValue &= module_mask;                
            }
            else {
                if (mValue > LowerBound) {
                    --mValue;
                }
                else {
                    mValue = UpperBound;
                }
            }
            return *this;
        }
        inline constexpr uint_ranged_circular& operator++() {
            if constexpr(use_mask_modulo) {
                ++mValue;
                mValue &= module_mask;
            }
            else {
                if (mValue < UpperBound) {
                    ++mValue;
                }
                else {
                    mValue = LowerBound;
                }
            }
            return *this;
        }
        
        template<T Shift>
        inline constexpr uint_ranged_circular leftShift() volatile {
            static_assert(Shift >= 0);
            static_assert(Shift <= Upper);
            if (Shift <= mValue) {
                return uint_ranged_circular(mValue - Shift);
            }
            else {
                return uint_ranged_circular(mValue + (Upper + 1 - T{Shift}));
            }
        }
        
//        inline bool operator==(value_type rhs) const {
//            return mValue == rhs;
//        }
        
//        inline bool operator==(value_type rhs) const volatile {
//            return mValue == rhs;
//        }
        
//        inline bool operator!=(value_type rhs) const {
//            return !(mValue == rhs);
//        }
        
        inline constexpr auto operator<=>(const uint_ranged_circular&) const = default;
        
        inline constexpr bool operator==(const uint_ranged_circular&) const = default;
        
        inline constexpr void operator++() volatile {
            if constexpr(use_mask_modulo) {
                mValue = mValue + 1;
                mValue = mValue & module_mask;
            }
            else {
                if (mValue < UpperBound) {
                    ++mValue;
                }
                else {
                    mValue = LowerBound;
                }
                
            }
        }
        inline constexpr void operator--() volatile {
            if constexpr(use_mask_modulo) {
                --mValue;
                mValue &= module_mask;
            }
            else {
                if (mValue > LowerBound) {
                    --mValue;
                }
                else {
                    mValue = UpperBound;
                }
            }
        }
//        inline constexpr uint_ranged_circular& operator=(const T& rhs) {
//            assert(rhs >= LowerBound);
//            assert(rhs <= UpperBound);
//            mValue = rhs;
//            return *this;
//        }
//        inline constexpr void operator=(T rhs) volatile {
//            assert(rhs >= LowerBound);
//            assert(rhs <= UpperBound);
//            mValue = rhs;
//        }
        
        inline constexpr uint_ranged_circular flip() const {
            return uint_ranged_circular(UpperBound - mValue);
        }
        inline constexpr uint_ranged<T, LowerBound, UpperBound> toRanged() const {
            return uint_ranged<T, LowerBound, UpperBound>{mValue};
        }
        inline constexpr operator T() const {
            return mValue;
        }
        inline constexpr T toInt() const {
            return mValue;
        }
        inline constexpr T toInt() const volatile {
            return mValue;
        }
        //    private: // structural
        T mValue{LowerBound};
    };
    
}
