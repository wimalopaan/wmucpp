#pragma once

#include <cstdint>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <cassert>
#include <optional>

#include "traits.h"

__attribute__((__error__("assert error"))) inline void constant_assert();

namespace etl {
    template<auto Low, auto High>
    struct ranged_NaN;
    
    template<auto Low, auto High>
    struct ranged final {
        using nan_type = ranged_NaN<Low, High>;
        using value_type = typeForIntervall_t<Low, High>;
        
        static inline constexpr value_type Lower = Low;
        static inline constexpr value_type Upper = High;
        
        constexpr ranged() = default;
        explicit constexpr ranged(const value_type v) : mValue{std::clamp(v, Lower, Upper)} {
            assert(v >= Low);
            assert(v <= High);
        }
        ranged& operator=(const ranged& rhs) = default;
        
        void set(const value_type v) {
            assert(v >= Low);
            assert(v <= High);
            mValue = std::clamp(v, Lower, Upper);            
        }
        constexpr operator value_type() const {
            return {mValue};
        }
        constexpr void operator++() {
            if (mValue < High) {
                ++mValue;
            }
        }
        constexpr void toBottom() {
            mValue = Low;
        }
        constexpr bool isTop() const {
            return mValue == High;
        }
    private:
        value_type mValue{Low};
    };
    
    template<auto Low, auto High>
    requires((Low <= High) && (Low >= 0))
    struct ranged_circular final {
        using value_type = typeForIntervall_t<Low, High>;
        
        static inline constexpr value_type Lower{Low};
        static inline constexpr value_type Upper{High};
        
        inline static constexpr size_t module{Upper + 1};
        inline static constexpr value_type module_mask{Upper};
        inline static constexpr bool use_mask_modulo{(Lower == 0) && (etl::isPowerof2(module))};
        
        
        constexpr ranged_circular() = default;
        
        constexpr explicit ranged_circular(const value_type v) requires(use_mask_modulo) : mValue{v & module_mask} {
            assert(v >= Lower);
            assert(v <= Upper);
        }

        constexpr explicit ranged_circular(const value_type v) requires(!use_mask_modulo) : mValue{std::clamp(v, Lower, Upper)} {
            assert(v >= Lower);
            assert(v <= Upper);
        }
        
//        constexpr ranged_circular(const ranged_circular& rhs) = default;
        
//        constexpr void operator=(const uint_ranged_circular& rhs) volatile {
//            assert(rhs >= LowerBound);
//            assert(rhs <= UpperBound);
//            mValue = rhs.mValue;
//        }
        
//        inline constexpr bool isTop() const {
//            return mValue == Upper;
//        }
        
//        inline constexpr bool isBottom() const {
//            return mValue == Lower;
//        }

//        inline constexpr bool isHalf() const {
//            return mValue == (Upper + Lower) / 2;
//        }
        
//        inline constexpr void setToBottom() {
//            mValue = LowerBound;
//        }

//        inline constexpr void setToTop() {
//            mValue = UpperBound;
//        }
        
//        inline constexpr uint_ranged_circular& operator--() {
//            if constexpr(use_mask_modulo) {
//                --mValue;
//                mValue &= module_mask;                
//            }
//            else {
//                if (mValue > LowerBound) {
//                    --mValue;
//                }
//                else {
//                    mValue = UpperBound;
//                }
//            }
//            return *this;
//        }
        inline constexpr void operator++() {
            if constexpr(use_mask_modulo) {
                ++mValue;
                mValue &= module_mask;
            }
            else {
                if (mValue < Upper) {
                    ++mValue;
                }
                else {
                    mValue = Lower;
                }
            }
        }
        inline constexpr void operator++() volatile {
            if constexpr(use_mask_modulo) {
                ++mValue;
                mValue &= module_mask;
            }
            else {
                if (mValue < Upper) {
                    ++mValue;
                }
                else {
                    mValue = Lower;
                }
            }
        }
        
//        template<T Shift>
//        inline constexpr uint_ranged_circular leftShift() volatile {
//            static_assert(Shift >= 0);
//            static_assert(Shift <= Upper);
//            if (Shift <= mValue) {
//                return uint_ranged_circular(mValue - Shift);
//            }
//            else {
//                return uint_ranged_circular(mValue + (Upper + 1 - T{Shift}));
//            }
//        }

//        inline constexpr uint_ranged_circular leftShift(const T s) volatile {
//            if (s <= mValue) {
//                return uint_ranged_circular(mValue - s);
//            }
//            else {
//                return uint_ranged_circular(mValue + (Upper + 1 - s));
//            }
//        }
        
////        inline bool operator==(value_type rhs) const {
////            return mValue == rhs;
////        }
        
////        inline bool operator==(value_type rhs) const volatile {
////            return mValue == rhs;
////        }
        
////        inline bool operator!=(value_type rhs) const {
////            return !(mValue == rhs);
////        }
        
//        inline constexpr auto operator<=>(const uint_ranged_circular&) const = default;
        
//        inline constexpr bool operator==(const uint_ranged_circular&) const = default;
        
//        inline constexpr void operator++() volatile {
//            if constexpr(use_mask_modulo) {
//                mValue = mValue + 1;
//                mValue = mValue & module_mask;
//            }
//            else {
//                if (mValue < UpperBound) {
//                    ++mValue;
//                }
//                else {
//                    mValue = LowerBound;
//                }
                
//            }
//        }
//        inline constexpr void operator--() volatile {
//            if constexpr(use_mask_modulo) {
//                --mValue;
//                mValue &= module_mask;
//            }
//            else {
//                if (mValue > LowerBound) {
//                    --mValue;
//                }
//                else {
//                    mValue = UpperBound;
//                }
//            }
//        }
////        inline constexpr uint_ranged_circular& operator=(const T& rhs) {
////            assert(rhs >= LowerBound);
////            assert(rhs <= UpperBound);
////            mValue = rhs;
////            return *this;
////        }
////        inline constexpr void operator=(T rhs) volatile {
////            assert(rhs >= LowerBound);
////            assert(rhs <= UpperBound);
////            mValue = rhs;
////        }
        
//        inline constexpr uint_ranged_circular flip() const {
//            return uint_ranged_circular(UpperBound - mValue);
//        }
//        inline constexpr uint_ranged<T, LowerBound, UpperBound> toRanged() const {
//            return uint_ranged<T, LowerBound, UpperBound>{mValue, etl::RangeCheck<false>{}};
//        }
//        inline constexpr uint_ranged_NaN<T, LowerBound, UpperBound> toRangedNaN() const {
//            return uint_ranged_NaN<T, LowerBound, UpperBound>{mValue};
//        }
        
//        template<typename R>
//        requires((R::Lower <= Lower) && (R::Upper >= Upper) && (std::is_same_v<value_type, typename R::value_type>))
//        inline constexpr R to() {
//            return R{mValue};
//        }
        
        constexpr operator value_type() const {
            return mValue;
        }
        constexpr operator value_type() const volatile {
            return mValue;
        }
//        inline constexpr T toInt() const {
//            return mValue;
//        }
//        inline constexpr T toInt() const volatile {
//            return mValue;
//        }
        private: // structural
        value_type mValue{Lower};
    }; 
    
    
    
    template<auto Low, auto High>
    struct ranged_NaN final {
        using value_type = typeForIntervall_t<Low, High>;
        using ranged_type = ranged<Low, High>;
        
        static inline constexpr value_type NaN = []{
            if constexpr (Low != std::numeric_limits<value_type>::min()) {
                return std::numeric_limits<value_type>::min();
            }
            else if constexpr (High != std::numeric_limits<value_type>::max()) {
                return std::numeric_limits<value_type>::max();
            }
            else {
                static_assert(false);
            }
        }();
//        std::integral_constant<value_type, NaN>::_;
        
        static inline constexpr value_type Lower = Low;
        static inline constexpr value_type Upper = High;
        
        ranged_NaN() = default;
        explicit ranged_NaN(const value_type v) : mValue{std::clamp(v, value_type{Low}, value_type{High})} {
            assert(v >= Low);
            assert(v <= High);
            if (__builtin_constant_p(v)) {
                if (!((v >= Lower) && (v <= Upper))) {
                    constant_assert();
                }
            }
        }
        
        ranged_NaN& operator=(const ranged_type& rhs) {
            mValue = rhs;
            return *this;
        }
        
        void setNaN() {
            mValue = NaN;
        }
        operator value_type() const {
            assert(mValue != NaN);
            return {mValue};
        }
        value_type toInt() const {
            assert(mValue != NaN);
            return {mValue};
        }
        explicit operator bool() const {
            return mValue != NaN;
        }
    private:
        value_type mValue{NaN};
    };

    template<auto L, auto U, typename R>
    requires(std::is_arithmetic_v<R>)
    bool operator<(const ranged_NaN<L, U> lhs, const R rhs) {
        if (lhs) {
            return lhs.toInt() < rhs;
        }
        return false;
    }
    

    template<typename T, uint8_t X, uint8_t Y>
    struct uint2D_ranged {
        constexpr T x() const {
            return mX;
        }
        constexpr T y() const {
            return mY;
        }
        constexpr const uint2D_ranged& operator++() {
            ++mX;
            if (mX == X) {
                mX = 0;
                incY();
            }
            return *this;
        }
        constexpr void next(const auto f) {
            ++mX;
            if (mX == X) {
                mX = 0;
                incY(f);
            }
        }
        constexpr void resetX() {
            mX = 0;
        }
        constexpr void reset() {
            mX = mY = 0;
        }
        constexpr void incY() {
            ++mY;
            if (mY == Y) {
                mY = 0;
            }
        }
        constexpr void incY(auto const f) {
            if (mY == (Y - 1)) {
                f();
            }
            else {
                ++mY;
            }
        }
    private:
        T mX{};
        T mY{};
    };




    template<typename A>
    struct index_type;
    
    template<typename ItemType, auto N>
    struct index_type<std::array<ItemType, N>> {
        using type = ranged<0u, N - 1>;
    };
    
    template<typename A>
    using index_type_t = index_type<A>::type;
}

namespace IO {
    namespace detail {
        template<typename Device>
        inline constexpr void out_impl(const char* p);
        
        template<typename Device, typename T>
        requires ((std::is_signed_v<T> || std::is_unsigned_v<T>)) 
        inline constexpr void out_impl(const T& v);
        
        template<typename Device, auto L, auto U>
        inline constexpr void out_impl(const etl::ranged_NaN<L, U>& v) {
            if (v) {
                out_impl<Device>(v.toInt());
            }
            else {
                out_impl<Device>("NaN");
            }
        } 
    }
}
