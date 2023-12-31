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
#include <ratio>
#include "std/concepts.h"

namespace std {
    
    template<typename Representation, typename Period = ratio<1, 1>>
    struct duration;
    
    template<>
    struct duration<uint16_t, std::milli> final
    {
        typedef std::milli period_type;
        uint16_t value = 0;
        duration<uint16_t, std::milli>& operator--() {
            --value;
            return *this;
        }
        duration<uint16_t, std::milli>& operator++() {
            ++value;
            return *this;
        }
    };
    
    template<>
    struct duration<uint16_t, std::micro> final
    {
        typedef std::micro period_type;
        uint16_t value = 0;
        duration<uint16_t, std::micro>& operator--() {
            --value;
            return *this;
        }
    };
    
    template<>
    struct duration<uint16_t, std::centimicro> final
    {
        typedef std::centimicro period_type;
        uint16_t value = 0;
    };
    
    template<>
    struct duration<uint16_t> final
    {
        typedef std::ratio<1,1> period_type;
        
        uint16_t value = 0;
        
        duration<uint16_t>& operator--() {
            --value;
            return *this;
        }
        operator duration<uint16_t, std::milli>() const{
            return {static_cast<uint16_t>(value * std::milli::denom)};
        }
    };
    
    template<>
    struct duration<uint32_t> final
    {
        typedef std::ratio<1,1> period_type;
        
        uint32_t value = 0;
    };
    
    
    template<typename T>
    bool operator==(const T& lhs, const T& rhs) {
        return lhs.value == rhs.value;
    }
    
    using centimicroseconds = duration<uint16_t, std::centimicro>;
    using microseconds = duration<uint16_t, std::micro>;
    using milliseconds = duration<uint16_t, std::milli>;
    using seconds = duration<uint16_t>;
    using wideSeconds = duration<uint32_t>;
    
    constexpr uint16_t operator/(const std::milliseconds& lhs, const std::milliseconds& rhs) {
        return lhs.value / rhs.value;
    }
    
    constexpr uint16_t operator/(const std::microseconds& lhs, const std::microseconds& rhs) {
        return lhs.value / rhs.value;
    }
    
    template<Integral F>
    //requires Integral<F>()
    constexpr std::microseconds operator/(const std::microseconds& lhs, F v) {
        return {(uint16_t)(lhs.value / v)};
    }
    
    template<Integral F>
    //requires Integral<F>()
    constexpr std::wideSeconds operator*(const std::wideSeconds& lhs, F v) {
        return {lhs.value * v};
    }
    
    template<typename R, typename P>
    constexpr duration<R, P> operator-(const duration<R, P>& t1, const duration<R, P>& t2) {
        return duration<R, P>{t1.value - t2.value};
    }
    
    template<typename R, typename P>
    constexpr duration<R, P> operator+(const duration<R, P>& t1, const duration<R, P>& t2) {
        return duration<R, P>{t1.value + t2.value};
    }
    
    template<typename R, typename P>
    constexpr duration<R, P>& operator+=(duration<R, P>& t1, const duration<R, P>& t2) {
        t1.value += t2.value;
        return t1;
    }
    template<typename R, typename P>
    constexpr duration<R, P>& operator-=(duration<R, P>& t1, const duration<R, P>& t2) {
        t1.value -= t2.value;
        return t1;
    }
    
    template<typename R, typename P>
    constexpr bool operator==(const duration<R, P>& lhs, const duration<R, P>& rhs) {
        return lhs.value == rhs.value;
    }
    
    template<typename R, typename P>
    constexpr bool operator!=(const duration<R, P>& lhs, const duration<R, P>& rhs) {
        return !(lhs == rhs);
    }
    
    template<typename R, typename P>
    constexpr bool operator>(const duration<R, P>& lhs, const duration<R, P>& rhs) {
        return lhs.value > rhs.value;
    }
    
    template<typename R, typename P>
    constexpr bool operator>=(const duration<R, P>& lhs, const duration<R, P>& rhs) {
        return (lhs.value > rhs.value) || (lhs.value == rhs.value);
    }
    
}
