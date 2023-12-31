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
#include <time.h>
#include "mcu/concepts.h"

namespace DateTime {
    class TimeTm;
}

template<typename Stream> 
Stream& operator<<(Stream& out, const DateTime::TimeTm& t);

namespace std::detail {
    template<MCU::Stream Stream> void out(const DateTime::TimeTm& t);
}

namespace DateTime {
    
    struct Second {
        uint8_t value = 0;
    };
    struct Minute {
        uint8_t value = 0;
    };
    struct Hour {
        uint8_t value = 0;
    };
    struct Day {
        uint8_t value = 0;
    };
    struct Month {
        uint8_t value = 0;
    };
    struct Year {
        uint16_t value = 0;
    };
    
    class Time {
        
    };
    
    class Date {
        
    };
    
    class TimeTm {
        template<typename Stream> friend Stream& ::operator<<(Stream& out, const TimeTm& t);
        template<MCU::Stream Stream> friend void std::detail::out(const TimeTm& t);
        
    public:
        typedef struct tm tm_t;
        
        constexpr const tm_t& tm() const {
            return mTime;
        }
        
        constexpr TimeTm(const tm_t& t) : mTime(t) {}
        constexpr TimeTm() = default;
        
        TimeTm(time_t tt) {
            localtime_r(&tt, &mTime);
        }
        
        constexpr Second seconds() const {
            return {static_cast<uint8_t>(mTime.tm_sec)};
        }    
        constexpr Minute minutes() const {
            return {static_cast<uint8_t>(mTime.tm_min)};
        }    
        constexpr Hour hours() const {
            return {static_cast<uint8_t>(mTime.tm_hour)};
        }    
        constexpr Day day() const {
            return {static_cast<uint8_t>(mTime.tm_mday)};
        }    
        constexpr Year year () const {
            return {static_cast<uint16_t>(mTime.tm_year + 1900)};
        }    
        constexpr Month month() const {
            return {static_cast<uint8_t>(mTime.tm_mon + 1)};
        }    
        constexpr TimeTm(Day day, Month month, Year year, Hour hour, Minute minute, Second second, bool dst) 
            : mTime{(int8_t)second.value, (int8_t)minute.value, (int8_t)hour.value, (int8_t)day.value, 0, 
                    (int8_t)month.value, (int16_t)year.value, 0, dst ? 1 : 0}
        {
    }
                                                                       
                                                                       private:
                                                                       tm_t mTime{};
    };
    
}
