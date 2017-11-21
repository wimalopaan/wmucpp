/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

//#define NDEBUG

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>

#include "std/time.h"

#include "util/types.h"

#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminal = SimAVRDebugConsole;

namespace std {
    constexpr std::basic_ostream<terminal> cout;
    constexpr std::lineTerminator<CRLF> endl;
}

namespace DateTime {

constexpr uint32_t dateToJulianDayNumber(const DateTime::TimeTm& date)
{
    // from http://www.tondering.dk/main/index.php/calendar-information
    uint32_t a = (14-date.month().value)/12;
    uint32_t y = date.year().value+4800-a;
    uint32_t m = date.month().value + 12*a - 3;
    return date.day().value + (153*m+2)/5 + y*365 + y/4 - y/100 + y/400 - 32045;
}

constexpr DateTime::TimeTm julianDayNumberToDate(uint32_t julianDayNumber)
{
    // from http://www.tondering.dk/main/index.php/calendar-information
    uint32_t a = julianDayNumber + 32044;
    uint32_t b = (4*a+3)/146097;
    uint32_t c = a - (b*146097)/4;
    uint32_t d = (4*c+3)/1461;
    uint32_t  e = c - (1461*d)/4;
    uint32_t m = (5*e+2)/153;
	uint8_t day = static_cast<uint8_t>(e - (153 * m + 2) / 5 + 1);
	uint8_t month = static_cast<uint8_t>(m + 3 - 12 * (m / 10));
	uint16_t year = static_cast<uint16_t>(b * 100 + d - 4800 + m / 10);
    return DateTime::TimeTm(DateTime::Day{day}, DateTime::Month{month}, DateTime::Year{year}, 
                            DateTime::Hour{}, DateTime::Minute{}, DateTime::Second{}, false);
}

class CompilationDate {
public:
    constexpr CompilationDate() : mJulianDays(dateToJulianDayNumber(parseDATE(__DATE__))){
    }
    constexpr uint32_t value() const {
        return mJulianDays;
    }
    constexpr TimeTm date() const {
        return parseDATE(__DATE__);
    }
private:
    static constexpr uint_NaN<uint8_t> monthFromDate(const char* date) {
        for(uint8_t month = 0; month < mMonthNames.size; ++month) {
            if (std::equal(std::begin(mMonthNames[month]), std::end(mMonthNames[month]) - 1, date)) {
                return uint_NaN<uint8_t>{month};
            }   
        }
        return uint_NaN<uint8_t>{}; 
    }
    static constexpr uint_NaN<uint8_t> dayFromDate(const char* date) {
        uint8_t value = 0;
        if (date[4] != ' ') {
            value = (date[4] - '0') * 10;
        }  
        value += date[5] - '0';
        if (value <= 31) {
            return uint_NaN<uint8_t>{value};
        }
        return uint_NaN<uint8_t>{}; 
    }
    static constexpr uint16_t yearFromDate(const char* date) {
        uint16_t value = (date[7] - '0') * 1000;
        value += (date[8] - '0') * 100;
        value += (date[9] - '0') * 10;
        value += (date[10] - '0');
        return value;
    }
    static constexpr TimeTm parseDATE(const char* date) {
        auto month = monthFromDate(date);
        auto day = dayFromDate(date);
        auto year = yearFromDate(date);
        if (month && day) {
            return TimeTm{Day{day}, Month{month}, Year{year - 1900}, Hour{0}, Minute{0}, Second{0}, false};
        }
        return TimeTm{};
    }
    inline static constexpr auto mMonthNames = [](){
        std::array<std::array<char, 4>, 12> names;
        names[0] = std::to_array("Jan");
        names[1] = std::to_array("Feb");
        names[2] = std::to_array("Mar");
        names[3] = std::to_array("Apr");
        names[4] = std::to_array("May");
        names[5] = std::to_array("Jun");
        names[6] = std::to_array("Jul");
        names[7] = std::to_array("Aug");
        names[8] = std::to_array("Seb");
        names[9] = std::to_array("Oct");
        names[10] = std::to_array("Nov");
        names[11] = std::to_array("Dec");
        return names;
    }();
    const uint32_t mJulianDays = 0;
};

}

constexpr DateTime::CompilationDate cdate;

//constexpr auto cdate2 = std::to_array(__DATE__);

int main() {
    uint32_t d = cdate.value();
    
//    volatile uint32_t dd = 2 * d;
    
    std::cout << d << std::endl;
    std::cout << cdate.date() << std::endl;
    
//    volatile char m1 = cdate2[0];
//    volatile char m2 = cdate2[1];
//    volatile char m3 = cdate2[2];

    
//    for(const char& c : cdate2) {
//        m1 = c;    
//    }
    
//    volatile char mx1 = __DATE__[0];
//    constexpr char mx2 = __DATE__[1];
    
//    for(const char& c : __DATE__) {
//        mx1 = c;
//    }
    
    while(true) {
    }
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif
