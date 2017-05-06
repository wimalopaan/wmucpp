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

#pragma once

#if __has_include(<avr/pgmspace.h>)
# include <avr/pgmspace.h>
#endif

#ifndef __AVR__
# undef PROGMEM
# define PROGMEM
#endif

// this is a g++ / clang++ extension

template<typename C, C... CC>
struct PgmString;

template<typename C, C... CC>
constexpr PgmString<C, CC...> operator"" _pgm();

struct  PgmStringView {
    const char* ptrToPgmData = nullptr;
};

template<typename C, C... CC>
struct PgmString final {
    typedef char value_type;
    friend struct PgmStringView;
    friend PgmString<C, CC...> operator"" _pgm<>();
    template<typename Stream, typename C1, C1... CC1> friend Stream& operator<<(Stream& out, const PgmString<C1, CC1...>& s);
    template<uint8_t, typename, char> friend class StringBuffer;

    static constexpr uint8_t size = sizeof...(CC);
    static constexpr const char data[] PROGMEM = {CC..., '\0'};
    
    char operator[](uint8_t index) const {
            return pgm_read_byte(&data[index]);
    }
};

template<typename C, C... CC>
constexpr const char PgmString<C, CC...>::data[] PROGMEM;

template<typename C, C... CC>
constexpr PgmString<C, CC...> operator"" _pgm(){
    return PgmString<C, CC...>();
}


