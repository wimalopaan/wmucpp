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

#if __has_include(<avr/pgmspace.h>)
# include <avr/pgmspace.h>
#endif

#include <cstdint>
#include <cstddef>
#include <cassert>

#include <etl/char.h>

template<typename C, C... CC>
struct PgmString;

template<typename C, C... CC>
constexpr PgmString<C, CC...> operator"" _pgm();

class PgmStringView {
    template<typename C, C... CC> friend struct PgmString;
public:
    inline etl::Char operator[](uint8_t index) const {
//        assert(ptrToPgmData != nullptr);
        return etl::Char{pgm_read_byte(ptrToPgmData + index)};
    }
    template<typename C, C... CC> 
    explicit constexpr PgmStringView(const PgmString<C, CC...>& ps) : ptrToPgmData(ps.data) {}
private:
    explicit constexpr PgmStringView(const char* pgm) : ptrToPgmData(pgm) {}
    const char* const ptrToPgmData = nullptr;
};

template<typename C, C... CC>
struct PgmString final {
    using value_type = etl::Char;
    
    constexpr PgmString() = default;
    
    class Iterator {
    public:
        explicit constexpr Iterator(uint8_t index = 0) : mIndex(index) {}

        inline etl::Char operator*() const {
            return etl::Char{pgm_read_byte(&data[mIndex])};
        }
        inline void operator++() {
            ++mIndex;
        }
        inline bool operator!=(const Iterator& rhs) const {
            return mIndex != rhs.mIndex;
        }
    private:
        uint8_t mIndex = 0;
    };
    constexpr Iterator begin() const {
        return Iterator();
    }
    constexpr Iterator end() const {
        return Iterator(size());
    }
    etl::Char operator[](uint8_t index) const {
        assert(index < size());
        return etl::Char{pgm_read_byte(&data[index])};
    }
    inline static constexpr uint8_t size() {
        return sizeof...(CC);
    }
    inline constexpr operator PgmStringView() const {
        return PgmStringView{data};
    }
private:
    inline static constexpr const char data[] PROGMEM = {CC..., '\0'};
};

template<typename C, C... CC>
constexpr PgmString<C, CC...> operator"" _pgm(){
    return PgmString<C, CC...>();
}
