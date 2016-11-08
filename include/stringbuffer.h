/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#include "stdint.h"
#include "pgmstring.h"
#include "algorithm.h"
#include "dassert.h"

#include <avr/pgmspace.h>

// todo: typw-ctor fuer PgmString

template<uint8_t L, typename T = char, char Fill = ' '>
class StringBuffer final {
    template<uint8_t N, typename, char> friend class StringBuffer;
public:
    typedef T type;
    static constexpr const uint8_t size = L;

    StringBuffer() = default;
    StringBuffer(const StringBuffer&) = delete;
    StringBuffer& operator=(const StringBuffer&) = delete;
    StringBuffer& operator=(StringBuffer&&) = delete;

    StringBuffer(StringBuffer&& other) {
        assert(other.size <= size);
        std::copy(std::begin(other), std::end(other), data);
    }

    template<uint8_t M>
    void insertAt(uint8_t position, const StringBuffer<M, T>& sb) {
        assert(position < L);
        insertAt(position, sb.data);
    }

    template<typename C, C... CC>
    void insertAt(uint8_t position, const PgmString<C, CC...>& ps) {
        static_assert(L > ps.size, "wrong length");
        assert(position < L);
        const char* ptr = ps.data;
        // todo: memcpy_P
//        memcpy_p(data + position, ps.data, min(L-position, ps.size));
        for(uint8_t i = position, j = 0; (i < L) && (j < ps.size); ++i, ++j) {
            data[i] = pgm_read_byte(ptr++);
        }
    }
    template<typename C, C... CC>
    void insertAtFill(uint8_t position, const PgmString<C, CC...>& ps) {
        static_assert(L > ps.size, "wrong length");
        const char* ptr = ps.data;
        uint8_t i = position;
        for(uint8_t j = 0; (i < L) && (j < ps.size); ++i, ++j) {
            data[i] = pgm_read_byte(ptr++);
        }
        for(; i < L; ++i) {
            data[i] = Fill;
        }
    }
    void insertAt(uint8_t position, const char* s) {
        assert(position < L);
        for(uint8_t i = position; (i < L) && (*s != '\0'); ++i) {
            data[i] = *s++;
        }
    }
    constexpr const T& at(uint8_t index) const {
        assert(index < size);
        return data[index];
    }
    constexpr const T& operator[](uint8_t index) const {
        assert(index < size);
        return data[index];
    }
    constexpr T& operator[](uint8_t index) {
        assert(index < size);
        return data[index];
    }
    constexpr const T* begin() const {
        return &data[0];
    }
    constexpr const T* end() const {
        return &data[L];
    }
    constexpr T* begin(){
        return &data[0];
    }
    constexpr T* end() {
        return &data[L];
    }
private:
    T data[L] = {};
};

template<typename Stream, uint8_t L, typename T, char fill>
Stream& operator<<(Stream& out, const StringBuffer<L, T, fill>& sb) {
    for(const auto& c : sb) {
        out << c;
    }
    return out;
}

template<typename C, uint8_t L1, uint8_t L2>
StringBuffer<L1 + L2, C> operator+(const StringBuffer<L1, C>& lhs, const StringBuffer<L2, C>& rhs) {
    StringBuffer<(uint8_t)(L1 + L2), C> sum;
    sum.insertAt(0, lhs);
    sum.insertAt(lhs.size - 1, rhs);
    return sum;
}
