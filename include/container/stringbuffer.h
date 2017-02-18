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

#include <stdint.h>
#include "container/pgmstring.h"
#include "std/algorithm.h"
#include "std/traits.h"
#include "util/dassert.h"

#include <avr/pgmspace.h>

template<uint8_t Length, typename T = char, char Fill = ' '>
class StringBuffer final {
    template<uint8_t N, typename, char> friend class StringBuffer;
public:
    typedef T type;
    typedef typename std::conditional<Length <= 255, uint8_t, uint16_t>::type size_type;
    static constexpr const size_type size = Length;

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
        assert(position < Length);
        insertAt(position, sb.data);
    }

    template<typename C, C... CC>
    void insertAt(uint8_t position, const PgmString<C, CC...>& ps) {
        static_assert(Length > ps.size, "wrong length");
        assert(position < Length);
        const char* ptr = ps.data;
        // todo: memcpy_P
//        memcpy_p(data + position, ps.data, min(L-position, ps.size));
        for(uint8_t i = position, j = 0; (i < Length) && (j < ps.size); ++i, ++j) {
            data[i] = pgm_read_byte(ptr++);
        }
    }
    template<typename C, C... CC>
    void insertAtFill(uint8_t position, const PgmString<C, CC...>& ps) {
        static_assert(Length > ps.size, "wrong length");
        const char* ptr = ps.data;
        uint8_t i = position;
        for(uint8_t j = 0; (i < Length) && (j < ps.size); ++i, ++j) {
            data[i] = pgm_read_byte(ptr++);
        }
        for(; i < Length; ++i) {
            data[i] = Fill;
        }
    }
    void insertAt(uint8_t position, const char* s) {
        assert(position < Length);
        for(uint8_t i = position; (i < Length) && (*s != '\0'); ++i) {
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
        return &data[Length];
    }
    constexpr T* begin(){
        return &data[0];
    }
    constexpr T* end() {
        return &data[Length];
    }
private:
    T data[Length] = {};
};

template<uint8_t Begin, uint8_t Length, typename ItemType = char>
class StringBufferView {
public:
    typedef ItemType type;
    
    template<uint8_t SBLength, typename T, char Fill>
    StringBufferView(StringBuffer<SBLength, T, Fill>& stringBuffer) : data(stringBuffer.begin() + Begin) {
        static_assert((Begin + Length) <= SBLength, "wrong begin or length");
    }
    static constexpr uint8_t size = Length;
    
    constexpr const ItemType& operator[](uint8_t index) const {
        assert(index < size);
        return data[index];
    }
    constexpr ItemType& operator[](uint8_t index) {
        assert(index < size);
        return data[index];
    }
private:
    ItemType* data = nullptr;
};


template<typename Buffer>
class BufferDevice {
public:
    BufferDevice(Buffer& b) : mBuffer(b) {}
    
    bool put(typename Buffer::type c) {
        if (counter < Buffer::size) {
            mBuffer[counter] = c;
            ++counter;
        }
        return true;  
    }
    Buffer& buffer() {
        return mBuffer;
    }
    void rewind() {
        counter = 0;
    }

private:
    Buffer& mBuffer;
    uint8_t counter = 0;
};


template<typename Stream, uint8_t Length, typename T, char fill>
Stream& operator<<(Stream& out, const StringBuffer<Length, T, fill>& sb) {
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
