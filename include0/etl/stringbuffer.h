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
#include <algorithm>
#include <type_traits>

#include "types.h"

#include <mcu/pgm/pgmstring.h>

namespace etl {
    template<uint8_t Length, typename T = Char, Char Fill = Char{' '}>
    class StringBuffer final {
        template<uint8_t LLL, typename TTT, Char FFF> friend class StringBuffer;
    public:
        using value_type = T;
        using size_type = typeForValue_t<Length>;
        
        static inline constexpr size_type size() {
            return Length;
        }

//        inline constexpr StringBuffer(const AVR::Pgm::StringView& ps) {
//            insertAt(0, ps);
//        }

        template<typename C, C... CC>
        inline constexpr StringBuffer(const AVR::Pgm::String<C, CC...>& ps) {
            insertAt(0, ps); 
        }
        
        inline constexpr StringBuffer() = default;
        
        StringBuffer(const StringBuffer&) = delete;
        
        inline constexpr StringBuffer& operator=(const StringBuffer& rhs) {
            insertAt(0, rhs);
            return *this;
        }
        
        StringBuffer& operator=(StringBuffer&&) = delete;
    
        inline constexpr StringBuffer(StringBuffer&& other) {
            assert(other.size() <= size());
            etl::copy(data, other);
        }
    
        template<uint8_t M>
        inline constexpr void insertAt(uint8_t position, const StringBuffer<M, T>& sb) {
            assert(position < Length);
            insertAt(position, &sb.data[0]);
        }
    
        template<typename C, C... CC>
        inline constexpr void insertAt(uint8_t position, const AVR::Pgm::String<C, CC...>& ps) {
            static_assert(Length > ps.size(), "wrong length");
            assert(position < Length);
            for(uint8_t i = position, j = 0; (i < Length) && (j < ps.size()); ++i, ++j) {
                data[i] = ps[j];
            }
        }
        template<typename C, C... CC>
        inline constexpr void insertAtFill(uint8_t position, const AVR::Pgm::String<C, CC...>& ps) {
            static_assert(Length > ps.size(), "wrong length");
            uint8_t i = position;
            for(uint8_t j = 0; (i < Length) && (j < ps.size()); ++i, ++j) {
                data[i] = ps[j];
            }
            for(; i < Length; ++i) {
                data[i] = Fill;
            }
        }
        inline constexpr void insertAt(uint8_t position, const Char* s) {
            assert(position < Length);
            for(uint8_t i = position; (i < Length) && (*s != Char{'\0'}); ++i) {
                data[i] = *s++;
            }
        }
        inline constexpr auto insertAt(uint8_t position, const AVR::Pgm::StringView& s) {
            assert(position < Length);
            uint8_t i = position;
            for(uint8_t n = 0; (i < Length) && (s[n] != Char{'\0'}); ++i, ++n) {
                data[i] = s[n];
            }
            return i;
        }
        inline constexpr auto insertAtFill(uint8_t position, const AVR::Pgm::StringView& s) {
            assert(position < Length);
            uint8_t i = position;
            for(uint8_t n = 0; (i < Length) && (s[n] != Char{'\0'}); ++i, ++n) {
                data[i] = s[n];
            }
            auto p = i;
            for(; i < Length; ++i) {
                data[i] = Fill;
            }
            return p;
        }
//        template<uint8_t Position>
//        inline constexpr void insertAtFill(const PgmStringView& s) {
//            assert(position < Length);
//            uint8_t i = Position;
//            for(uint8_t n = 0; (i < Length) && (s[n] != Char{'\0'}); ++i, ++n) {
//                data[i] = s[n];
//            }
//            for(; i < Length; ++i) {
//                data[i] = Fill;
//            }
//        }
        inline constexpr void clear() {
            etl::fill(data, Fill);
        }
        inline constexpr const T& at(uint8_t index) const {
            assert(index < size());
            return data[index];
        }
        inline constexpr const T& operator[](uint8_t index) const {
            assert(index < size());
            return data[index];
        }
        inline constexpr T& operator[](uint8_t index) {
            assert(index < size());
            return data[index];
        }
        inline constexpr const volatile T& operator[](uint8_t index) const volatile {
            assert(index < size());
            return data[index];
        }
        inline constexpr volatile T& operator[](uint8_t index) volatile {
            assert(index < size());
            return data[index];
        }
        inline constexpr const T* begin() const {
            return &data[0];
        }
        inline constexpr const T* end() const {
            return &data[Length - 1] + 1;
        }
        inline constexpr T* begin(){
            return &data[0];
        }
        inline constexpr T* end() {
            return &data[Length - 1] + 1;
        }
    private:
        std::array<T, Length> data;
    };
    
    template<uint8_t Begin, uint8_t Length, typename ItemType = char>
    class StringBufferView {
    public:
        typedef ItemType type;
        static constexpr uint8_t size() {
            return Length;
        }
        
        template<typename C>
        inline StringBufferView(const C& c) : data((const ItemType*)c.begin() + Begin) {
            static_assert((Begin + Length) <= C::size(), "wrong begin or length");
        }
        inline const ItemType* begin() const {
            return data;
        }
        inline const ItemType* end() const {
            return data + Length;
        }
        inline constexpr const ItemType& operator[](const uint8_t index) const {
            assert(index < size());
            return data[index];
        }
    private:
        const ItemType* const data = nullptr;
    };
    
    template<uint8_t Begin, uint8_t Length, typename ItemType = char>
    class StringBufferPart {
    public:
        typedef ItemType value_type;
    
        template<typename C>
        inline StringBufferPart(C& c) : data((ItemType*)(c.begin()) + Begin) {
            static_assert((Begin + Length) <= C::size(), "wrong begin or length");
        }
    
        static constexpr uint8_t size() {
            return Length;
        }
        
        inline constexpr const ItemType& operator[](uint8_t index) const {
            assert(index < size());
            return data[index];
        }
        inline constexpr ItemType& operator[](uint8_t index) {
            assert(index < size());
            return data[index];
        }
    private:
        ItemType* const data = nullptr;
    };
    
    
    template<typename Buffer>
    class BufferDevice {
    public:
        BufferDevice(Buffer& b) : mBuffer(b) {}
        
        inline bool put(typename Buffer::value_type c) {
            if (counter < Buffer::size) {
                mBuffer[counter] = c;
                ++counter;
            }
            return true;  
        }
        inline Buffer& buffer() {
            return mBuffer;
        }
        inline void rewind() {
            counter = 0;
        }
    
    private:
        Buffer& mBuffer;
        uint8_t counter = 0;
    };
      
    template<typename C, uint8_t L1, uint8_t L2>
    inline StringBuffer<L1 + L2, C> operator+(const StringBuffer<L1, C>& lhs, const StringBuffer<L2, C>& rhs) {
        StringBuffer<(uint8_t)(L1 + L2), C> sum;
        sum.insertAt(0, lhs);
        sum.insertAt(lhs.size - 1, rhs);
        return sum;
    }
}

