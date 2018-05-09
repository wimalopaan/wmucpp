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
#include <optional>
#include <type_traits>
#include <cassert>

namespace std {    
    template<typename T, uint16_t Capacity>
    class FixedVector final {
    public:
        typedef typename std::conditional<Capacity <= 255, uint8_t, uint16_t>::type size_type;
        
        // no iterator because of removal
        T& operator[](size_type index) {
            assert(index < Capacity);
            return data[index];
        }
        const T& operator[](size_type index) const {
            assert(index < Capacity);
            return data[index];
        }
        
        static constexpr const size_type capacity = Capacity;
        typedef T value_type;
        
        constexpr size_type size() const {
            return mSize;
        }
        
        void clear() {
            mSize = 0;
        }
        
        void push_back(const T& item) {
            assert(mSize < Capacity);
            data[mSize++] = item;
        }
        
        // requires bool() Operator of elements
        std::optional<size_type> insert(const T& item) {
            for(size_type i = 0; i < capacity; ++i) {
                if (!data[i]) {
                    data[i] = item;
                    ++mSize;
                    return i;
                }
            }
            return {};
        }
        
        // preserve index
        void removeAt(size_type index) {
            assert(index < Capacity);
            data[index] = T();
            --mSize;
        }
        constexpr const T* begin() const {
            return &data[0];
        }
        constexpr const T* end() const {
            return &data[Capacity];
        }
        constexpr const volatile T* begin() const volatile {
            return &data[0];
        }
        constexpr const volatile T* end() const volatile {
            return &data[Capacity];
        }
        constexpr T* begin() {
            return &data[0];
        }
        constexpr T* end() {
            return &data[Capacity];
        }
        constexpr volatile T* begin() volatile {
            return &data[0];
        }
        constexpr volatile T* end() volatile {
            return &data[Capacity];
        }
    private:
        T data[Capacity] = {};
        size_type mSize = 0;
    };
    
}
