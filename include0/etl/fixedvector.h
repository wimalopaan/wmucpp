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

namespace etl {    
    template<typename T, auto Capacity>
    class FixedVector final {
    public:
        typedef typename std::conditional_t<Capacity <= 255, uint8_t, uint16_t> size_type;
        typedef T value_type;
        inline static constexpr const size_type capacity = Capacity;
        
        // no iterator because of removal
        inline constexpr T& operator[](size_type index) {
            assert(index < Capacity);
            return data[index];
        }
        inline constexpr const T& operator[](size_type index) const {
            assert(index < Capacity);
            return data[index];
        }
        
        inline constexpr size_type size() const {
            return mSize;
        }
        
        inline constexpr void clear() {
            mSize = 0;
        }
        
        inline constexpr void push_back(const T& item) {
            assert(mSize < Capacity);
            data[mSize++] = item;
        }
        
        // requires bool() Operator of elements
        inline constexpr std::optional<size_type> insert(const T& item) {
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
        inline constexpr void removeAt(size_type index) {
            assert(index < Capacity);
            data[index] = T{};
            --mSize;
        }
    private:
//        std::array<T, Capacity> data; // Wird größer, Warum???
        T data[Capacity] = {};
        size_type mSize = 0;
    };
    
}
