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
#include <limits>
#include <type_traits>
#include <cassert>
#include <initializer_list>

namespace etl {    
    template<typename T, auto Capacity>
    class Vector final {
    public:
        typedef typename std::conditional_t<(Capacity < 256), uint8_t, uint16_t> size_type;
        typedef typename std::conditional_t<(Capacity < 128), int8_t, int16_t> signed_size_type;
        typedef T value_type;
        inline static constexpr const size_type capacity = Capacity;
        
        static_assert(Capacity <= std::numeric_limits<uint16_t>::max());
        
        inline constexpr Vector(const std::initializer_list<T>& list) {
                for(const auto& i : list) {
                    push_back(i);
                }
        }
        
        constexpr Vector() = default;
        
        inline constexpr const T* begin() const {
            return &data[0];
        }
        inline constexpr const T* end() const {
            return &data[mSize];
        }
        inline constexpr const volatile T* begin() const volatile {
            return &data[0];
        }
        inline constexpr const volatile T* end() const volatile {
            return &data[mSize];
        }
        inline constexpr const T* cbegin() {
            return &data[0];
        }
        inline constexpr const T* cend() {
            return &data[mSize];
        }
        inline constexpr T* begin() {
            return &data[0];
        }
        inline constexpr T* end() {
            return &data[mSize];
        }
        inline constexpr volatile T* begin() volatile {
            return &data[0];
        }
        inline constexpr volatile T* end() volatile {
            return &data[mSize];
        }
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
            mSize = size_type{0};
        }
        
        inline constexpr void push_back(const T& item) {
            assert(mSize < Capacity);
            data[mSize++] = item;
        }
        inline constexpr void push_back(T&& item) {
            assert(mSize < Capacity);
            data[mSize++] = std::move(item);
        }
        inline constexpr T& back() {
            assert(mSize > 0);
            return data[mSize - 1];
        }
        inline constexpr const T& back() const {
            assert(mSize > 0);
            return data[mSize - 1];
        }
        inline constexpr void pop_back() {
            assert(mSize > 0);
            --mSize;
        }
    private:
        T data[Capacity];
        size_type mSize{0};
    };
}
