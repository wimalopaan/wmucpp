/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <algorithm>
#include <initializer_list>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <cassert>

namespace etl {    
    template<typename T, auto Capacity>
    class FixedVector final {
    public:
        using size_type = std::conditional_t<Capacity <= 255, uint8_t, uint16_t>;
        using value_type = T;
        
        template<typename... TT>
        requires((std::is_same_v<T, TT> && ...) && (sizeof...(TT) <= Capacity))
        constexpr FixedVector(const TT&... e) : data{e...}, mSize{sizeof...(TT)} {
        }
        
        inline constexpr T& operator[](size_type index) {
            assert(index < Capacity);
            return data[index];
        }
        inline constexpr const T& operator[](size_type index) const {
            assert(index < Capacity);
            return data[index];
        }
        inline constexpr const T* begin() const {
            return &data[0];
        }
        inline constexpr const T* end() const {
            return &data[mSize];
        }
        inline constexpr T* begin() {
            return &data[0];
        }
        inline constexpr T* end() {
            return &data[mSize];
        }
        inline constexpr void reserve(size_type s) {
            assert(s <= Capacity);
            mSize = std::min(s, Capacity);
        }
        inline constexpr size_type size() const {
            return mSize;
        }
        inline constexpr size_type capacity() const {
            return Capacity;
        }
        inline constexpr void clear() {
            mSize = 0;
        }
        inline constexpr void push_back(const T& item) {
            assert(mSize < Capacity);
            data[mSize++] = item;
        }
        inline constexpr const T& back() const {
            assert(mSize > 0);
            return data[mSize - 1];
        }
        inline constexpr T& back() {
            assert(mSize > 0);
            return data[mSize - 1];
        }
        inline constexpr const T& front() const {
            assert(mSize > 0);
            return data[0];
        }
        inline constexpr T& front() {
            assert(mSize > 0);
            return data[0];
        }
    private:
        T data[Capacity];
        size_type mSize{0};
    };
}
