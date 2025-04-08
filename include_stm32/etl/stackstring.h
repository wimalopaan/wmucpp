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
#include <array>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <cassert>

namespace etl {    
    template<typename T, auto Capacity>
    class StackString final {
    public:
        using size_type = std::conditional_t<Capacity <= 255, uint8_t, uint16_t>;
        using value_type = T;
        inline static constexpr const size_type capacity = Capacity;

        constexpr StackString() = default;
        
        template<typename E, auto N>
        requires (N <= capacity)
        constexpr StackString(const E (&s)[N]) : mSize{N}, data{std::to_array(s)} {
        }

        constexpr explicit StackString(const T* const s) {
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
        inline constexpr void reserve(size_type s) {
            assert(s <= Capacity);
            mSize = std::min(s, Capacity);
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
        
    private:
        std::array<T, Capacity> data;
        size_type mSize{0};
    };
}
