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
#include <limits>

#include "traits.h"

namespace etl {
    using namespace std;
    
    template<typename T, uint16_t Size = 32>
    class FiFo final {
    public:
        static_assert(Size <= std::numeric_limits<uint16_t>::max());
        static_assert(etl::isPowerof2(Size));
        
        using size_type = uint16_t;
//        using size_type = etl::typeForValue_t<Size>;

        static inline constexpr size_type size_mask = Size - 1;
        
        inline static constexpr size_type size() {
            return Size;
        }
        // using index_type = size_type;
        using index_type = std::conditional_t<std::is_volatile_v<T>, volatile size_type, size_type>;
//        using index_type = etl::uint_ranged_circular<size_type, 0, size() - 1>;

        template<typename U>
        inline bool emplace_back(const U& item) {
            size_type next{in};
            next = (next + 1) & size_mask;
            if (out == next) {
                return false;
            }
            mData[in] = item;
            // std::copy(std::begin(item), std::end(item), std::begin(mData[in]));
            in = next;
            return true;
        }
        
        inline bool push_back(const T& item) {
            size_type next{in};
            next = (next + 1) & size_mask;
            
            if (out == next) {
                return false;
            }
            mData[in] = item;
            in = next;
            return true;
        }
        inline bool pop_front(T& item) {
            if (in == out) {
                return false;
            }
            item = mData[out];
            out = (out + 1) & size_mask;
            return true;
        }
        inline std::optional<T> pop_front() {
            if (in == out) {
                return {};
            }
            const T item = mData[out];
            out = (out + 1) & size_mask;
            return item;
        }
        inline T& front() {
            return mData[out];
        }
        inline const T& front() const {
            return mData[out];
        }
        inline void clear() {
            in = out = 0;
        }
        inline bool empty() const {
            return in == out;
        }
        inline uint16_t elements() const {
            return in - out;
        }
        const auto& data() const {
            return mData;
        }
        auto& data() {
            return mData;
        }
    private:
        index_type in{};
        index_type out{};
        T mData[Size] {};
    };
    
    template<typename T>
    class FiFo<T, 0> final { // needed to prevent warn for zero-sized array
    public:
        inline bool push_back(volatile const T& ) volatile {
            return false;
        }
        inline bool push_back(const T& ) volatile {
            return false;
        }
        inline bool pop_front(T& ) volatile {
            return false;
        }
        inline std::optional<T> pop_front() volatile {
            return {};
        }
        inline void clear() volatile {
        }
        inline bool empty() volatile const {
            return true;
        }
        inline constexpr uint8_t size() const {
            return 0;
        }
    private:
    };
    
}
