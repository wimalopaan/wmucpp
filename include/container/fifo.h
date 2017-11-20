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

#include "std/optional"
#include "std/type_traits"

#include "util/disable.h"

namespace std {
    // safe to use with ISRs
    template<typename T, uint16_t Size = 32>
    class FiFo final {
    public:
        typedef typename std::conditional< Size <= 255, uint8_t, uint16_t>::type size_type;
        
        inline static constexpr bool sizeIsAtomic = DefaultMcuType::template is_atomic<size_type>();
        
        bool push_back(volatile const T& item) volatile {
            size_type next = (in + 1) % Size;
            {
                Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
                if (out == next) {
                    return false;
                }
            }
            data[in] = item;
            in = next;
            return true;
        }
        bool push_back(const T& item) volatile {
            size_type next = (in + 1) % Size;
            {
                Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
                if (out == next) {
                    return false;
                }
            }
            data[in] = item;
            in = next;
            return true;
        }
        bool pop_front(T& item) volatile {
            {
                Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
                if (in == out) {
                    return false;
                }
            }
            item = data[out];
            out = (out + 1) % Size;
            return true;
        }
        std::optional<T> pop_front() volatile {
            {
                Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
                if (in == out) {
                    return {};
                }
            }
            T item = data[out];
            out = (out + 1) % Size;
            return item;
        }
        void clear() volatile {
            Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
            in = out = 0;
        }
        bool empty() volatile const {
            Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
            return in == out;
        }
        static constexpr const size_type size = Size;
    private:
        size_type in = 0;
        size_type out = 0;
        T data[Size] = {};
    };
    
    template<typename T>
    class FiFo<T, 0> final { // needed to prevent warning for zero-sized array
    public:
        bool push_back(volatile const T& item) volatile {
            return false;
        }
        bool push_back(const T& item) volatile {
            return false;
        }
        bool pop_front(T& item) volatile {
            return false;
        }
        std::optional<T> pop_front() volatile {
            return {};
        }
        void clear() volatile {
        }
        bool empty() volatile const {
            return true;
        }
        static constexpr const uint8_t size = 0;
    private:
    };
    
}
