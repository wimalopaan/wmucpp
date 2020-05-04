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

#include "type_traits.h"
#include "types.h"

namespace etl {
    using namespace std;
    
    // safe to use with ISRs
    template<typename T, uint16_t Size = 32>
    class FiFo final {
    public:
        using size_type = etl::typeForValue_t<Size>;

        using index_type = etl::uint_ranged_circular<size_type, 0, Size - 1>;
//        using index_type = size_type;
        
        inline static constexpr bool sizeIsAtomic = DefaultMcuType::template is_atomic<size_type>();
        
        inline bool push_back(volatile const T& item) volatile {
            asm("; replace udivmod1");
            size_type next = in + 1;
            if (next == Size) next = 0;
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
        inline bool push_back(const T& item) volatile {
            asm("; replace udivmod2");
            size_type next = in.toInt() + 1;
            if (next == Size) next = 0;
            {
                Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
                if (out.toInt() == next) {
                    return false;
                }
            }
            data[in.toInt()] = item;
            in = index_type{next};
            return true;
        }
        inline bool push_back(const T& item) {
            asm("; replace udivmod3");
            index_type next{in};
            ++next;            
//            index_type next = in + 1;
//            if (next == Size) next = 0;
            
            {
                Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
                if (out == next) {
                    return false;
                }
            }
            data[in] = item;
//            data[in] = item;
            in = next;
            return true;
        }
        inline bool pop_front(T& item) {
            {
                Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
                if (in == out) {
                    return false;
                }
            }
            item = data[out];
//            item = data[out];
            asm("; replace udivmod4");
            ++out;
//            if (++out == Size) out = 0;
            return true;
        }
        inline bool pop_front(T& item) volatile {
            {
                Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
                if (in == out) {
                    return false;
                }
            }
//            item = data[out];
            item = data[out];
            asm("; replace udivmod5");
            ++out;
//            if (++out == Size) out = 0;
            return true;
        }
        inline std::optional<T> pop_front() volatile {
            {
                Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
                if (in.toInt() == out.toInt()) {
                    return {};
                }
            }
//            T item = data[out];
            T item = data[out.toInt()];
            asm("; replace udivmod6");
//            if (++out == Size) out = 0;
            ++out;
            return item;
        }
        inline std::optional<T> pop_front() {
            {
                Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
                if (in == out) {
                    return {};
                }
            }
//            T item = data[out];
            T item = data[out];
            asm("; replace udivmod7");
            ++out;
//            if (++out == Size) out = 0;
            return item;
        }
        inline void clear() volatile {
            Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
            in = out = 0;
        }
        inline bool empty() const {
            Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
            return in == out;
        }
        inline bool empty() volatile const {
            Scoped<DisbaleInterrupt<RestoreState>, !sizeIsAtomic> di;
            return in.toInt() == out.toInt();
        }
        inline constexpr size_type size() const {
            return Size;
        }
    private:
        index_type in{0};
        index_type out{0};
        T data[Size] {};
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
