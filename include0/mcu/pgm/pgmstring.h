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
#include <cstddef>
#include <cassert>

#include <etl/char.h>
#include <etl/concepts.h>

#if __has_include(<avr/pgmspace.h>)
# include <avr/pgmspace.h>
#endif

#include "pgm.h"

namespace AVR::Pgm {
    template<typename C, C... CC> struct String;
}

template<typename C, C... CC>
consteval /*constexpr */AVR::Pgm::String<C, CC...> operator"" _pgm();

namespace AVR::Pgm {
    
    class StringView final {
        template<typename C, C... CC> friend struct String;
    public:
        inline etl::Char operator[](uint8_t index) const {
//            assert(ptrToPgmData); // not possible: recursive
            return etl::Char{pgm_read_byte(ptrToPgmData + index)};
        }
        template<typename C, C... CC> 
        constexpr StringView(const String<C, CC...>& ps) : ptrToPgmData(ps.data) {}
    private:
        explicit constexpr StringView(const Ptr<char>& pgm) : ptrToPgmData(pgm.value) {}
        const char* const ptrToPgmData{nullptr};
    };
    
    template<typename C, C... CC>
    struct String final {
        friend class StringView;
        
        using value_type = etl::Char;
        
        constexpr String() = default;
        
        class Iterator final {
        public:
            explicit constexpr Iterator(uint8_t index = 0) : mIndex(index) {}
            
            inline etl::Char operator*() const {
                return etl::Char{pgm_read_byte(&data[mIndex])};
            }
            inline void operator++() {
                ++mIndex;
            }
            inline bool operator!=(const Iterator& rhs) const {
                return mIndex != rhs.mIndex;
            }
        private:
            uint8_t mIndex = 0;
        };
        constexpr Iterator begin() const {
            return Iterator();
        }
        constexpr Iterator end() const {
            return Iterator(size());
        }
        etl::Char operator[](uint8_t index) const {
//            assert(index < size()); // not possible here
            return etl::Char{pgm_read_byte(&data[index])};
        }
        inline static constexpr uint8_t size() {
            return sizeof...(CC);
        }
        inline constexpr operator StringView() const {
            return StringView{Ptr<char>{data}};
        }
    private:
        inline static constexpr const char data[] PROGMEM {CC..., '\0'};
    };
}

template<typename C, C... CC>
consteval /*constexpr */AVR::Pgm::String<C, CC...> operator"" _pgm(){
    return AVR::Pgm::String<C, CC...>{};
}

namespace etl::detail {
    template<etl::Concepts::Stream Stream>
    constexpr inline void out_impl(const AVR::Pgm::StringView& a) {
        for(uint8_t i{0}; a[i] != etl::Char{'\0'}; ++i) {
            put<typename Stream::device_type>(std::byte{a[i]});
        }
    }
}
