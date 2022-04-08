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
        struct Iterator {
            constexpr Iterator() = default;
            explicit constexpr Iterator(const Ptr<char>& p) : pgmPtr(p.value) {}
            inline /*constexpr */etl::Char operator*() const {
                return etl::Char{pgm_read_byte(pgmPtr)};
            }
            inline constexpr void operator++() {
                ++pgmPtr;
            }
            inline constexpr bool operator!=(const Iterator& rhs) const {
                if (rhs.pgmPtr) {
                    return pgmPtr != rhs.pgmPtr;
                }
                else {
                    return **this != etl::Char{'\0'};
                }
            }
            const char* pgmPtr{};
        };
        constexpr Iterator begin() const {
            return Iterator{ptrToPgmData};            
        }
        constexpr Iterator end() const {
            return Iterator{};            
        }
        inline etl::Char operator[](uint8_t index) const {
            return etl::Char{pgm_read_byte(ptrToPgmData.value + index)};
        }
        template<typename C, C... CC> 
        constexpr StringView(const String<C, CC...>& ps) : ptrToPgmData(ps.data) {}
//    private: // structural
        explicit constexpr StringView(const Ptr<char>& pgm) : ptrToPgmData(pgm.value) {}
        Ptr<char> const ptrToPgmData{nullptr};
    };
    
    template<typename C, C... CC>
    struct String final {
        friend class StringView;
        
        using value_type = etl::Char;
        
        consteval String() = default;
        
        class Iterator final {
        public:
            explicit constexpr Iterator(uint8_t index = 0) : mIndex(index) {}
            
            inline /*constexpr*/ etl::Char operator*() const {
                return etl::Char{pgm_read_byte(&data[mIndex])};
            }
            inline constexpr void operator++() {
                ++mIndex;
            }
            inline constexpr bool operator!=(const Iterator& rhs) const {
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

    template<typename> struct isString : std::false_type {};
    
    template<typename C, C... CC>
    struct isString<String<C, CC...>> : std::true_type {}; 
    
    template<typename C, C... CC>
    static constexpr bool isString_v = isString<C, CC...>::value;
}

template<typename C, C... CC>
consteval /*constexpr */AVR::Pgm::String<C, CC...> operator"" _pgm(){
    return AVR::Pgm::String<C, CC...>{};
}

namespace etl {
    template<etl::Concepts::Device Device, bool ensure = false>
    constexpr void put(std::byte b);
}

namespace etl::detail {
    template<etl::Concepts::Stream Stream>
    constexpr inline void out_impl(const AVR::Pgm::StringView& a) {
        for(uint8_t i{0}; a[i] != etl::Char{'\0'}; ++i) {
            put<typename Stream::device_type>(std::byte(a[i]));
        }
    }
}
