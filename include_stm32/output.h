/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "traits.h"
#include "units.h"

#include <limits>
#include <array>
#include <charconv>

namespace IO {
    namespace detail {
        template<typename Device>
        inline constexpr void out_impl(const char v) {
            Device::put(v);
        } 
        template<typename Device>
        inline constexpr void out_impl(const char* p) {
            while(const char c = *p++) {
                Device::put(c);
            }  
        } 
        
        template<typename Device, typename C>
        requires (sizeof(typename C::value_type) == 1)
        constexpr inline void out_impl(const C& a) {
            for(const typename C::value_type& c : a) {
                if (c == typename C::value_type{'\0'}) {
                    break;
                }
                Device::put(c);
            };   
        }
        template<typename Device>
        inline constexpr void out_impl(const std::byte& v) {
            std::array<char, 16> buffer{};
            std::to_chars(std::begin(buffer), std::end(buffer), uint8_t(v), 16);
            out_impl<Device>(buffer);
        }
        template<typename Device, typename T>
        requires ((std::is_signed_v<T> || std::is_unsigned_v<T>) && std::is_integral_v<T>)
        inline constexpr void out_impl(const T& v) {
            std::array<char, etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
            // std::array<char, 16> buffer{};
            std::to_chars(std::begin(buffer), std::end(buffer), v);
            out_impl<Device>(buffer);
        }
        template<typename Device, typename T>
        requires (std::is_pointer_v<T>)
        inline constexpr void out_impl(const T& v) {
            std::array<char, 16> buffer{};
            std::to_chars(std::begin(buffer), std::end(buffer), (uint32_t)v, 16);
            out_impl<Device>(buffer);
        }

        // template<typename Device>
        // inline constexpr void out_impl(const float v) {
        //     std::array<char, 8> buffer{};
        //     std::to_chars(std::begin(buffer), std::end(buffer), v);
        //     out_impl<Device>(buffer);
        // }
    }
    template<typename Stream, typename... TT>
    constexpr inline void outl(const TT&... vv) {
        if constexpr (!std::is_same_v<Stream, void>) {
            ((detail::out_impl<Stream>(vv)),..., detail::out_impl<Stream>('\n'));
        }
    }    
    template<typename Stream, typename... TT>
    constexpr inline void out(const TT&... vv) {
        if constexpr(!std::is_same_v<Stream, void>) {
            ((detail::out_impl<Stream>(vv)),...);
        }
    }    
}




