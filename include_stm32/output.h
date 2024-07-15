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
        
        template<typename Device, typename T>
        requires ((std::is_signed_v<T> || std::is_unsigned_v<T>) && std::is_integral_v<T>)
        inline constexpr void out_impl(const T& v) {
            std::array<char, etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
            std::to_chars(std::begin(buffer), std::end(buffer), v);
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
        ((detail::out_impl<Stream>(vv)),..., detail::out_impl<Stream>('\n'));
    }    
    template<typename Stream, typename... TT>
    constexpr inline void out(const TT&... vv) {
        ((detail::out_impl<Stream>(vv)),...);
    }    
}




