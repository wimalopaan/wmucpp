#pragma once

#include <cstdint>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>

namespace etl {
    template<typename C>
    constexpr C::value_type maximum(const C& c) {
        return [&]<auto... II>(const std::index_sequence<II...>){
            typename C::value_type max{c[0]};
            ((c[II + 1] > max ? max = c[II + 1]: 0), ...);
            return max;
        }(std::make_index_sequence<std::tuple_size_v<C> - 1>{});
    }   

    template<typename C>
    inline consteval bool isSet(const C& c) {
        using size_type = C::size_type;
        for(size_type i{0}; i < c.size(); ++i) {
            for(size_type n = i + 1; n < c.size(); ++n) {
                if (c[i] == c[n]) {
                    return false;
                }
            }
        }
        return true;
    }
    
    
    
    template<uint8_t Number, typename T>
    constexpr inline std::byte nth_byte(const T& v);
    
    template<>
    constexpr inline std::byte nth_byte<0, uint8_t>(const uint8_t& v) {
        return std::byte(v);
    }
    template<>
    constexpr inline std::byte nth_byte<0, std::byte>(const std::byte& v) {
        return v;
    }
    template<>
    constexpr inline std::byte nth_byte<0, uint16_t>(const uint16_t& v) {
        return std::byte(v);
    }
    template<>
    constexpr inline std::byte nth_byte<1, uint16_t>(const uint16_t& v) {
        return std::byte(v >> 8);
    }
    
    template<>
    constexpr inline std::byte nth_byte<0, uint32_t>(const uint32_t& v) {
        return std::byte(v);
    }
    template<>
    constexpr inline std::byte nth_byte<1, uint32_t>(const uint32_t& v) {
        return std::byte(v >> 8);
    }
    template<>
    constexpr inline std::byte nth_byte<2, uint32_t>(const uint32_t& v) {
        return std::byte(v >> 16);
    }
    template<>
    constexpr inline std::byte nth_byte<3, uint32_t>(const uint32_t& v) {
        return std::byte(v >> 24);
    }
}

