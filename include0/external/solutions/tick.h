#pragma once

#include <std/chrono>
#include <etl/types.h>

#include <compare>

namespace External {
    
    template<typename Timer, typename T = uint16_t>
    struct Tick {
        inline static constexpr auto intervall = Timer::intervall;
        constexpr Tick() = default;
        
        template<typename U>
        constexpr explicit Tick(const U& v) : value{v / intervall} {}
        
        template<typename U>
        void operator=(const U& v) {
            value = (v / intervall);
        }        
        
        constexpr void operator++() {
            ++value;
        }
        inline constexpr bool operator<(const Tick& rhs) const {
            return (value < rhs.value);
        }
        inline constexpr bool operator>(const Tick& rhs) const {
            return (value > rhs.value);
        }
        
        inline constexpr auto operator<=>(const Tick& rhs) const = default;
        
        inline constexpr void reset() {
            value = 0;
        }
//    private:
        etl::uint_ranged<T> value;
    };
}
