#pragma once

#include <std/chrono>
#include <etl/types.h>

#include <compare>

namespace External {
    
    template<typename Timer, typename T = uint16_t>
    struct Tick {
        inline static constexpr auto intervall = Timer::intervall;
        
        inline constexpr Tick() = default;
        
        template<typename U>
        inline constexpr explicit Tick(const U& v) : value{v / intervall} {
        }
        
        template<typename U>
        inline constexpr void operator=(const U& v) {
            value = (v / intervall);
        }        
        
        inline constexpr void operator++() {
            ++value;
        }
        
        template<typename F>
        inline constexpr void on(const Tick& t, F f) {
            if (value == t.value) {
                f();
                reset();
            }
        }
        
        inline constexpr auto operator<=>(const Tick& rhs) const = default;
        
        inline constexpr void reset() {
            value = 0;
        }
//    private:
        etl::uint_ranged<T> value;
    };
}
