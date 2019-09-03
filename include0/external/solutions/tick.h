#pragma once

#include <std/chrono>
#include <etl/types.h>

template<typename Timer, typename T = uint16_t>
struct Tick {
    inline static constexpr auto intervall = Timer::intervall;
    constexpr Tick() = default;
    
    template<typename U>
    constexpr explicit Tick(const U& v) : value{v / intervall} {}
    
//    constexpr explicit Tick(const T& v) : value{v}{}
    
    constexpr void operator++() {
        ++value;
    }
    inline constexpr bool operator<(const Tick& rhs) const {
        return (value < rhs.value);
    }
    inline constexpr bool operator>(const Tick& rhs) const {
        return (value > rhs.value);
    }
    inline constexpr void reset() {
        value = 0;
    }
private:
    etl::uint_ranged<T> value;
};

