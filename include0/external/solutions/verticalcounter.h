#pragma once

#include <cstdint>
#include <utility>
#include <array>

#include <etl/type_traits.h>

namespace External {
    
    template<auto N> using NumberOfCounters = std::integral_constant<size_t, N>;
    template<auto N> using NumberOfBits     = std::integral_constant<size_t, N>;
    
    template<typename Counters, typename CounterSize>
    struct VCounter {
        inline static constexpr auto nCounters = Counters::value;
        inline static constexpr auto nDigits   = CounterSize::value;
        static_assert(nCounters <= (sizeof(uint64_t) * 8));
        
        using value_type = etl::typeForBits_t<nCounters>;
        
        inline constexpr VCounter& operator<<(const value_type which) {
            digits[0] = ~(digits[0] & which);
            update(std::make_index_sequence<nDigits-1>{}, which);
            return *this;        
        }
        inline constexpr value_type maxReached() {
            return [&]<auto... II>(std::index_sequence<II...>){
                return (digits[II] & ...);
            }(std::make_index_sequence<nDigits>{});
    }
    private:
    using digits_t = std::array<value_type, nDigits>;
    template<auto... II>
    inline constexpr void update(std::index_sequence<II...>, const value_type which) {
        value_type r{digits[0]};
        (((digits[II + 1] = (r ^ (digits[II + 1] & which))), r &= digits[II + 1]), ...);
    }
    digits_t digits = [&]{
        digits_t d;
        for(auto& i : d) {
            i = static_cast<value_type>(-1);
        }
        return d;
    }();
};
}
