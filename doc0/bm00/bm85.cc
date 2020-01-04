#include <cstdint>
#include <utility>
#include <array>

#if __has_include(<etl/type_traits.h>)
# include <etl/type_traits.h>
# define HAVE_ETL
#endif

template<auto N> using NumberOfCounters = std::integral_constant<size_t, N>;
template<auto N> using NumberOfBits     = std::integral_constant<size_t, N>;

template<typename Counters, typename CounterSize>
struct VCounter {
    inline static constexpr auto nCounters = Counters::value;
    inline static constexpr auto nDigits   = CounterSize::value;
    static_assert(nCounters <= (sizeof(uint64_t) * 8));
    
#ifdef HAVE_ETL
    using value_type = etl::typeForBits_t<nCounters>;
#else
    template<auto Bits>
    struct typeForBits {
        using type = typename std::conditional<(Bits <= 8), uint8_t, 
                         typename std::conditional<(Bits <= 16), uint16_t, 
                             typename std::conditional<(Bits <= 32), uint32_t,
                                  typename std::conditional<(Bits <= 64), uint64_t, void>::type>::type>::type>::type;
    };
    using value_type = typename typeForBits<nCounters>::type;
#endif
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

using VC = VCounter<NumberOfCounters<6>, NumberOfBits<2>>;

int main() {
    VC c;
    
    c << VC::value_type{0x01}; // count-up bit 1
    c << VC::value_type{0x01};
    c << VC::value_type{0x01};
    c << VC::value_type{0x01};
    c << VC::value_type{0x01};
    c << VC::value_type{0x01};
    c << VC::value_type{0x01};
    c << VC::value_type{0x01};

//    c << VC::value_type{0x00}; // reset
    
    return c.maxReached(); // check
}
