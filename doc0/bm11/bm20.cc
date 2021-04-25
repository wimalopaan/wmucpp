#include <cstdint>
#include <cstddef>
#include <cassert>
#include <vol_access>

#include <etl/type_traits.h>
#include <etl/constant.h>

using namespace std::experimental;

template<size_t Bits>
struct mask_t {
private:
    struct Shift {
        using value_type = etl::typeForValue_t<Bits>;
        constexpr inline void operator++() {
            assert(*this);
            ++value;            
        }
        constexpr inline explicit operator bool() const {
            return value != Bits;
        }
        constexpr inline operator value_type() const {
            return value;
        }
    private:
        value_type value{};
    };
public:
    using value_type = etl::typeForBits_t<Bits>;
    using shift_type = Shift;
    constexpr inline explicit mask_t(const value_type v = value_type{}): value{v} {}

    constexpr inline bool any() const {
        return value != 0;
    }
        
    constexpr inline value_type toInt() const {
        return value;    
    }
    
    constexpr inline mask_t operator<<(const shift_type d) const {
        assert(d);
        return mask_t{value << d};        
    }
    template<size_t N>
    constexpr inline mask_t operator<<(const std::integral_constant<size_t, N>) const {
        static_assert(N < Bits, "too far");
        return mask_t{value << N};        
    }
    template<size_t N>
    constexpr inline void operator<<=(const std::integral_constant<size_t, N>) {
        static_assert(N < Bits);
        value <<= N;
    }
    private:
    value_type value{};
};

using m_t = mask_t<32>;

volatile uint32_t r;

using namespace etl::literals;

void bad0() {
    for(uint8_t i = 0; i < 32; i++) {
        uint32_t mask = uint32_t{1} << i;
        r = mask;
    }
}
void better_10() {
    for(m_t::shift_type i; i; ++i) {
        m_t mask = m_t{1} << i;
        volatile_store(&r, mask.toInt());
    }
    
}
void bad1() {
    uint32_t mask = 1;
    mask <<= 10;
    r = mask;
    mask <<= 1;
    r = mask;
}
void better_11() {
    m_t mask{1};
    mask <<= 10_c;
    volatile_store(&r, mask.toInt());
    mask <<= 1_c;

    volatile_store(&r, mask.toInt());
}

int main() {
    bad0();
    better_10();
    bad1();
    better_11();
}
