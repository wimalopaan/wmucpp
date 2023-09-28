#include <cstdint>
#include <cstddef>
#include <cstring>
#include <type_traits>

namespace etl {
    namespace detail {
        template<size_t N> struct typeForBits;
        template<> struct typeForBits<8> {
            using type = uint8_t;
        };
        template<> struct typeForBits<16> {
            using type = uint16_t;
        };
        template<> struct typeForBits<32> {
            using type = uint32_t;
        };
        template<> struct typeForBits<64> {
            using type = uint64_t;
        };
    }
    template<size_t N>
    using typeForBits_t = detail::typeForBits<N>::type;
    
    template<typename T>
    struct enable_bitmask_operators : std::false_type {};
    template<typename T>
    concept HasBitMaskOps = enable_bitmask_operators<T>::value;
    
    template<typename T>
    struct enable_arithmetic_operators : std::false_type {};
    template<typename T>
    concept HasArithOps = enable_arithmetic_operators<T>::value;
    
    template<size_t N>
    struct byte {
        using value_type = typeForBits_t<N>;
        constexpr byte(const value_type v = 0) : value{v} {}
        constexpr value_type uint() const {
            return value;
        }
    private:
        value_type value;
    };
    
    template<size_t N>
    struct enable_bitmask_operators<byte<N>> : std::true_type {};
    
    template<size_t N>
    struct enable_arithmetic_operators<byte<N>> : std::true_type {};
    
    struct Byte8 {
        using value_type = uint8_t;
        constexpr value_type uint() const {
            return value;
        }
    private:
        value_type value;
    };
    
    
    template<HasBitMaskOps T, typename I>
    constexpr T operator<<(const T lhs, const I Shift) {
        return T{static_cast<T::value_type>(lhs.uint() << Shift)};    
    }
    
    template<HasArithOps T, HasArithOps U>
    requires (std::is_same_v<typename T::value_type, typename U::value_type>)
    constexpr T operator+(const T lhs, const U rhs) {
        return T{static_cast<T::value_type>(lhs.uint() + rhs.uint())};    
    }
}

volatile uint32_t r;
volatile uint8_t r8;

volatile etl::byte<8> b8;
volatile etl::byte<16> b16;
volatile etl::byte<32> b32;
volatile etl::byte<64> b64;

namespace {
    volatile uint8_t dummy;
    void noop() {
        (void)dummy;
    } 
}
int main() {
    
    while(true) {
        noop();
    }
    r8 <<= 1;
    
    etl::byte<8> a8{r8};
    a8 = a8 << 1;
    //    r = a8.uint();
    
    //    etl::byte<8> c8{r8};
    
    //    a8 = a8 + c8;
}
