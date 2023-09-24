#include <cstdint>
#include <limits>
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace detail {
    template<typename T> 
    struct bit_type : std::false_type {};
    template<typename T> 
    requires ((std::is_integral_v<T> && !std::is_same_v<std::remove_cvref_t<T>, bool>) || std::is_same_v<std::remove_cvref_t<T>, std::byte>)
    struct bit_type<T> : std::true_type {};

    template<typename T>
    static inline constexpr bool bit_type_v = bit_type<T>::value;
    
    template<uint8_t N, typename T> requires ((N < (sizeof(T) * 8)) && bit_type_v<T>)
    struct Bit {
        using value_type = std::remove_cvref_t<T>;
        friend auto bit<N, T>(T& v);

        template<typename B> requires (std::is_same_v<B, bool> && !std::is_const_v<T>)
        void operator=(const B b) {
            if (b) {
                d = d | value_type(1UL << N);
            }
            else {
                d = d & value_type(~(1UL << N));
            }
        }
        explicit operator bool() const {
                      return (d & value_type(1U << N)) != value_type{0};
        }
    private:
        explicit Bit(T& v) : d{v}{}
        T& d;
    };
}

template<uint8_t N, typename T> requires ((N < (sizeof(T) * 8)) && detail::bit_type_v<T>)
auto bit(T& v) {
    return detail::Bit<N, T>{v};    
}

//double v;
//std::byte v{};
/*volatile */char v{};

auto& x = v;

int main() {    
    bit<3>(v) = true;
    bit<4>(x) = true;

    if (bit<3>(v)) {
        
    }
    
    // bit<8>(v) = false;
    // bit<0>(v) = 1;
    
//    detail::Bit<0, std::byte>{v};
    
//    uint8_t a[10] = {};
    
//    for(uint8_t i = 0; i < 10; ++i) {
//        a[i] = 42 + i;
//    }
    
//    return a[8];
    
//    constexpr auto b = []{
//        int8_t x = std::numeric_limits<int8_t>::max();
////        ++x;
//        return (x > std::numeric_limits<int8_t>::max());
//    }();

//    auto p = "1" + 2;

//    int a1[100] {};
    
    auto a = []{
        std::array<uint8_t, 100> a;
        for(size_t i{}; auto& e: a) {
            e = sin(i++ * 2 * M_PI / a.size());
        }
        return a;
    }();
    
//    constexpr auto b2 = []{
//        uint8_t a[10];
//        uint8_t b[10];
        
//        auto p1 = &a[0];
////        auto p2 = &b[9];
//        auto p2 = &a[9];
        
//        return p1 < p2;
//    }();
}
