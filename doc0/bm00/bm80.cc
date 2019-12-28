#include <bits/byte.h>
#include <array>
#include <tuple>

#define V2

#ifdef V1 
template<typename T>
struct BytesOf {
    const T mValue;  
    template<auto N>
    constexpr std::byte get() const {
        return std::byte((mValue >> (N * 8)) & 0xff);
    }
    
};

template<typename T>
constexpr auto bytesOf(T&& value) {
    return BytesOf{value};
}

namespace std {
    template<typename T>
    struct tuple_size<BytesOf<T>> : std::integral_constant<size_t, sizeof(T)> {};
    
    template<auto I, typename T> struct tuple_element;
    
    template<auto  I, typename T>
    struct tuple_element<I, BytesOf<T>> {
        using type = std::byte;
    };
    template<auto  I, typename T>
    struct tuple_element<I, const BytesOf<T>> {
        using type = std::byte;
    };
}
#endif
#ifdef V2
template<auto E = std::endian::little, typename T = void>
inline constexpr auto bytesOf(T value) {
    std::array<std::byte, sizeof(T)> data;
    if constexpr(E == std::endian::little) {
        [&]<auto... II>(std::index_sequence<II...>) {
            ((data[II] = std::byte(value >> ((8 * II)))), ...);  
        }(std::make_index_sequence<sizeof(T)>{});
    }
    else if constexpr (E == std::endian::big){
        [&]<auto... II>(std::index_sequence<II...>) {
            ((data[sizeof(T) - 1 - II] = std::byte(value >> ((8 * II)))), ...);  
        }(std::make_index_sequence<sizeof(T)>{});
    }
    else {
        static_assert(std::false_v<T>);
    }
    return data;
}
#endif
#ifdef V3
template<typename C, typename T>
requires (std::is_same_v<typename C::value_type, std::byte>) && (std::tuple_size_v<C> == sizeof(T))
static inline void bytesOf(C& b, T val) {
    static_assert(b.size() == sizeof(T));
    [&]<auto... II>(std::index_sequence<II...>) {
        ((b[II] = std::byte(val >> (8 * II))), ...);  
    }(std::make_index_sequence<b.size()>{});    
}
#endif

volatile auto r = 0x04030201;

int main() {
#if defined(V1) || defined(V2)
    auto [b1] = bytesOf(uint8_t{0x01});
    auto [lsb2, msb2] = bytesOf(0x0201);
    auto [lsb3, b31, b32, msb3] = bytesOf<std::endian::big>(r);
    auto [lsb4, b41, b42, msb4] = bytesOf(0x04030201);
    auto [lsb5, b51, b52, b53, b54, b55, b56, msb5] = bytesOf(0x0403020104030201UL);
         
    return (int)b1;
#endif
#ifdef V3            
    std::array<std::byte, 4> ar1;
    std::array<std::byte, 2> ar2;
    std::array<std::byte, 8> ar3;
    bytesOf(ar1, r);
    bytesOf(ar2, 0x0403);
    bytesOf(ar3, 0x0403020104030201UL);
            
    return (int)ar2[1];
#endif
}
            
