#include <bits/byte.h>
#include <array>
#include <tuple>

#define V1

#ifdef V1 
namespace etl {
    template<typename T>
    requires (std::is_integral_v<std::remove_cvref_t<T>>)
    struct BytesOf {
        using value_type = std::byte;
        const std::remove_cvref_t<T> mValue;  
        template<auto N>
        requires (N < sizeof(T))
        inline constexpr std::byte get() const {
            return std::byte((mValue >> (N * 8)) & 0xff);
        }
    };
    template<auto I, typename T>
    constexpr auto get(const BytesOf<T>& bo) {
        return bo.template get<I>();
    }
    template<typename T>
    inline constexpr auto bytesOf(T&& value) {
        using value_type = std::remove_volatile_t<std::remove_reference_t<T>>;
        return BytesOf<value_type>{std::forward<T>(value)};
    }
}

namespace std {
    template<typename T>
    struct tuple_size<etl::BytesOf<T>> : std::integral_constant<size_t, sizeof(etl::BytesOf<T>::mValue)> {};

    template<auto I, typename T> struct tuple_element;
    template<auto I, typename T>
    struct tuple_element<I, etl::BytesOf<T>> {
        using type = etl::BytesOf<T>::value_type;
    };
    template<auto I, typename T>
    struct tuple_element<I, const etl::BytesOf<T>> {
        using type = etl::BytesOf<T>::value_type;
    };
}
#endif
#ifdef V2
namespace etl {
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
}
#endif
#ifdef V3
namespace etl {
    template<typename C, typename T>
    requires (std::is_same_v<typename C::value_type, std::byte>) && (std::tuple_size_v<C> == sizeof(T))
    static inline void bytesOf(C& b, T val) {
        static_assert(b.size() == sizeof(T));
        [&]<auto... II>(std::index_sequence<II...>) {
            ((b[II] = std::byte(val >> (8 * II))), ...);  
        }(std::make_index_sequence<b.size()>{});    
    }
}
#endif

volatile const auto r = 0x04030201;

int main() {
    using etl::bytesOf;
#if defined(V1) || defined(V2)
    const auto [b1] = bytesOf(uint8_t{0x01});
    const auto [lsb2, msb2] = bytesOf(0x0201);
    const auto [lsb3, b31, b32, msb3] = bytesOf(r);
    const auto [lsb4, b41, b42, msb4] = bytesOf(0x04030201);
    const auto [lsb5, b51, b52, b53, b54, b55, b56, msb5] = bytesOf(0x0403020104030201UL);
        
    const auto x1 = bytesOf(r).get<3>();
    const auto x2 = get<1>(bytesOf(r));
            
    return (uint8_t)msb3 + (uint8_t)lsb3;
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
            
