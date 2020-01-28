#include <cstdint>
#include <array>
#include <tuple>

namespace etl {
    template<typename T>
    requires (std::is_integral_v<std::remove_cvref_t<T>>)
    struct BytesOf {
        using value_type = std::byte;
        template<auto N>
        [[nodiscard]] inline constexpr std::byte get() const requires (N < sizeof(T)) {
            return std::byte((mValue >> (N * 8)) & 0xff);
        }
        const std::remove_cvref_t<T> mValue;  
    };
    template<size_t I, typename T>
    [[nodiscard]] constexpr auto get(const BytesOf<T>& bo) {
        return bo.template get<I>();
    }
    template<typename T>
    [[nodiscard]] inline constexpr auto bytesOf(T&& value) {
        using value_type = std::remove_volatile_t<std::remove_reference_t<T>>;
        return BytesOf<value_type>{std::forward<T>(value)};
    }
}

namespace std {
    template<typename T>
    struct tuple_size<etl::BytesOf<T>> : std::integral_constant<size_t, sizeof(etl::BytesOf<T>::mValue)> {};

    template<size_t I, typename T> struct tuple_element;
    template<size_t  I, typename T>
    struct tuple_element<I, etl::BytesOf<T>> {
        using type = etl::BytesOf<T>::value_type;
    };
    template<size_t I, typename T>
    struct tuple_element<I, const etl::BytesOf<T>> {
        using type = etl::BytesOf<T>::value_type;
    };
}

volatile const auto r = 0x04030201;

int main() {
    using etl::bytesOf;
    const auto [b1] = bytesOf(uint8_t{0x01});
    const auto [lsb2, msb2] = bytesOf(uint16_t{0x0201});
    const auto [lsb3, b31, b32, msb3] = bytesOf(r);
    const auto [lsb4, b41, b42, msb4] = bytesOf(0x04030201);
    const auto [lsb5, b51, b52, b53, b54, b55, b56, msb5] = bytesOf(0x0403020104030201UL);
        
    const auto x1 = bytesOf(r).get<3>();
    const auto x2 = get<1>(bytesOf(r));
    constexpr auto x3 = get<1>(bytesOf(0x04030201));
            
    
            
    return (int)msb3;
}
            
