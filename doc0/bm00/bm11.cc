#include <cstdint>
#include <limits>
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>

template<uint8_t... NN> struct all_of;

template<uint8_t... NN> struct any_of;

namespace detail {
    template<typename T> 
    struct bit_type : std::false_type {};
    template<typename T> requires ((std::is_integral_v<T> && !std::is_same_v<std::remove_cvref_t<T>, bool>) || std::is_same_v<std::remove_cvref_t<T>, std::byte>)
    struct bit_type<T> : std::true_type {};

    template<typename T> static inline constexpr bool bit_type_v = bit_type<T>::value;
    
    template<typename Bits, size_t N> struct less;
    template<template<uint8_t...> typename L, uint8_t... NN, size_t N>
    struct less<L<NN...>, N> {
        static inline constexpr bool value = ((NN < N) && ...);
    };
            
    template<typename T, size_t N>
    static inline constexpr bool less_v = less<T, N>::value;
    
    template<typename Bits, typename T> struct Bit;
    
    template<template<uint8_t... NN> typename L, uint8_t... NN, typename T> requires (less_v<L<NN...>, (8 * sizeof(T))> && bit_type_v<T>)
    struct Bit<L<NN...>, T> {
        using value_type = std::remove_cvref_t<T>;
        friend auto bit<L<NN...>, T>(T& v);

        template<typename B> requires (std::is_same_v<B, bool> && !std::is_const_v<T> && std::is_same_v<L<NN...>, all_of<NN...>>)
        void operator=(const B b) {
            if (b) {
                d = d | (value_type(1UL << NN) | ...);
            }
            else {
                d = d & (value_type(~(1UL << NN)) & ...);
            }
        }
        explicit operator bool() const requires (std::is_same_v<L<NN...>, any_of<NN...>>) {
            return (d & (value_type(1U << NN) | ...)) != value_type{0};
        }
        explicit operator bool() const requires (std::is_same_v<L<NN...>, all_of<NN...>>) {
            return (d & (value_type(1U << NN) | ...)) != (value_type(1U << NN) | ...);
        }
    private:
        explicit Bit(T& v) : d{v}{}
        T& d;
    };
}

template<typename Bits, typename T> requires (detail::less_v<Bits, (8 * sizeof(T))> && detail::bit_type_v<T>)
auto bit(T& v) {
    return detail::Bit<Bits, T>{v};    
}

template<uint8_t N, typename T> requires ((N < 8 * sizeof(T)) && detail::bit_type_v<T>)
auto bit(T& v) {
    return bit<all_of<N>>(v);    
}

//double v;
//std::byte v{};
volatile char v{};
auto& x = v;

//template<typename T> 
//auto foo(int v) { 
//    return 1 * v;    
//}
//template<typename T, typename U> 
//auto foo(int v) {
//    return 3 * v;    
//}
//template<uint8_t N> 
//auto foo(int v) { 
//    return 2 * v;    
//}

//template<typename T> struct F;

////template<uint8_t N> struct F; // interpreted as redeclaration

int main() {    
//    return foo<void>(42) + foo<1>(42);
    
    bit<all_of<3, 4>>(v) = true;
    bit<4>(x) = true;

    if (bit<any_of<3, 5>>(v)) {
        return 1;
    }
    
    // bit<8>(v) = false;
    // bit<0>(v) = 1;
    
//    detail::Bit<0, std::byte>{v};
    
}
