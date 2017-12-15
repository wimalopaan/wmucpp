#include <cstdint>
#include <cstddef>
#include <utility>

#include "mcu/avr8.h"
#include "mcu/register.h"
#include "hal/flag.h"

#include "util/meta.h"
#include "util/type_traits.h"

using flagRegister = AVR::RegisterFlags<typename DefaultMcuType::GPIOR, 0, std::byte>;

template<template<typename> typename T>
struct NumberOfFlags : std::integral_constant<size_t, 0> {};

template<typename T, typename Flag>
struct A final {
    A() = delete;
    static void f(){
        Flag::set();
    }
    inline static T mData{};
};

template<typename Flag, typename T>
struct B final {
    B() = delete;
    static void f() {
        if (Flag::isSet()) {
            Flag::reset();
        }
    }
    inline static T mData{};
};

template<typename Flag>
using AF = A<uint8_t, Flag>;
template<>
struct NumberOfFlags<AF> : std::integral_constant<size_t, 2> {};


template<typename Flag>
using BF = B<Flag , uint16_t>;
template<>
struct NumberOfFlags<BF> : std::integral_constant<size_t, 1> {};


template<typename FlagRegister, template<typename> typename ... X>
struct Controller {
    static_assert(sizeof...(X) <= 8, "too much ressources");
    using ressourceList = Meta::TList<X...>;
    static_assert(Meta::is_set_T<ressourceList>::value, "all ressources must be different");
    
    using bitList = Meta::List<std::integral_constant<uint8_t, NumberOfFlags<X>::value>...>;
    
    template<typename L, auto N> struct sum;
    template<typename... C, template<typename...>typename L, typename F, auto N>
    struct sum<L<F, C...>, N> {
        inline static constexpr auto Next = N + F::value;
        using x = std::pair<std::integral_constant<uint8_t, N>, std::integral_constant<uint8_t, Next - 1>>;
        using type = typename std::conditional<(sizeof...(C) == 0), Meta::List<x>,
                                      Meta::concat<Meta::List<x>, typename sum<L<C...>, Next>::type>>::type;
    };
    template<template<typename...>typename L, auto N>
    struct sum<L<>, N> {
        typedef L<> type;
    };
    
    using startBitList = typename sum<bitList, 0>::type;

    template<template<typename>typename T, typename Pair>
    struct makeNumberedFlags {
        typedef T<Hal::Flag<FlagRegister, Pair::first_type::value, Pair::second_type::value>> type;  
    };
    
    template<template<template<typename>typename, typename> typename F, typename L1, typename L2> struct build;
    template<template<template<typename>typename, typename> typename F, 
             template<template<typename>typename...> typename L1, template<typename>typename... I1, 
             template<typename...> typename L2, typename... I2>
    struct build<F, L1<I1...>, L2<I2...>> {
        static_assert(sizeof...(I1) == sizeof...(I2));
        typedef Meta::List<typename F<I1, I2>::type...> type;
    };
    
    using numberedRessouceList = typename build<makeNumberedFlags, ressourceList, startBitList>::type;

    template<template<typename> typename T>
    struct get {
        static constexpr auto index = Meta::index_T<ressourceList, T>::value;
        typedef Meta::nth_element<index, numberedRessouceList> type;
    };
};

using controller = Controller<flagRegister, AF, BF>;

//controller::rlist::_;

using a = controller::get<AF>::type;
using b = controller::get<BF>::type;

int main() {
    a::f();
    b::f();
    
}