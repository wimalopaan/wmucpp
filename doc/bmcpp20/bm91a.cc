#include <cstdint>
#include <array>
#include <optional>
#include <algorithm>
#include <initializer_list>
#include <util/algorithm.h>
#include <type_traits>
#include <util/meta.h>

namespace Meta {
    namespace detail {
        template<typename> struct sort_impl;
        template<template<auto...> typename L, auto... NN>
        struct sort_impl<L<NN...>> {
            inline static constexpr auto sorted = []{
                std::array<size_t, sizeof...(NN)> values{NN...};
                Util::sort(values);
                return values;
            }();
            template<typename> struct expand;
            template<auto... II>
            struct expand<std::index_sequence<II...>> {
                typedef L<sorted[II]...> type;
            };
            typedef typename expand<std::make_index_sequence<sizeof...(NN)>>::type type;
        };     
    }
    template<typename L>
    using sort = typename detail::sort_impl<L>::type;
    
    template<typename L> struct convert;
    template<template<auto...> typename L, auto... NN>
    struct convert<L<NN...>> {
        template<typename T>
        static constexpr std::array<T, sizeof...(NN)> toArray() {
            return std::array<T, sizeof...(NN)>{NN...};
        }
    };
}

volatile uint8_t result;
volatile uint8_t value = 10;

struct RightOpen;
struct LeftOpen;
struct Closed;
struct Open;

struct Adjacent;
struct Disjoint;


// besser als Klasse
template<typename Kind, typename Inter, typename T, typename Values, typename Function>
constexpr void lookup(T v, const Function& f) {
    constexpr auto values = Meta::convert<Values>::template toArray<T>();
    constexpr bool isSorted = [&]{
        for(size_t i = 0; i < values.size - 1; ++i) {
            if (values[i] >= values[i + 1]) {
                return false;
            }
        }
        return true;
    }();      
    static_assert(isSorted);
    
    if constexpr(std::is_same_v<Kind, RightOpen>) {
        [&]<auto... II>(std::index_sequence<II...>) {
                (void)((v < values[II] ? (f(II), false) : true) && ...);
        }(std::make_index_sequence<Values::size>{});
    }
    else if constexpr(std::is_same_v<Kind, LeftOpen>) {
        // ...
    }
}

int main() {
    lookup<RightOpen, Adjacent, uint8_t, Meta::NList<10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110>>(value, [](auto index){result = index;});
}

