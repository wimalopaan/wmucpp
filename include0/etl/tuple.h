#pragma once 

#include <tuple>
#include <cassert>
#include <etl/types.h>

namespace etl {
    namespace tuple::detail {
        template<uint8_t N>
        struct visit {
            template<typename T, typename F>
            constexpr static uint8_t at(T& tuple, uint8_t index, const F& f) {
                if (index == (N - 1)) {
                    return f(std::get<N - 1>(tuple));
                }
                else {
                    return visit<N - 1>::at(tuple, index, f);
                }
            }
        };
        template<>
        struct visit<0> {
            template<typename T, typename F>
            constexpr static uint8_t at(T&, uint8_t , const F&) {
                assert(false);
                return 0;
            }
        };
        template<typename T, typename F, size_t... I>
        constexpr void all(const T& tuple, const F& f, std::index_sequence<I...>) {
            (f(std::get<I>(tuple)), ...);
        }
        template<typename T, typename F, size_t... I>
        constexpr void all(T& tuple, const F& f, std::index_sequence<I...>) {
            (f(std::get<I>(tuple)), ...);
        }
    }
    template<typename... T, typename F>
    constexpr uint8_t visitAt(const std::tuple<T...>& tuple, uint8_t index, const F& f) {
        return tuple::detail::visit<sizeof...(T)>::at(tuple, index, f);
    }
    template<typename... T, typename F>
    constexpr uint8_t visitAt(std::tuple<T...>& tuple, uint8_t index, const F& f) {
        return tuple::detail::visit<sizeof...(T)>::at(tuple, index, f);
    }
    template<typename... T, typename F>
    constexpr void visit(const std::tuple<T...>& tuple, const F& f) {
        tuple::detail::all(tuple, f, std::make_index_sequence<sizeof...(T)>{});
    }
    template<typename... T, typename F>
    constexpr void visit(std::tuple<T...>& tuple, const F& f) {
        tuple::detail::all(tuple, f, std::make_index_sequence<sizeof...(T)>{});
    }
}
