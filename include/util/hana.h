#pragma once

#include <type_traits>
#include <utility>

namespace Hana {
    namespace detail {
        template<auto Index, typename T> struct tuple_base {};
        
        template<typename Seq, typename... TT> struct tuple_impl;
        
        template<auto... II, typename... TT>
        struct tuple_impl<std::index_sequence<II...>, TT...> : tuple_base<II, TT>... {
        };
        
        template<auto N, typename Tuple>
        auto get() {
        }
    }
    template<typename... TT>
    struct tuple {
        using items = detail::tuple_impl<std::make_index_sequence<sizeof...(TT)>, TT...>;
    };

    
    
    namespace detail {
        template<typename Tuple>
        auto front_impl(Tuple) {
        }
    }
    
    template<typename Tuple>
    using front = decltype(detail::front_impl(Tuple{}));

    namespace tests {
        struct A;
        struct B;
        struct C;
        
        using t1 = tuple<A, B, C>;
    }
}
