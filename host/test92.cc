#include <iostream>
#include <type_traits>
//#include <tuple>

#include <cstdint>
//#include <utility>
//#include <util/dassert.h>

namespace std {
    
    template<typename... V> struct tuple {};
    template<typename T, typename... V>
    struct tuple<T, V...> : tuple<V...> { 
        constexpr tuple() : tuple<V...>{}, mData{} {}
        constexpr tuple(T t, V... vs) : tuple<V...>{vs...}, mData{t} {}
        template<uint8_t N>
        const auto& get() const {
            if constexpr(N == 0) {
                return mData;
            }
            else {
                return tuple<V...>::template get<N-1>();
            }
        }
        template<uint8_t N>
        auto& get() {
            if constexpr(N == 0) {
                return mData;
            }
            else {
                return tuple<V...>::template get<N-1>();
            }
        }
        T mData;
    };
    template<uint8_t N, typename T, typename... TT>
    constexpr const auto& get(const std::tuple<T, TT...>& tuple) {
        if constexpr(N == 0) {
            return tuple.mData;
        }    
        else {
            const std::tuple<TT...>& base = tuple;
            return get<N - 1>(base);
        }
    }
    template<uint8_t N, typename T, typename... TT>
    constexpr auto& get(std::tuple<T, TT...>& tuple) {
        if constexpr(N == 0) {
            return tuple.mData;
        }    
        else {
            std::tuple<TT...>& base = tuple;
            return get<N - 1>(base);
        }
    }
    
    template<typename... T>
    tuple(T...) -> tuple<T...>;
    
    template <typename>
    struct tuple_size;     
    template <typename... Types>
    struct tuple_size<tuple<Types...>> : public std::integral_constant<size_t, sizeof...(Types)>{};
    
    template<typename T>
    class tuple_size<const T> : public std::integral_constant<size_t, tuple_size<T>::value> {};
       
//    namespace detail {
//        template <typename Tuple1, size_t... Indices1, typename Tuple2, size_t... Indices2>
//        constexpr auto tuple_cat(const Tuple1& tup1, const Tuple2& tup2,
//                                  index_sequence<Indices1...>, index_sequence<Indices2...>) {
//            return tuple(get<Indices1>(tup1)..., get<Indices2>(tup2)...);
//        }
//    }
    
//    template <typename Tuple1, typename Tuple2>
//    constexpr auto tuple_cat(const Tuple1& tup1, const Tuple2& tup2) {
//        return detail::tuple_cat(tup1, tup2, make_index_sequence<tuple_size<Tuple1>::value>{}, make_index_sequence<tuple_size<Tuple2>::value>{});
//    }
} // !std


namespace std {
    
//    template<typename T>
//    struct remove_cvref {
//        using type = std::remove_cv_t<std::remove_reference_t<T>>;
//    };
    
//    template<typename T>
//    using remove_cvref_t = typename remove_cvref<T>::type;
    
}

template<typename T, template<typename...> typename U>
struct is_derived_from_template {

    template<typename... Ts>
    static constexpr std::true_type f(U<Ts...>);
//    static constexpr std::true_type f(const U<>&);
    static constexpr std::false_type f(...);
    
    using tt = std::remove_cvref_t<T>;
    static constexpr bool value{decltype(f(std::declval<tt>()))::value};
    
};

template<typename... Ts>
struct S : std::tuple<Ts...> {
    S() = default;
};

struct X {};

int main() {
    
    S<> s0;
    S<int, double> s1;
    S<std::tuple<>> s2;
    
    
    std::cout << is_derived_from_template<decltype(s0), std::tuple>::value << "\n";
    std::cout << is_derived_from_template<decltype(s1), std::tuple>::value << "\n";
    std::cout << is_derived_from_template<decltype(s2), std::tuple>::value << "\n";
    std::cout << is_derived_from_template<X, std::tuple>::value << "\n";
    
    return 0;
}
