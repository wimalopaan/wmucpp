#include <cstdint>
#include <cstddef>
#include <cassert>
#include <tuple>

#include <etl/meta.h>


template<typename... TT>
struct list {
    using l = Meta::List<TT...>;
    template<typename... UU>
    inline static constexpr bool is(){
        using ul = Meta::List<UU...>;
        constexpr bool length_are_equal = (sizeof...(UU) == sizeof...(TT));
        constexpr bool is_set = Meta::is_set_v<ul>;
        constexpr bool u_in_l = (Meta::contains_v<l, UU>&& ...);
        static_assert(is_set, "parameter types must differ");
        return(u_in_l && length_are_equal && is_set);
    }
};

template<typename l1, typename l2> struct is_f;
template<typename... LL1, typename... LL2>
struct is_f<list<LL1...>, list<LL2...>> {
    inline static constexpr bool value = list<LL1...>::template is<LL2...>();
};
template<typename l1, typename... ll2>
constexpr bool is = is_f<l1, list<ll2...>>::value;

template<typename T, typename... TT>
auto parameter(TT&&... tt) {
    return std::get<T>(std::tuple{std::forward<TT>(tt)...});
}

struct A {
    uint8_t value{};
};
struct B {
    uint8_t value{};
};
struct C {
    uint8_t value{};
};

template<typename... TT>
uint8_t foo(TT... vv) requires(is<list<TT...>, B, A>) {
    auto a = std::get<A>(std::tuple{vv...});
    auto b = std::get<B>(std::tuple{vv...});
    return a.value + b.value;
}
template<typename... TT>
uint8_t foo(TT... vv) requires(is<list<TT...>, A, B, C>) {
    auto a = parameter<A>(vv...);
    auto b = std::get<B>(std::tuple{vv...});
    auto c = std::get<C>(std::tuple{vv...});
    return a.value + b.value + c.value;
}

int main() {
    auto x = foo(A{1}, B{2});
    x = foo(B{1}, A{2});
    x = foo(B{1}, A{2}, C{3});
   // foo(1, 2);
    return x;
}
