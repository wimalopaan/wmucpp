#include <cstdint>
#include <cstddef>
#include <cassert>
#include <string>
#include <tuple>
#include <variant>

#include "../include0/etl/meta.h"

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

template<class... Ts> struct overload_t : Ts... { using Ts::operator()...; };
template<class... Ts> overload_t(Ts...) -> overload_t<Ts...>;

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
constexpr auto parameter(TT&&... tt) {
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
    auto b = parameter<B>(vv...);
    auto c = parameter<C>(vv...);
    return a.value + b.value + c.value;
}

struct Day final {
    uint8_t value{};
};
struct Month final {
    uint8_t value{};
};
struct Year final {
    uint16_t value{};
};
struct Date final {
    template<typename... TT>
    constexpr Date(TT... vv) requires(is<list<TT...>, Day, Month, Year>) : 
        d{parameter<Day>(vv...).value}, m{parameter<Month>(vv...).value}, y{parameter<Year>(vv...).value}{}
    constexpr bool operator==(const Date&) const = default;
private:    
    uint8_t d{};
    uint8_t m{};
    uint16_t y{};
};

int main() {
    foo(A{1}, B{2});
    foo(B{1}, A{2});
    foo(B{1}, A{2}, C{3});
//    foo(1, 2);
    
    constexpr Date d1{Year{1903}, Day{1}, Month{3}};
    constexpr Date d2{Day{1}, Month{3}, Year{1903}};
    static_assert(d1 == d2);
    
    using namespace std::string_literals;

    std::tuple t{uint8_t{1}, uint16_t{2}, "abc"s};
    
    std::byte bits{};

#if 0    
    etl::visit(t, [&]<typename T>(T v){
                   if constexpr(std::is_same_v<T, uint8_t>) {
                       if (v == 1) {
                           bits |= std::byte{0x01};
                       }
                   }
                   if constexpr(std::is_same_v<T, uint16_t>) {
                       if (v == 2) {
                           bits |= std::byte{0x02};
                       }
                   }
                   if constexpr(std::is_same_v<T, std::string>) {
                       if (v == "abc"s) {
                           bits |= std::byte{0x04};
                       }
                   }
    });
#else  
    etl::visit(t, overload_t{
                   [&](uint8_t v){
                       if (v == 1) {
                           bits |= std::byte{0x01};
                       }
                   },
                   [&](uint16_t v){
                       if (v == 2) {
                           bits |= std::byte{0x02};
                       }
                   },
                   [&](std::string v){
                       if (v == "abc"s) {
                           bits |= std::byte{0x04};
                       }
                   },
               });
    
#endif
    return int(bits);
}
