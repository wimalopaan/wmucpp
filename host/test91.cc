#include <cstdint>
#include <cstddef>
#include <tuple>

namespace Meta {
    template<typename... T> struct List {};    
    template<typename T> struct Type {using value_type = T;};
    template<typename... ll1, typename... ll2>
    consteval size_t size(List<ll1...>) {
        return sizeof...(ll1);
    }    
    template<typename... ll1, typename U>
    consteval bool contains(List<ll1...>, Type<U>) {
        return (std::is_same_v<Type<U>, Type<ll1>> || ...);
    }

    template<typename F, typename... RR> consteval bool isSet(List<F, RR...>);
    
    namespace detail {
        template<typename F, typename... RR>
        consteval bool isSet(F f, List<RR...> l) {
            if constexpr (size(l) > 0) {
                return !contains(l, f) && Meta::isSet(l);
            }
            return true;
        }
    }
    template<typename F, typename... RR>
    consteval bool isSet(List<F, RR...>) {
        return detail::isSet(Type<F>{}, List<RR...>{});
    }
}

namespace Parameter {
    template<typename... TT> using List = Meta::List<TT...>;
    
    template<typename... ll1, typename... ll2>
    consteval bool is_equal(Meta::List<ll1...> l1, Meta::List<ll2...> l2) {
        return (Meta::size(l1) == Meta::size(l2)) && (Meta::contains(l1, Meta::Type<ll2>{}) && ...) && Meta::isSet(l2);
    }
    
    template<typename l1, typename... ll2> constexpr bool is = is_equal(l1{}, List<ll2...>{});
    
    template<typename T, typename... TT>
    constexpr auto get(TT&&... tt) {
        return std::get<T>(std::tuple{std::forward<TT>(tt)...});
    }
    template<typename T, typename... TT>
    requires requires(T t) {t.value;}
    constexpr auto get(TT&&... tt) {
        return std::get<T>(std::tuple{std::forward<TT>(tt)...}).value;
    }
}

namespace Test {
    using namespace Parameter;
    
    struct A { uint8_t value{}; };
    struct B { uint16_t value{}; };
    struct C { double value{}; };
    
    template<typename... TT>
    uint8_t simple(TT... vv) {
        auto a = std::get<A>(std::tuple{vv...});
        auto b = std::get<B>(std::tuple{vv...});
        return a.value + b.value;
    }
    
    template<typename... TT>
    uint8_t foo(TT... vv) requires(is<List<TT...>, B, A>) {
        auto a = std::get<A>(std::tuple{vv...});
        auto b = std::get<B>(std::tuple{vv...});
        return a.value + b.value;
    }
    template<typename... TT>
    uint8_t foo(TT... vv) requires(is<List<TT...>, A, B, C>) {
        auto a = get<A>(vv...);
        auto b = get<B>(vv...);
        auto c = get<C>(vv...);
        return a + b + c;
    }
    template<typename... TT>
    uint8_t bar(TT... vv) requires(is<List<TT...>, uint8_t, uint16_t>) {
        auto a = get<uint8_t>(vv...);
        auto b = get<uint16_t>(vv...);
        return a + b;
    }
}
namespace Chrono {
    using namespace Parameter;
    struct Day final { uint8_t value{}; };
    struct Month final { uint8_t value{}; };
    struct Year final { uint16_t value{}; };
    struct Date final {
        template<typename... TT>
        constexpr Date(TT... vv) requires(is<List<TT...>, Day, Month, Year>) : 
            d{get<Day>(vv...)}, m{get<Month>(vv...)}, y{get<Year>(vv...)}{}
        constexpr bool operator==(const Date&) const = default;
    private:    
        uint8_t d{};
        uint8_t m{};
        uint16_t y{};
    };
}

int main() {
    using namespace Test;

    simple(A{2}, B{1}); // ok
    simple(B{1}, A{2}); // ok 
    simple(B{1}, A{2}, C{3}); // not intentional
    
    foo(A{1}, B{2}); // every permutation possible
    foo(B{1}, A{2});
    foo(B{1}, A{2}, C{3}); // every permutation possible
    foo(B{1}, C{3}, A{3});
//    foo(1, 2); // not possible
    
    bar(uint8_t{1}, uint16_t{2});
    bar(uint16_t{2}, uint8_t{1});
//    bar(uint8_t{2}, 1); // not possible
    
    using namespace Chrono;    
    
    constexpr Date d1{Year{1903}, Day{1}, Month{3}}; // every permutation possible
    constexpr Date d2{Day{1}, Month{3}, Year{1903}};

    static_assert(d1 == d2);
}
