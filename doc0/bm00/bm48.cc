#include <cstdint>i
#include <tuple>
#include <functional>
#include <array>

namespace Dry {
    constexpr auto tuple = [](auto... ts){
        return [=](const auto& f){return f(ts...);};
    };  
    constexpr auto and_f = [](auto f){
        return [=](auto... e){return (f(e) && ...);};
    };
    constexpr auto or_f = [](auto f){
        return [=](auto... e){return (f(e) || ...);};
    };
    constexpr auto bind_rh = [](auto f, auto rhs) {
        return [=](auto lhs){return f(lhs, rhs);};
    };
    constexpr auto greater = [](auto rhs){
        return bind_rh(std::greater{}, rhs);
    };
    constexpr auto equal = [](auto rhs){
        return bind_rh(std::equal_to{}, rhs);
    };
    constexpr auto not_equal = [](auto rhs){
        return bind_rh(std::not_equal_to{}, rhs);
    };
    
    template<typename Func, typename Tuple>
    struct op_t {
        [[no_unique_address]] Func func;
        Tuple tup;
        template<typename F>
        auto apply(F f) const {
            return tup(func(f));
        }
        template <typename U>
        friend constexpr bool operator<(U&& u, op_t&& a) {
            return std::forward<op_t>(a) > std::forward<U>(u);            
        }
        template <typename U>
        friend constexpr bool operator!=(U&& u, op_t&& a) {
            return std::forward<op_t>(a) != std::forward<U>(u);            
        }
        template<typename T>
        bool operator>(const T& rhs) const {
            return apply(greater(rhs));
        }
        template<typename T>
        bool operator!=(const T& rhs) const {
            return apply(not_equal(rhs));
        }
        template<typename T>
        bool operator==(const T& rhs) const {
            return apply(equal(rhs));
        }
    };
    
    constexpr auto any_of = [](auto... ts){return op_t{and_f, tuple(ts...)};};
    constexpr auto each_of = [](auto... ts){return op_t{or_f, tuple(ts...)};};
}

volatile uint8_t x{1};
uint8_t y{2};
uint8_t z{3};
volatile uint8_t v1{2};
std::array<uint8_t, 10> a;

//#define PLAIN

int main() {
    using namespace Dry;
    uint8_t r{};
#ifdef PLAIN
    if ((v1 < x) || (v1 < y) || (v1 < z)) {
        r += 1;
    }
    if ((x == v1) || (y == v1) || (z == v1)) {
        r += 2;
    }
    if ((v1 != x) || (v1 != y) || (v1 != z)) {
        r += 3;
    }
    if ((x != v1) && (z != v1)) {
        r += 4;
    }
#else
    if (v1 < any_of(x, y, z)) {
        r += 1;
    }
    if (any_of(x, y, z) > v1) {
        r += 2;
    }
    if (v1 != any_of(x, y, z)) {
        r += 3;
    }
    if (each_of(x, z) != v1) {
        r += 4;
    }
#endif
//    auto it = std::find_if(std::begin(a), std::end(a), Dry::equal(any_of(x, y, z)));
    return r;
}
