#include <cstdint>
#include <tuple>
#include "container/tree.h"

template<auto N>
struct A {
    inline static constexpr auto size = N;
    const uint8_t d[N] {};
};
template<auto N, auto... II>
struct B {};

constexpr auto a1 = A<2>{1, 2};
constexpr auto a2 = A<3>{2, 3, 4};
constexpr auto t1 = std::tuple(a1, a2);

auto l1 = [&]{
    constexpr auto a1 = A<2>{1, 2};
    constexpr auto a2 = A<3>{2, 3, 4};
    constexpr auto t1 = std::tuple(a1);
    return t1;
};

constexpr auto b1 = B<a1.size, a1.d[0], a1.d[1]>{};
constexpr auto b2 = B<a2.size, a2.d[0], a2.d[1], a2.d[2]>{};

template<typename T>
constexpr auto transform_1(const T& tuple) {
    constexpr auto x1 = std::get<0>(tuple);
    constexpr auto b1 = B<x1.size, x1.d[0], x1.d[1]>{}; 
    return std::tuple(b1);
}
template<typename T>
constexpr auto transform_2(const T& l) {
    constexpr auto tuple = l();
    constexpr auto x1 = std::get<0>(tuple);
    constexpr auto b1 = B<x1.size, x1.d[0], x1.d[1]>{}; 
    constexpr auto c1 = Util::tuple_tail(tuple);
    
//    decltype(c1)::_;
    
    return std::tuple(x1);
}

//constexpr auto t2 = transform_1(t1); // not possible
//constexpr auto t2 = transform_1(std::tuple(a1, a2)); // not possible

constexpr auto t2 = transform_2(l1); // OK!!!

int main() {
}
