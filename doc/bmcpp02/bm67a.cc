#include "util/meta.h"

struct A {
    typedef std::integral_constant<uint8_t, 0> v;
};

struct B {
    typedef std::integral_constant<uint8_t, 1> v;
};

using l1 = Meta::List<A, B>;

volatile uint8_t x;

int main() {
    Meta::visit<l1>([](auto v){
        using type = typename decltype(v)::type;
        x = type::v::value;
    });
}
