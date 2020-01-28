#include <utility>
#include <array>
#include <variant>
#include <type_traits>

volatile uint8_t d;

struct A {
    A() {
        d = 1;
    }
    ~A() {
        d = 2;
    }
};
struct B {
    B() {
        d = 10;
    }
    ~B() {
        d = 20;
    }
};



int main() {
    std::array<std::variant<A, B, nullptr_t>, 4> p;
//    std::array<std::variant<A, B, nullptr_t>, 4> p{A{}, B{}, nullptr};
//    std::variant<A, B, nullptr_t> v;
//    std::detail::recursive_union<std::integral_constant<size_t, 0>, A, B> v;
//    v = A();
//    v = B();    
}
