#include <cstdint>

volatile uint8_t d;

struct A {
    A() {
        d = 1;
    }
    ~A() {
        d = 2;
    }
    uint8_t m{1};
};
struct B {
    B() {
        d = 10;
    }
    ~B() {
        d = 20;
    }
    uint16_t m{2};
};
union U {
    U() : a{} {
        d = 100;
    }
    ~U() {
        d = 200;
    }
    A a;
    B b;
};

int main() {
    U u;
    u.a = A();
//    u.b = B();
    return u.a.m ;
}
