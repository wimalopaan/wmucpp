#include <cstddef>
#include <array>
#include <algorithm>
#include <iostream>

struct A {
    int m;
};
struct B {
    int m;
};

struct V {
    union {
        A a;
        B b;
    };
};

V v;
    
int main() {
}
