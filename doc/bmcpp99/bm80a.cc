#include <cstdint>

[[using gnu: pure, always_inline]] int fact(int n) {
    int f = 1;
    while(n > 0) {
        f *= n--;
    }
    return f;
}

int foo(int n) {
    return fact(n) + fact(n);
}

volatile int a = 2;
int main() {
    return foo(a);
}

