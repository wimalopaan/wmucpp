#include <stdint.h>

namespace {
    template<typename T>
    T foo(T x) {
        constexpr int8_t shift{6 - 10};
        if constexpr(shift < 0) {
            return x >> -shift;
        } else {
            return x << shift;
        }
    }
}

volatile uint8_t x = 0;
volatile uint8_t y = 0;

int main() {
    y = foo(x);
}
