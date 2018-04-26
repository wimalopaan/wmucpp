#include <stdint.h>

volatile uint16_t ubrr;

constexpr uint16_t foo(uint16_t b) {
    uint16_t v = ((F_CPU / (b * 16L)) - 1);
    return v;
}

int main() {
    constexpr auto u = foo(2400); 
    ubrr =  u;
}
