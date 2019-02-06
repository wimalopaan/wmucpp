#include <etl/type_traits.h>
#include <etl/fixedpoint.h>
#include <mcu/avr.h>

volatile uint16_t v1;
volatile uint16_t v2;
volatile uint16_t o;

constexpr uint16_t m = 3200;

template<typename T>
T mul(T a, T b) {
    uint32_t y = (int32_t)a * b + (int32_t)m * m - (int32_t)m * (a + b) + m;
    return y >> 16;
}

int main() {
    o = mul(v1, v2);
    asm(";xx");
    
}
