#include "mcu.h"

#include "algorithm.h"

volatile uint16_t a;
volatile uint16_t b;
volatile uint16_t c;

void f() __attribute__((noinline));
void f() {
    c = a % b;
}

std::array<float, 32> v;
volatile float m;

void g1() {
    m = etl::maximum(v);
}

void g2() {
    float r{};
    for(uint8_t i = 0; i < v.size(); ++i) {
        if (v[i] > r) {
            r = v[i];
        }
    }
    m = r;
}

volatile float f1, f2;

int main(){
    
    f1 = f1 * f2;
    
    while(true) {
//        f();
        g2();
    }
}
