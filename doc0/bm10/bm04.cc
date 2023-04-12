#include <avr/io.h>
#include <stdint.h>

uint16_t b;
uint8_t a;

template<typename A, typename B>
B Mul(const A a, const B b) {
    static const uint8_t shift = (sizeof(B) - sizeof(A)) * 8;
    return static_cast<A>(b >> shift) * a ;
}

uint16_t mul(const uint8_t a, const uint16_t b) {
    const uint8_t x = b >> 8;
    return (x * a);
//    return static_cast<uint8_t>((b >> 8)) * a ;
}

uint8_t test(uint8_t a, uint8_t b) { return (a*b) >> 7; }

int main() {
    return mul(a, b);
}

//#include <stdint.h>

//#include <avr/io.h>

//enum E1 {a, b, c, d};
//enum E2 {e, f, g, h};

//struct R1 {
//    uint8_t a:1;
//    enum E1 b:2;
//    uint8_t c:3;
//    uint8_t d:4;
//};
//struct R2 {
//    uint8_t w:4;
//    enum E2 x:3;
//    uint8_t y:2;
//    uint8_t z:1;
//};

//struct R1 r1;
//struct R2 r2;

//int main(void) {
//    return bit_is_set(PORTB, 10);
    
//    r1.b = e;
//    r2.x = a;
//}
