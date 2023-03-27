#include <stdint.h>

#include <avr/io.h>

enum E1 {a, b, c, d};
enum E2 {e, f, g, h};

struct R1 {
    uint8_t a:1;
    enum E1 b:2;
    uint8_t c:3;
    uint8_t d:4;
};
struct R2 {
    uint8_t w:4;
    enum E2 x:3;
    uint8_t y:2;
    uint8_t z:1;
};

struct R1 r1;
struct R2 r2;

int main(void) {
    return bit_is_set(PORTB, 10);
    
    r1.b = e;
    r2.x = a;
}
