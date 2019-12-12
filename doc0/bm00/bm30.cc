#include <avr/io.h>

#include "bm30.h"

struct Bits {
    uint8_t bit0:1;
    uint8_t bit1:1;
    uint8_t bit2:1;
    uint8_t bit3:1;
    uint8_t bit4:1;
    uint8_t bit5:1;
    uint8_t bit6:1;
    uint8_t bit7:1;
};

int main() {
//    setBit(1);
//    PORTA.INTFLAGS = 0x01; // korrekt
    auto p = reinterpret_cast<volatile Bits*>(&PORTA);
//    auto p = reinterpret_cast<volatile Bits*>(0x10);
    p->bit0 = 1; // falsch
    
}
