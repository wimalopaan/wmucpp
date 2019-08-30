#include <avr/io.h>

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
    PORTA.INTFLAGS = 0x01; // korrekt
    auto p = reinterpret_cast<volatile Bits*>(&PORTA);
    p->bit0 = 1; // falsch
}
