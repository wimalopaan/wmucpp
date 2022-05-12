#include <avr/io.h>

volatile unsigned short a;

int main() {
//    PORTB = 0x01; // no warning
    
    (*(volatile uint8_t *)((0x05) + 0x20)) = 0x01; // warning
    
}
