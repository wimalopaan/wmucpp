#include <avr/io.h>

int main() {
    DDRB = 0x01;
    
    while (1) {
        PORTB = 0x01;
        PORTB = 0x02;
    }
}
