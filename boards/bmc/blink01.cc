#include <avr/io.h>


int main() {
    DDRC = 0x01;
    
    while (true) {
        PINC = 0x01;
    }
}
