#include <stdint.h>

volatile uint8_t result = -1;
volatile uint8_t value = 159;

// Grenzen?
// Datentypen
// andere Intervalles
// Testen?

uint8_t lookup1(uint8_t value, uint8_t min, uint8_t max) {
    uint8_t l = value / 10; 
    if (l < min) {
        l = min;
    }
    else if (l > max) {
        l = max;
    }
    return l;
}

int main() {
    result = lookup1(value, 1, 5);
}
