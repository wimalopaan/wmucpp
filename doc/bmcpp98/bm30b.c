#include <avr/interrupt.h>

struct AppFlags {
    int expired : 1;
} __attribute__((packed));

int main() {
}

ISR(TIMER0_COMPA_vect) { // warum die push/pop in dieser ISR
    volatile struct AppFlags* f = (struct AppFlags*)(0x3e);
    f->expired = 1;
}
