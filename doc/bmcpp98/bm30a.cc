#include <avr/interrupt.h>

struct AppFlags final {
    bool expired : 1;
} __attribute__((packed));

int main() {
}

//ISR(TIMER0_COMPA_vect) __attribute__((naked));

ISR(TIMER0_COMPA_vect) { // warum die push/pop in dieser ISR
    auto f = reinterpret_cast<volatile AppFlags*>(0x3e);
    f->expired = true;
//    reti();
}
