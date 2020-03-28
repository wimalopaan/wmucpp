//#include <cstdint>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

//#ifdef   PROGMEM
//# undef  PROGMEM
//# define PROGMEM __attribute__((address_space(1)))
//#endif

//const uint16_t x PROGMEM = 1;

volatile uint8_t v1;
volatile uint8_t v2;

//struct A {
//    const uint8_t m{};
//};

int main() {
    sei();

    while(1) {
        v1 = v2;
    }

    
    DDRB = 0x01;
    
//    A a{32};
    
//    return x + a.m;
}


//ISR(ADC_vect) {
//    v2 = v1;
    
//}

extern "C" {
__attribute__((interrupt)) void __vector_21(void)  {
    v2 = v1;
}

}
