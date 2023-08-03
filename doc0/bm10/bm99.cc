#ifndef F_CPU 
# define F_CPU 2000000
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>


#if 0
struct HW {
    struct Led0 {
        static inline auto& port = PORTR;
        static inline constexpr uint8_t pin = 0b0000'0001;
        
        static void init() {
            port.DIRSET = pin;
        }
        static void toggle() {
            port.OUTTGL = pin;
        }
    };
};
 
int main() {
    HW::Led0::init();
    while(true) {
        HW::Led0::toggle();
        _delay_ms(500);
    }
}
#else

struct Address {
    const uint32_t value;
};

void f(Address a, uint32_t v);


struct HW {
    static inline constexpr uint8_t led0 = 0b0000'0001;
};
 
int main() {
    f(Address{1}, 2);
//    f(2, Address{1});
    
    PORTR.DIRSET = HW::led0;
    while(true) {
        PORTR.OUTTGL = HW::led0;
        _delay_ms(500);
    }
}

#endif
