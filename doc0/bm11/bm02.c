#include <avr/io.h>
#include <stdfix.h>
#include <stdint.h>
#include <stdbool.h>

volatile int8_t r1;
volatile int8_t r2;

typedef float fp_t;

// 33,5µs
// 50,3µs

int main() {
    CCP = 0xd8;
    CLKCTRL.MCLKCTRLB = 0x00;
    PORTA.DIR = (1 << 6);
    
    fp_t sum = 0;
    while(true) {
        PORTA.OUTTGL = (1 << 6);
        
        const fp_t a = r1;
        const fp_t b = r1;
        sum += a * b;
        fp_t d = sum * a;
        r2 = d / 2.0f;
//        r2 = sum;
    }
}
