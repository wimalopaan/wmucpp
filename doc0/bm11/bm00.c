#include <avr/io.h>
#include <stdfix.h>
#include <stdint.h>
#include <stdbool.h>

volatile int8_t r1;
volatile int8_t r2;

//typedef signed short _Accum fp_t;

void bar(void);

static void foo();

int main() {
    foo();
    bar();
//    const int x = 2;
//    int a[x];
//    CCP = 0xd8;
//    CLKCTRL.MCLKCTRLB = 0x00;
//    PORTA.DIR = (1 << 6);
    
//    fp_t sum = 0;
//    while(true) {
//        PORTA.OUTTGL = (1 << 6);
        
//        const fp_t a = r1;
//        const fp_t b = r1;
//        sum += a * b;
//        fp_t d = sum * a;
//        r2 = d / 2.0hk;
////        r2 = sum;
//    }
}

static void foo() {
}

