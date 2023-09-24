
#include <stdfix.h>
#include <stdint.h>
#include <stdbool.h>

volatile uint16_t r1;
volatile uint16_t r2;
volatile uint16_t r3;

//typedef signed short  _Accum fp_t;

#define X 1
int main() {
    while(X) {
        
    }
//    unsigned _Accum x;
    
    
//    fp_t sum = 0;
//    while(true) {
//        const fp_t a = r1;
//        const fp_t b = r2;
//        sum += a * b;
//        r3 = sum;
////        r3 = sum / 2;
//    }
    
    printf("%02d\n", 12345678%100);
}
