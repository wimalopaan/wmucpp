#include <stdint.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <stdatomic.h>
#include <stdbool.h>
    
static volatile  uint8_t x;

//ISR( TIMER2_OVF_vect ) {
//    volatile uint8_t y = x;
    
//}

void foo() __attribute__ ((__signal__));
void foo() {
//    x *= 2;
//    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        x += 3;
//    }
}

int main() {
    
    while(true) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            x += 2;
        }
    }

//    x = 0;
//    x = 1;
//    atomic_signal_fence(memory_order_seq_cst);    
//    x = 10;
//    x = 20;
//    atomic_signal_fence(memory_order_seq_cst); 
    
//    while(true) {}
    
}


