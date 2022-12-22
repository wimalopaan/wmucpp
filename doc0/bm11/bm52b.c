#include "bm52.h"

void circular_call(const fsm_part_t* const functions) {
    static uint8_t n = 0;
    const fsm_part_t f = functions[n];    
    if (f) {
        (*f)();
        ++n;
    }
    else {
        n = 0;
    }
}
