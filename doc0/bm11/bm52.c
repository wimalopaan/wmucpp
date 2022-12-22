#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "bm52.h"

volatile uint8_t r;

static void fsm_part1(void) {
    r = 1;
}
static void fsm_part2(void) {
    r = 2;
}
static void fsm_part10(void) {
    r = 10;
}
static void fsm_part20(void) {
    r = 20;
}

typedef void(*fsm_part_t)(void) ;

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

static void fsm_periodic(void) {
    circular_call((fsm_part_t[]){fsm_part1, fsm_part2, fsm_part10, fsm_part20, (fsm_part_t)NULL});
}

int main() {
    while(true) { 
        fsm_periodic();
    }    
}
