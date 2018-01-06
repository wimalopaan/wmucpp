#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bm99.h"

extern volatile uint8_t p;
extern volatile uint8_t x;

void foo() {
    flag_t mem = {0};
    if (rising_edge(&p, 0, &mem)) {
        x = 1;
    }
    if (rising_edge(&p, 0, &mem)) {
        x = 2;
    }
}
