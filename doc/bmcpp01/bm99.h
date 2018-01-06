#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

struct Flag {
    uint8_t value;
};

typedef struct Flag flag_t;

inline static bool rising_edge(volatile uint8_t* data, uint8_t bit, flag_t* mem) {
    uint8_t ret = 0;
    if ( (*data & (1<<bit))&&(mem->value == 0)) {
        mem->value = 1;
        ret=1;
    }
    else {
        mem->value = 0;
        ret=0;
    }
    return ret;
}
