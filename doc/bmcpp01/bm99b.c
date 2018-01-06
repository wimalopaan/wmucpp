#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bm99.h"

//#define M

#ifdef M
#define RISING_AGE( DATA, BIT )            \
    ({                  \
    static uint8_t  mem;\
    uint8_t ret = 0;\
    if ( (DATA & (1<<BIT))&&(mem==0))\
{\
    mem=1;\
    ret=1;\
    }\
    else\
{\
    mem=0;\
    ret=0;\
    }\
    ret;/* return value of Macro */  \
    })
#define FALLING_AGE( DATA, BIT )            \
    ({                  \
    static uint8_t  mem;\
    uint8_t ret = 0;\
    if ( (!(DATA & (1<<BIT)))&&(mem==0))\
{\
    mem=1;\
    ret=1;\
    }\
    else\
{\
    mem=0;\
    ret=0;\
    }\
    ret;/* return value of Macro */  \
    })
#endif

volatile uint8_t p = 1;
volatile uint8_t x = 0;

int main() {
    flag_t mem = {0};
#ifndef M
    if (rising_edge(&p, 0, &mem)) {
        x = 1;
    }
    if (rising_edge(&p, 0, &mem)) {
        x = 2;
    }
#else 
        if (RISING_AGE(p, 0)) {
            x = 1;
        }
        if (RISING_AGE(p, 0)) {
            x = 2;
        }
#endif    
    
    return x;
}
