#include <stdlib.h>
#include <stdio.h>

#if 0
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

struct Flag {
    uint8_t value;
};

typedef struct Flag flag_t;

inline bool rising_edge(volatile uint8_t* data, uint8_t bit, flag_t* mem) {
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

volatile uint8_t p = 0;

int main() {
    flag_t mem{0};
    
    if (rising_edge(&p, 0, &mem)) {
        printf("A\n");        
    }
    p = 1;
    if (rising_edge(&p, 0, &mem)) {
        printf("B\n");        
    }
//    if (RISING_AGE(p, 0)) {
//    }
//    if (RISING_AGE(p, 0)) {
//        printf("B\n");        
//    }
    
}
