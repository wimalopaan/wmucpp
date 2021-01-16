#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

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
#endif

typedef struct {
  const char *projectName;
  const char *hardwareDate;
  const char *madeBy;  
} DevBoard_t;

typedef union {
    DevBoard_t db;
    const char* array[3];
} Strings;


void sendString(const char* str) {
    assert(str);
    do {
       printf("%c",*str);
    }  while (*str++);
}

void foo(char*) {
    
}

int main() {
    
    foo("abc");
    
    
    DevBoard_t db = {"abc", "def", "ghi"};
    Strings s = {db};

    printf("n: %s\n", s.array[0]);    
    printf("h: %s\n", s.array[1]);    
    printf("m: %s\n", s.array[2]);    
    
    const char** ptr = s.array;

    printf("n: %s\n", *ptr++);    
    printf("h: %s\n", *ptr++);    
    printf("m: %s\n", *ptr);    
    
//    flag_t mem{0};
    
//    if (rising_edge(&p, 0, &mem)) {
//        printf("A\n");        
//    }
//    p = 1;
//    if (rising_edge(&p, 0, &mem)) {
//        printf("B\n");        
//    }
////    if (RISING_AGE(p, 0)) {
////    }
////    if (RISING_AGE(p, 0)) {
////        printf("B\n");        
////    }
    
}
