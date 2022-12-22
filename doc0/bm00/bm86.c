#include <stdint.h>

static uint8_t ct0 = 0xFF, ct1 = 0xFF;


//void foo(const char* const*);
//void foo(const char* const* const a) {}

//void bar(int);
//void bar(const int b) {}

static void increment(uint8_t i) {
    ct0 = ~( ct0 & i );                             // reset or count ct0
    ct1 = ct0 ^ (ct1 & i);                          // reset or count ct1
}

int main() {
    increment(0x01);
    increment(0x01);
    increment(0x01);
    increment(0x01);
    
    return ct0 & ct1;    
}
