#include <avr/io.h>
#include <cstdint>

struct TCA_INTFLAGS_bits {
    uint8_t ovl:1;   
    uint8_t cmp0:1;   
} __attribute__((__packed__));

#define BF_TCA_INTFLAGS  (*(volatile struct TCA_INTFLAGS_bits *) &TCA0.SINGLE.INTFLAGS)


int main() {  

    BF_TCA_INTFLAGS.ovl = 1;

    return 0; 
}
