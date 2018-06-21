#include "bm100c.h"

uint8_t a1[2];
uint8_t a2[4];
uint8_t a3[8];
uint8_t a4[16];

const checker_t arithmetic = {.reset = arithmetic_sum_reset, .sum = arithmetic_sum, .put = arithmetic_sum_put};
const checker_t binary     = {.reset = binary_sum_reset,     .sum = binary_sum,     .put = binary_sum_put};

int main(){
    
    protocol(a1, sizeof(a1), spi1_put, arithmetic);
    protocol(a2, sizeof(a2), spi1_put, arithmetic);
    protocol(a3, sizeof(a3), spi2_put, binary);
    protocol(a4, sizeof(a4), spi2_put, binary);
}
