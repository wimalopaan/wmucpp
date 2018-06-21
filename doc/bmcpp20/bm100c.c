#include "bm100c.h"

volatile uint8_t sfr1;

void spi1_put(uint8_t data) {
    sfr1 = data;
}

volatile uint8_t sfr2;

void spi2_put(uint8_t data) {
    sfr2 = data;
}

void protocol(const uint8_t* data, uint16_t length, dev_t dev, checker_t checker) {
    checker.reset();
    for(uint16_t i = 0; i < length; ++i) {
        dev(data[i]);
        checker.sum(data[i]);
    }
    checker.put(dev, length);
}

static uint32_t asum;

void arithmetic_sum_reset() {
    asum = 0;
}
void arithmetic_sum(uint8_t v) {
    asum += v;
}
void arithmetic_sum_put(dev_t dev, uint16_t length) {
    dev(asum);
    dev(asum >> 8);
    if (length >= 256) {
        dev(asum >> 16);
        dev(asum >> 24);
    }
}

static uint8_t bsum;

void binary_sum_reset() {
    bsum = 0;
}
void binary_sum(uint8_t v) {
    bsum ^= v;
}
void binary_sum_put(dev_t dev, uint16_t length) {
    dev(bsum);
}
