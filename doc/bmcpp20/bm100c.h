#pragma once

#include <stdint.h>
#include <stddef.h>

typedef void (*dev_t)(uint8_t);

void arithmetic_sum_reset();
void arithmetic_sum(uint8_t);
void arithmetic_sum_put(dev_t, uint16_t);

void binary_sum_reset();
void binary_sum(uint8_t);
void binary_sum_put(dev_t, uint16_t);

typedef struct {
    const void (*reset)(void);
    const void (*sum)(uint8_t);
    const void (*put)(dev_t, uint16_t);
    
} checker_t;

void protocol(const uint8_t* data, uint16_t length, dev_t dev, checker_t checker);

void spi1_put(uint8_t data);
void spi2_put(uint8_t data);
