#include <cstdint>
#include <cstddef>

volatile uint8_t x;

void tft_wr(uint8_t* arr, size_t length) {
    x = *arr;
}
