#define NDEBUG

#include "util/algorithm.h"

volatile uint8_t s = 0;

volatile uint8_t x = 0;

volatile uint8_t y1 = 0;
volatile uint8_t y2 = 0;
volatile uint8_t y3 = 0;

int main() {
    x = Util::select(s, y1, y2, y3);
}
