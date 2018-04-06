#define NDEBUG

#include "util/algorithm.h"

volatile uint8_t s = 0;

volatile uint8_t x = 0;

volatile uint8_t y1 = 0;
volatile uint8_t y2 = 0;
volatile uint8_t y3 = 0;

int main() {
    if (s == 0) {
        x = y1;
    }
    else if (s == 1) {
        x = y2;
    }
    else  {
        x = y3;
    }
    //    else if (s == 2) {
//        x = y3;
//    }
//    else {
//        assert(false);
//    }
}
