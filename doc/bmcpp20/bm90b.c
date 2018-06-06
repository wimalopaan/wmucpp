#include <stdint.h>

volatile uint8_t result = -1;
volatile uint8_t v = 100;

int main() {
#if 0
    {
        uint8_t value = v;
        if ((value >= 0) && (value < 10)) {
            result = 0;
        }    
        else if ((value >= 10) && (value < 20)) {
            result = 1;
        }
        else if ((value >= 20) && (value < 40)) {
            result = 2;
        }
        else if ((value >= 40) && (value < 80)) {
            result = 3;
        }
        else if ((value >= 80) && (value < 160)) {
            result = 4;
        }
    }
#else
    {
        uint8_t value = v;
        if ((value < 10)) {
            result = 0;
        }    
        else if ((value < 20)) {
            result = 1;
        }
        else if ((value < 40)) {
            result = 2;
        }
        else if ((value < 80)) {
            result = 3;
        }
        else if ((value < 160)) {
            result = 4;
        }
    }
#endif
}
