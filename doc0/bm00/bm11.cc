#include <stdint.h>

#define DIR_HI  0x00
#define DIR_LO  0x02
#define PWM_2   0x00
#define PWM_1   0x01

volatile uint8_t r;

namespace {
    void High_IN1() {
        r = 1;
    }
    void High_IN2() {
        r = 0;
    }
    void Low_IN1() {
        r = 0;    
    }
    void Low_IN2() {
        r = 1;    
    }

    volatile uint8_t analogRichtung{};
    volatile uint8_t analogPwm{};
}

int main() {
    switch(analogRichtung | (analogPwm << 1)) {
      case DIR_HI | PWM_2: High_IN2(); break;
      case DIR_HI | PWM_1: High_IN1(); break;
      case DIR_LO | PWM_2: Low_IN2(); break;
      case DIR_LO | PWM_1: Low_IN1(); break;
    }
}
