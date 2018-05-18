#define NDEBUG

#include <stdint.h>

volatile uint8_t x = 0;

enum State {StateA, StateB, StateC};
typedef enum State state_t;

enum Event {E1, E2, E3};
typedef enum Event event_t;

static inline void process(event_t e) {
    static state_t state = StateA;
    switch(state) {
    case StateA: 
        if (e == E1) { 
            state = StateB; 
            x = 1;
        }
        break;
    case StateB: 
        if (e == E2) { 
            state = StateC;
            x = 2;
        }
        break;
    case StateC: 
        if (e == E3) { 
            state = StateA;
            x = 0;
        }
        break;
    }
}

int main() {
    process(E1);
    process(E2);
    process(E3);
}
