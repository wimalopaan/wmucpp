#include <mcu/avr.h>
#include <etl/types.h>
#include <etl/algorithm.h>

using namespace AVR;

volatile uint8_t r1;

int main() {
    while(true) {
        etl::circular_call(
                    [&]{r1 = 0;}, [&]{r1 = 1;}, [&]{r1 = 2;}
        );
        
    }
}
