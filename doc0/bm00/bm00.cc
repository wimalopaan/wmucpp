#include <mcu/avr.h>

#include <etl/type_traits.h>
#include <etl/fixedpoint.h>
#include <etl/control.h>

using fpq10 = etl::FixedPoint<int16_t, 10>;

volatile int16_t m = 0;
volatile int16_t d = 0;
volatile int16_t y = 0;
volatile uint16_t u = 0;
volatile uint16_t c = 0;

volatile fpq10 r;

int main() {    
//    Control::PID pid{fpq10{0.1}, fpq10{0.1}, fpq10{0.1}, 100, 30};
//    auto m2 = m;
//    auto y2 = y;
//    d = pid.correctionValue(m2, y2);

//     c = (int32_t(m) * s) / u;
    uint16_t s = 31;
     c = (int32_t(m) * s) >> 7;
    
}
