#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/port.h>
#include <cstdint>
#include <cstddef>
#include <etl/fixedpoint.h>

using namespace AVR;
using PortA = AVR::Port<AVR::A>;
using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using dbg  =  AVR::Pin<PortA, 6>;

static_assert(etl::has_arithmetic_right_shift_v<int16_t>);

using fp_t = etl::FixedPoint<int16_t, 8>; 
using fp_int_t = fp_t::integer_type;

volatile int8_t r1;
volatile int8_t r2;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });
    dbg::dir<Output>();
    
    fp_t sum;
    while(true) {
        dbg::toggle();
        const auto a = fp_t{r1}; 
        const auto b = fp_t{r1}; 
        sum += a * b;
//        auto d = sum * a;
//        r2 = (d / 2).integer();
        r2 = sum.integer();
    }    
}


