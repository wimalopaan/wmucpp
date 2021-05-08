#include <cstdint>
#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>

using ccp = AVR::Cpu::Ccp<>;
using clock = AVR::Clock<>;

int main() {
    ccp::unlock([]{
        clock::prescale<2>();
    } );
}
