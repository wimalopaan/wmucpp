#include <cstdint>
#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>

using ccp = AVR::Cpu::Ccp<>;
using clock = AVR::Clock<>;

int main() {
    return  etl::uint_ranged<uint8_t, 0, 9>{10};
}
