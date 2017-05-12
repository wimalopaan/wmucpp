#define NDEBUG

#include <stdint.h>
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "hal/constantrate.h"
#include "std/array.h"
#include "std/concepts.h"
#include "util/disable.h"
#include "util/fixedpoint.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

std::array<char, Util::numberOfDigits<uint64_t>() + 1> string;
constexpr uint8_t Base = 10;

int main() {
    Scoped<EnableInterrupt> interruptEnabler;
    uint64_t value = 1234;
    Util::V2::itoa<Base>(value, string);
    std::outl<terminal>(string);

    while(true) {}
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif

