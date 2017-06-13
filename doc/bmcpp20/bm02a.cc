#define NDEBUG

#include <stdint.h>
#include "mcu/avr8.h"
#include "std/array.h"
#include "util/disable.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

constexpr uint8_t Base = 10;
std::array<char, Util::numberOfDigits<uint64_t, Base>() + 1> string;

int main() {
//    Scoped<EnableInterrupt> interruptEnabler;
//    uint64_t value = 1234;
    uint32_t value = 1234;
    Util::V2::itoa<Base>(value, string);
    std::outl<terminal>(string);

    while(true) {}
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif

