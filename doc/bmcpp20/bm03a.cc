#define NDEBUG

#include <stdint.h>
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
    uint32_t value = 1234;
    ltoa(value, &string[0], 10);
    std::outl<terminal>(string);

    while(true) {}
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif
