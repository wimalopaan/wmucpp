#define NDEBUG

#include <stdint.h>
#include "std/array.h"
#include "util/disable.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

constexpr uint8_t Base = 16;
std::array<char, Util::numberOfDigits<uint64_t, Base>() + 1> string;

int main() {
    Scoped<EnableInterrupt> interruptEnabler;
//    uint64_t value = 1234;
    uint16_t value = 0x04;
    Util::itoa_r<Base>(value, string);
    std::outl<terminal>(string);
//    std::byte v{value};
//    std::outl<terminal>(v);

    while(true) {}
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif

