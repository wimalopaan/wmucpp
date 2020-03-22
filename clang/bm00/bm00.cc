#include <cstdint>

#include <avr/io.h>
#include <avr/pgmspace.h>

#ifdef   PROGMEM
# undef  PROGMEM
# define PROGMEM __attribute__((address_space(1)))
#endif

const uint16_t x PROGMEM = 1;

int main() {
    return x;
}
