#define NDEBUG

#include <stdlib.h>
#include "mcu/avr8.h"
#include "mcu/ports.h"

#include "console.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using db0 =  AVR::Pin<PortB, 0>;
using db1 =  AVR::Pin<PortB, 6>;
using db2 =  AVR::Pin<PortB, 7>;

int main() {
    OSCCAL = 0xff;
    db0::dir<AVR::Output>();
    while(true) {
        db0::toggle();
    }
 }
