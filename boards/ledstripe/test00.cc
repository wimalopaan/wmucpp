#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/event.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/spi.h>

#include <external/hal/alarmtimer.h>
#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
 
using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;

using spi = AVR::Spi<spiPosition, AVR::UseInterrupts<false>>;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });

    spi::init();
    
}

