#define NDEBUG

#include <stdlib.h>
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/mcupwm.h"
#include "mcu/avr/adcomparator.h"
#include "mcu/avr/watchdog.h"
#include "external/hott/hott.h"

#include "console.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

using pinLow0 = AVR::Pin<PortB, 7>;
using pinHigh0 = AVR::Pin<PortD, 3>;
using pinLow1 = AVR::Pin<PortD, 5>;
using pinHigh1 = AVR::Pin<PortD, 4>;
using pinLow2 = AVR::Pin<PortD, 7>;
using pinHigh2 = AVR::Pin<PortE, 1>;

using led =  AVR::Pin<PortB, 5>;

//using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0, UseEvents<false>, AsciiHandler, BinaryHandler, BCastHandler>, 
//MCU::UseInterrupts<true>, UseEvents<false>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>, MCU::UseInterrupts<true>, UseEvents<false>>;

using terminalDevice = rcUsart;
using terminal = std::basic_ostream<terminalDevice>;

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler>;

int main() {
    rcUsart::init<9600>();
    
    led::dir<AVR::Output>();
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            Util::delay(100_ms);
            led::toggle();
            
            std::outl<terminal>("test02"_pgm);
        }
    }
}

ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}
