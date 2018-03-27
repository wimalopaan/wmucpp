#define NDEBUG

#include "local.h"
#include "rcsensorled10.h"
#include "console.h"


using testPin0 = AVR::Pin<PortA, 0>; // LipoA1 = Chan0
using testPin1 = AVR::Pin<PortA, 1>; // LipoA2 = Chan1
using testPin2 = AVR::Pin<PortA, 2>; // LipoA3 = Chan2

using rcUsart = AVR::Usart<1, void, MCU::UseInterrupts<true>, UseEvents<false>, AVR::ReceiveQueueLength<32>>;

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler>;

using terminalDevice = rcUsart;
using terminal = std::basic_ostream<terminalDevice>;

int main() {
    isrRegistrar::init();
    rcUsart::init<19200>();
    {
        Scoped<EnableInterrupt<>> ei;
        uint8_t counter = 0;
        
        while(true) {
            Util::delay(333_ms);
            std::outl<terminal>("Test03: "_pgm, ++counter);
        }
    }
}

// SumD
ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif
