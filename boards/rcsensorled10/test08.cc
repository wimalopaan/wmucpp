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

namespace Constants {
    static constexpr const std::hertz fSCL = 100000_Hz;
}

int main() {
    isrRegistrar::init();
    TwiMaster::init<Constants::fSCL>();
    rcUsart::init<115200>();
    { 
        Scoped<EnableInterrupt<>> ei;
        uint8_t counter = 0;
        {
            mcp23008::startWrite(0x00, std::byte{0x00}); // output
            mcp23008::startWrite(0x09, std::byte{0x00}); // output
            while(true) {
//                std::array<TWI::Address, 4> i2cAddresses;
//                TwiMaster::findDevices(i2cAddresses);
//                for(const auto& d : i2cAddresses) {
//                    if (d) {
//                        std::outl<terminal>(d);
//                        Util::delay(100_ms);
//                    }
//                }
//                Util::delay(1000_ms);
                std::outl<terminal>("Test08: "_pgm, ++counter);
//                for(auto& d : i2cAddresses) {
//                    d = TWI::Address{};
//                }

                mcp23008::startWrite(0x09, std::byte{counter});
                do {
                    TwiMasterAsync::periodic();
                } while(TwiMasterAsync::active());
                blmc::startWrite(0, std::byte{counter});
                do {
                    TwiMasterAsync::periodic();
                } while(TwiMasterAsync::active());
                
                Util::delay(1000_ms);
            }
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
