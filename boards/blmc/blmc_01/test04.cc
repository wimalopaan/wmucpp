#define NDEBUG

#include <stdlib.h>
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/mcupwm.h"
#include "mcu/avr/adcomparator.h"
#include "mcu/avr/watchdog.h"
#include "mcu/avr/twislave.h"
#include "external/hott/hott.h"

#include "console.h"

namespace Constants {
    static constexpr std::hertz pwmFrequency = 1000_Hz * 256; 
    static constexpr std::hertz fSystem = 100_Hz;
}

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

using hall0 =  AVR::Pin<PortB, 0>;
using hall1 =  AVR::Pin<PortB, 1>;
using hall2 =  AVR::Pin<PortB, 2>;

using ppmIn =  AVR::Pin<PortE, 0>;

using oc3a =  AVR::Pin<PortD, 0>;
using oc3b =  AVR::Pin<PortD, 2>;

//using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0, UseEvents<false>, AsciiHandler, BinaryHandler, BCastHandler>, MCU::UseInterrupts<true>, UseEvents<false>> ;

using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>, MCU::UseInterrupts<true>, UseEvents<false>>;

using terminalDevice = rcUsart;
using terminal = std::basic_ostream<terminalDevice>;

constexpr TWI::Address address{0x50_B};
using i2c = TWI::Slave<0, address, 2>; // todo: no interrupts

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, i2c>;

using hardPwm = AVR::PWM<3, AVR::NonInverting>; // timer 3 Achtung: Umstellen auf OC3B

int main() {
    isrRegistrar::init();
    rcUsart::init<9600>();
    
    led::dir<AVR::Output>();
    
    pinHigh0::dir<AVR::Output>();
    pinHigh1::dir<AVR::Output>();
    pinHigh2::dir<AVR::Input>();
    
    oc3b::dir<AVR::Output>();
    oc3b::on(); // Achtung: Output Compare Modulator Bug
    
    hardPwm::init<Constants::pwmFrequency>();
    hardPwm::pwm<hardPwm::B>(0);
    
    pinLow0::dir<AVR::Output>();
    pinLow1::dir<AVR::Output>();
    pinLow2::dir<AVR::Output>();
    
    pinLow0::on();
    pinLow1::on();
//    pinLow2::on();

    {
        Scoped<EnableInterrupt<>> ei;
        std::outl<terminal>("Test04"_pgm);
        std::outl<terminal>(hardPwm::frequency());
        
        while(true) {
            Util::delay(100_ms);
            led::toggle();
        }
    }
}

ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}
