#define NDEBUG

#include "ppmmatrix.h"
#include "console.h"

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, btUsart::RxHandler, btUsart::TxHandler>;

using terminalDevice = btUsart;
using terminal = std::basic_ostream<terminalDevice>;

int main() {
    multiplexer::init();
        
    btUsart::init<9600>();
    rcUsart::init<115200>();
    
    cppm::init();

    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

    {
        Scoped<EnableInterrupt<>> ei;
        uint8_t counter = 0;
        
        uintN_t<3> select;
        multiplexer::select(select);
           
        while(true) {
            cppm::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([&](){
                alarmTimer::periodic([&](uint7_t timer) {
                    if (timer == *periodicTimer)  {
                         std::outl<terminal>("test03: "_pgm, counter++);
                    }
                });
            });
        }
    }
}

// BT
ISR(USART0_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
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
