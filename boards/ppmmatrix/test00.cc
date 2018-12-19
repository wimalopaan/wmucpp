#define NDEBUG

template<typename S1, typename S2>
[[noreturn]] void assertFunction([[maybe_unused]] const S1& expr, [[maybe_unused]] const S2& file, [[maybe_unused]] unsigned int line) {
    while(true) {
    }    
}

#include "board.h"

using isrRegistrar = AVR::IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, btUsart::RxHandler, btUsart::TxHandler>;
//using isrRegistrar = AVR::IsrRegistrar<btUsart::RxHandler, btUsart::TxHandler>;

using terminal = etl::basic_ostream<btUsart>;

int main() {
    using namespace etl;
    btUsart::init<9600>();

    outl<terminal>("abc"_pgm);    
    
    while(true) {
//        btUsart::periodic();
        ppmInPin::toggle();
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
