//#define NDEBUG

template<typename S1, typename S2>
[[noreturn]] void assertFunction([[maybe_unused]] const S1& expr, [[maybe_unused]] const S2& file, [[maybe_unused]] unsigned int line) {
    while(true) {
    }    
}

#include "board.h"

using isrRegistrar = AVR::IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, btUsart::RxHandler, btUsart::TxHandler>;

using terminal = etl::basic_ostream<btUsart>;


int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;

    btUsart::init<9600>();

    uint8_t counter = 0;

    Fraction<uint8_t, 4> fraction(0);

    FixedPoint<uint8_t, 4> fp;

    std::byte b{16};
    
    bool bb = true;
    
    int s = -1;
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            Util::delay(1000_ms);
            outl<terminal>("abc: "_pgm, counter++, fraction, fp, b);    
            outl<terminal>("abc: "_pgm, b, bb, Char{'z'}, s);    
            ppmInPin::toggle();
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
