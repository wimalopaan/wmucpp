#define NDEBUG

template<typename S1, typename S2>
[[noreturn]] void assertFunction([[maybe_unused]] const S1& expr, [[maybe_unused]] const S2& file, [[maybe_unused]] unsigned int line) {
    while(true) {
    }    
}

#include "board.h"

//using isrRegistrar = AVR::IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, btUsart::RxHandler, btUsart::TxHandler>;
//using isrRegistrar = AVR::IsrRegistrar<btUsart::RxHandler, btUsart::TxHandler>;

using terminal = etl::basic_ostream<rcUsart>;
using robo = etl::basic_ostream<btUsart>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    systemClock::init();
    
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    ppmInPin::dir<Output>();
    
    btUsart::init<9600>();
    rcUsart::init<115200>();
    
    uint8_t counter = 0;
    
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            btUsart::periodic();
            rcUsart::periodic();
            systemClock::periodic([&](){
                ppmInPin::toggle();
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t) {
                        outl<terminal>("cx: "_pgm, sumd::numberOfChannels());    
                        outl<terminal>("c0: "_pgm, sumd::value8Bit(0).toInt());    
                        outl<robo>("l0: "_pgm, counter);    
                        counter++;
                    }
                });
            });
        }    
    }
}

//// BT
//ISR(USART0_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
//}
//ISR(USART0_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
//}

//// SumD
//ISR(USART1_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
//}
//ISR(USART1_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
//}
