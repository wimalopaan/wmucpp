#define NDEBUG
#define OUTPUT

#include "local.h"
#include "rcsensorled10.h"
#include "console.h"

using testPin0 = AVR::Pin<PortA, 0>; // LipoA1 = Chan0
using testPin1 = AVR::Pin<PortA, 1>; // LipoA2 = Chan1
using testPin2 = AVR::Pin<PortA, 2>; // LipoA3 = Chan2

struct AsciiHandler;
struct BinaryHandler;
struct BCastHandler;

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0, UseEvents<false>, AsciiHandler, BinaryHandler, BCastHandler>, 
MCU::UseInterrupts<true>, UseEvents<false>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>, MCU::UseInterrupts<true>, UseEvents<false>>;

using terminalDevice = rcUsart;
using terminal = std::basic_ostream<terminalDevice>;

struct AsciiHandler {
    static void start() {
#ifdef OUTPUT
        std::outl<terminal>("hba start"_pgm);
#endif
    }    
    static void stop() {
    }    
    static void process(std::byte) {
    }
};
struct BinaryHandler {
    static void start() {
#ifdef OUTPUT
        std::outl<terminal>("hbb start"_pgm);
#endif
    }    
    static void stop() {
    }    
};
struct BCastHandler {
    static void start() {
#ifdef OUTPUT
        std::outl<terminal>("hbr start"_pgm);
#endif
    }    
    static void stop() {
    }    
};

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, 
                                  gpsUsart::RxHandler, gpsUsart::TxHandler,
                                  sensorUsart::RxHandler, sensorUsart::TxHandler
>;

int main() {
    isrRegistrar::init();
    
    gpsUsart::init<9600>();
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    {
        Scoped<EnableInterrupt<>> ei;
        StringBuffer<GPS::Sentence::TimeMaxWidth> time;
        std::outl<terminal>("Test05"_pgm);
        while(true) {
            Util::delay(1000_ms);
            GPS::RMC::timeRaw(time);
            std::outl<terminal>("time: "_pgm, time);
            auto v0 = Hott::SumDProtocollAdapter<0>::value(0);
            std::outl<terminal>("channel 0: "_pgm, v0.toInt());
        }
    }
}
// Sensor
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
// GPS
ISR(USART2_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<2>::RX>();
}
ISR(USART2_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<2>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif
