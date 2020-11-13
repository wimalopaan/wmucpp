#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>

#include <external/hal/alarmtimer.h>
#include <external/hott/sumdprotocolladapter.h>
//#include <external/hott/sumdprotocoll.h>
#include <external/solutions/gps.h>
#include <external/solutions/series01/swuart.h>
#include <external/ibus/ibus.h>

#include <external/hal/adccontroller.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>
#include <etl/converter.h>
#include <etl/fixedpoint.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;
using PortF = Port<F>;

using led = Pin<PortF, 5>; 

using dbg0 = Pin<Port<C>, 0>;
using dbg1 = Pin<Port<C>, 1>;
using dbg2 = Pin<Port<A>, 7>;

//using pf2 = Pin<PortF, 2>; 
//using pa0 = Pin<PortA, 0>; 
//using pd0 = Pin<Port<D>, 0>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
//using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Alt1>;
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Alt1>;

//using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltB>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart2Position>>;

using terminalDevice = Usart<usart2Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

//using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
//using rcUsart = AVR::Usart<usart1Position, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

using vtg = External::GPS::VTG;
using rmc = External::GPS::RMC;
using gps = External::GPS::GpsProtocollAdapter<0, vtg, rmc>;

//using gpsUsart = AVR::Usart<usart1Position, 
////                            External::Hal::NullProtocollAdapter, 
//                            gps, 
//                            AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using rxPin = Pin<Port<C>, 5>;
using txPin = void;
using gpsUsart = External::SoftSerial::Usart<Meta::List<rxPin, void>, Component::Tca<0>,
                                            External::GPS::GpsProtocollAdapter<0, vtg, rmc>,
                                            AVR::BaudRate<9600>>;

//using gpsUsart = External::SoftSerial::Usart<Meta::List<rxPin, void>, Component::Tcb<0>,
//                                            External::GPS::GpsProtocollAdapter<0, vtg, rmc>,
//                                            AVR::BaudRate<9600>, AVR::ReceiveQueueLength<0>>;

template<typename VTG>
struct SpeedProvider {
//    inline static constexpr auto ibus_type = IBus::Type::type::GROUND_SPEED; // m/s
    inline static constexpr auto ibus_type = IBus::Type::type::SPEED; // km/h
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        return s.integer();
    }
    static inline etl::FixedPoint<uint16_t, 4> s;
};

using speedP = SpeedProvider<vtg>;

template<typename ADC, uint8_t Channel>
struct InternalTempProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        return sigrow::adcValueToTemperature<std::ratio<1,10>, 40 - 15>(ADC::value(channel)).value;
    }
};

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0x1e>>; // 1e = temp
using iTempP = InternalTempProvider<adcController, 0>;

using isrRegistrar = IsrRegistrar<typename gpsUsart::StartBitHandler, typename gpsUsart::BitHandler>;

namespace  {
    constexpr auto dt = 2_ms;
    constexpr auto fRtc = 2000_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<speedP, iTempP>, 
                          systemTimer, void>;



int main() {
#if 1
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    terminalDevice::init<BaudRate<9600>>();
//    gpsUsart::init<AVR::HalfDuplex>();
    gpsUsart::init();
    ibus::init();
    
////    rcUsart::init<BaudRate<115200>>();
    
    
    systemTimer::init();
    
    led::template dir<Output>();     
    dbg0::template dir<Output>();     
    dbg1::template dir<Output>();     
    dbg2::template dir<Output>();     
//    pd0::template dir<Output>();     
//    pf2::template dir<Output>();     
//    pa0::template dir<Output>();     

    led::high();
//    pd0::high();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    etl::StringBuffer<External::GPS::Sentence::DecimalMaxWidth> s;
    etl::StringBuffer<External::GPS::Sentence::TimeMaxWidth> time;
    s.insertAt(0, "bla"_pgm);
    
    while(true) {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        
//        pf2::toggle();
        terminalDevice::periodic();
//        gpsUsart::periodic();
//        rcUsart::periodic();
        ibus::periodic();
//        auto d = gpsUsart::get();
//        if (d) {
//            terminalDevice::put(*d);
//        }
        systemTimer::periodic([&]{
            ibus::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
                    
                    // 50 us
                    // 117 us (bei 32-Bit parser)
                    // 17us (nur ein Dezimal)
                    dbg2::high();
                    vtg::speedRaw(s);
                    auto ss = etl::StringConverter<etl::FixedPoint<uint16_t, 4>>::parse<1>(s);
                    speedP::s = ss * 100;
                    dbg2::low();
                    
                    rmc::timeRaw(time);
                    
                    etl::outl<terminal>("test00: "_pgm, s, " t: "_pgm, time, " ss: "_pgm, ss);
                }
            });
        });
    }
#endif
}

ISR(PORTC_PORT_vect) {
    dbg0::high();
    isrRegistrar::isr<AVR::ISR::Port<rxPin::name_type>>();
    dbg0::low();
}

ISR(TCA0_OVF_vect) {
    dbg1::high();
    isrRegistrar::isr<AVR::ISR::Tca<0>::Ovf>();
    dbg1::low();
}

