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
#include <external/sbus/sport.h>

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
using dbg2 = Pin<Port<C>, 2>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

namespace  {
    constexpr auto fRtc = 2000_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

//using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
//using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Alt1>;
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Alt1>;

//using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltB>;

//using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart2Position>>;
using portmux = Portmux::StaticMapper<Meta::List<usart2Position>>;

using terminalDevice = Usart<usart2Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using rxPin = Pin<Port<C>, 5>;
using txPin = rxPin;

//using gpsUsart = External::SoftSerial::Usart<Meta::List<rxPin, txPin>, Component::Tcb<0>,
//                                            External::Hal::NullProtocollAdapter,
//                                            AVR::BaudRate<56700>,
//                                            AVR::ReceiveQueueLength<64>, AVR::SendQueueLength<64>,
//                                            etl::NamedFlag<true>, etl::NamedFlag<false>, dbg2>;

template<typename PA>
using sensorUsart = External::SoftSerial::Usart<Meta::List<rxPin, txPin>, Component::Tcb<0>, 
PA, AVR::BaudRate<57600>,
AVR::ReceiveQueueLength<0>,
AVR::SendQueueLength<64>,
etl::NamedFlag<true>,
etl::NamedFlag<false>
//                                            ,dbg1
>;

template<auto N>
struct VoltageProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::Voltage;
    inline static uint32_t value() {
        return ++v;
    }
    static inline uint16_t v{};
};

using vProv1 = VoltageProvider<0>;
using vProv2 = VoltageProvider<1>;

using sensor = External::SPort::Sensor<External::SPort::SensorId::ID3, sensorUsart, systemTimer, 
                    Meta::List<vProv1, vProv2>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0x1e>>; // 1e = temp

using isrRegistrar = IsrRegistrar<typename sensor::uart::StartBitHandler, typename sensor::uart::BitHandler>;

int main() {
#if 1
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    terminalDevice::init<BaudRate<9600>>();
    sensor::init();
    
    systemTimer::init();
    
    led::template dir<Output>();     
    dbg0::template dir<Output>();     
    dbg1::template dir<Output>();     
    dbg2::template dir<Output>();     
    
    led::high();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
    uint8_t counter{};
    while(true) {
        
        etl::Scoped<etl::EnableInterrupt<>> ei;
        
        terminalDevice::periodic();
        sensor::periodic();
        
        systemTimer::periodic([&]{
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
                    etl::outl<terminal>("test16: "_pgm, ++counter);
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

ISR(TCB0_INT_vect) {
    dbg1::high();
    isrRegistrar::isr<AVR::ISR::Tcb<0>::Capture>();
    dbg1::low();
}

