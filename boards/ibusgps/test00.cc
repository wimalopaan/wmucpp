#define NDEBUG

#include <mcu/avr.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>

#include <external/hal/alarmtimer.h>
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

using rxPin = Pin<Port<A>, 1>;
using txPin = void;
using dbg1 = Pin<Port<A>, 2>;
using dbg2 = Pin<Port<A>, 3>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using vtg = External::GPS::VTG;
using rmc = External::GPS::RMC;
using gps = External::GPS::GpsProtocollAdapter<0, vtg, rmc>;

using gpsUsart = External::SoftSerial::Usart<Meta::List<rxPin, void>, Component::Tca<0>,
                                            External::GPS::GpsProtocollAdapter<0, vtg, rmc>,
                                            AVR::BaudRate<9600>>;

template<typename VTG>
struct SpeedProvider {
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

using isrRegistrar = IsrRegistrar<typename gpsUsart::StartBitHandler, typename gpsUsart::RxBitHandler>;

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
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    gpsUsart::init();
    ibus::init();
    
    systemTimer::init();
    
    dbg1::template dir<Output>();     
    dbg2::template dir<Output>();     
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    etl::StringBuffer<External::GPS::Sentence::DecimalMaxWidth> s;
    
    while(true) {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        ibus::periodic();
        systemTimer::periodic([&]{
            ibus::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    vtg::speedRaw(s);
                    auto ss = etl::StringConverter<etl::FixedPoint<uint16_t, 4>>::parse<1>(s);
                    speedP::s = ss * 100;
                }
            });
        });
    }
}

ISR(PORTA_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<rxPin::name_type>>();
}

ISR(TCA0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Tca<0>::Ovf>();
}

