#define NDEBUG

#define USE_SPORT
//#define USE_IBUS

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/event.h>
#include <mcu/internals/ccl.h>

#include <external/hal/alarmtimer.h>
#include <external/sbus/sbus.h>
#include <external/ibus/ibus.h>
#include <external/sbus/sport.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using ccl0Position = Portmux::Position<Component::Ccl<0>, Portmux::Default>;

using lut0 = Ccl::SimpleLut<0, Ccl::Input::Mask, Ccl::Input::Usart<0>, Ccl::Input::Mask>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, ccl0Position>>;

namespace  {
#ifdef USE_SPORT
    constexpr auto fRtc = 1000_Hz; 
#endif
#ifdef USE_IBUS
    constexpr auto fRtc = 2000_Hz;
#endif
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

struct RawProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::Temp1;
    inline static constexpr auto ibus_type = IBus::Type::type::ARMED;
    inline static constexpr void init() {
    }
    inline static uint16_t value() {
        return ++counter;
    }
    static inline uint16_t counter{};
};

#ifdef USE_SPORT
template<typename PA>
using sensorUsart = AVR::Usart<usart0Position, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
using sport = External::SPort::Sensor<External::SPort::SensorId::ID3, sensorUsart, systemTimer, 
                                       Meta::List<RawProvider>>;
#endif
#ifdef USE_IBUS
using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<RawProvider>, 
                          systemTimer, void>;
#endif
int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);

#ifdef USE_SPORT
    lut0::init(std::byte{0xcc}); // route TXD to lut0-out 
    sport::init();
    sport::uart::txPinDisable();
#endif
#ifdef USE_IBUS
    lut0::init(std::byte{0xff}); // high on lut0-out 
    ibus::init();
    ibus::uart::txOpenDrain();
#endif
    
    while(true) {
#ifdef USE_SPORT
        sport::periodic();    
#endif
#ifdef USE_IBUS
            ibus::periodic();
#endif
        systemTimer::periodic([&]{
#ifdef USE_IBUS
            ibus::ratePeriodic();
#endif
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                }
            });
        });
    }
}

