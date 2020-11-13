#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>

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

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

namespace  {
    constexpr auto fRtc = 1000_Hz; 
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

template<typename PA>
using sensorUsart = AVR::Usart<usart0Position, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
using sport = External::SPort::Sensor<External::SPort::SensorId::ID3, sensorUsart, systemTimer, 
                                       Meta::List<RawProvider>>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    systemTimer::init();
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);

    sport::init();
    
    while(true) {
        sport::periodic();    
        systemTimer::periodic([&]{
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                }
            });
        });
    }
}

