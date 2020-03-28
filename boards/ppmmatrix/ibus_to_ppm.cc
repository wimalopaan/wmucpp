#define NDEBUG

#define IBUS_PPM
#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/sleep.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/cppm.h>
#include <mcu/pgm/pgmarray.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>
#include <external/ibus/ibus.h>
#include <external/units/music.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>

#include <std/chrono>
#include <etl/output.h>

#ifdef MEM
# include "util/memory.h"
#endif
 
namespace AVR {
    template<typename PinList, typename MCU = DefaultMcuType> struct Multiplexer;
    
    template<typename... Pins, typename MCU>
    struct Multiplexer<Meta::List<Pins...>, MCU> {
        inline static constexpr auto size = sizeof...(Pins);
        
        using index_type = etl::uint_ranged<uint8_t, 0, (1 << size) - 1>;
        
        constexpr inline static void init() {
            (Pins::template dir<AVR::Output>(), ...);
            (Pins::off(), ...);
        }
        
        template<auto... II>
        constexpr inline static void select(index_type number, std::index_sequence<II...>) {
            (((number.toInt() & (1 << II)) ? Pins::on() : Pins::off()), ...);    
        }
        
        constexpr inline static void select(index_type number) {
            select(number, std::make_index_sequence<sizeof...(Pins)>());
        }
    private:
    };
    
    template<typename... CC>
    struct Components {
        inline static constexpr void periodic() {
            (CC::periodic(), ...);    
        }
    };
}

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace Parameter {
    constexpr auto fRtc = 2000_Hz;
}

using selectA0 = Pin<Port<A>, 0>;
using selectA1 = Pin<Port<A>, 7>;
using selectA2 = Pin<Port<A>, 6>;

using multiplexer = AVR::Multiplexer<Meta::List<selectA0, selectA1, selectA2>>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

using cppm = AVR::Cppm<1, AVR::A, 8, multiplexer>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using rcUsart = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

namespace  {
    using namespace std::literals::chrono;
    constexpr auto interval = 10_ms;
//    constexpr auto interval = External::Units::duration_cast<std::chrono::milliseconds>(Hott::hottDelayBetweenBytes);
}


template<typename S1, typename S2>
[[noreturn]] void assertFunction([[maybe_unused]] const S1& expr, [[maybe_unused]] const S2& file, [[maybe_unused]] unsigned int line) {
    while(true) {
    }    
}

using components = AVR::Components<rcUsart, cppm>;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    systemTimer::init();
    cppm::init();
    
    rcUsart::init<AVR::BaudRate<115200>>();
    
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    using channel_t = servo_pa::channel_t;
    using out_t = cppm::index_type;

    {
        etl::Scoped<etl::EnableInterrupt<>> ei;

        etl::uint_ranged_circular<uint8_t, 0, out_t::Upper> index{};
        
        while(true) {
            components::periodic();
            ++index;
            auto v = servo_pa::value(channel_t(10 + index));
            cppm::ppm(index, v);
            systemTimer::periodic([&](){
                alarmTimer::periodic([&](const alarmTimer::index_type timer){
                    if (timer == t) {
                        auto v = servo_pa::value(channel_t{14});
                    }
                });
            });
        }    
    }
}


