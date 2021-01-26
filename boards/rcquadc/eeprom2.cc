// HW03: Fehler auf Platine: Brücke inB1 -> PD0 (Pin10)
// HW03: Fehler auf Platine: AnalogSwitches

//#define NDEBUG

//#define USE_IBUS
#define USE_SBUS

#ifdef USE_SBUS
# define USE_SPORT
#endif

#define SCALE_ADC_PWM

#define LEARN_DOWN

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/syscfg.h>
#include <mcu/internals/ccl.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 1000_Hz;
}


using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Servo / DBG
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Sensor

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using portmux = Portmux::StaticMapper<Meta::List<usart1Position, usart2Position>>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;

using servo = Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

using terminal = etl::basic_ostream<servo>;

using nvmctrl = AVR::NvmCtrl<>;

int main() {
    portmux::init();
    ccp::unlock([]{
//        clock::prescale<1>();
        clock::init<Project::Config::fMcuMhz>();
    });

    servo::init<BaudRate<115200>>();
    
//    eeprom::init();
    
    systemTimer::init();

    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    const auto eTimer = alarmTimer::create(10000_ms, External::Hal::AlarmFlags::Periodic);
    
    {
        while(true) {
            servo::periodic();
            
//            if (!eeprom::saveIfNeeded()) {
//                etl::outl<terminal>("s"_pgm);
//            }
            
//            eeprom::saveIfNeeded([]{
//                etl::outl<terminal>("save"_pgm);
//            });
            systemTimer::periodic([&]{
                alarmTimer::periodic([&](const auto& t){
                    if (t == periodicTimer) {
                        const std::byte v = nvmctrl::read_eeprom(0);
//                        const uint8_t v = eeprom_read_byte((uint8_t*)0x1400);
                        etl::outl<terminal>("exp: "_pgm, v);
//                        etl::outl<terminal>("exp: "_pgm, eeprom::data().magic());
//                        eeprom::data().expire();
                    }
                    if (t == eTimer) {
                        if (nvmctrl::eeprom_ready()) {
                            nvmctrl::write_eeprom(std::byte{42}, 0);
//                            eeprom_update_byte((uint8_t*)0x1400, 42);
                            etl::outl<terminal>("set"_pgm);
                        }
//                        eeprom::data().magic() = 42;
//                        eeprom::data().change();
                        
                    }
                });
                                
            });
        }
    }
}


#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
