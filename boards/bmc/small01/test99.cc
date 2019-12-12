#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/event.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/ccl.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
//#include <external/hott/sumdprotocolladapter.h>
//#include <external/hott/experimental/sensor.h>
//#include <external/hott/hott.h>
//#include <external/hott/menu.h>

#include <external/solutions/series01/sppm_in.h>

#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

struct Storage final {
    Storage() = delete;
    enum class AVKey : uint8_t {TimeOut = 0, SoftStart, SensorType,
                                _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        etl::uint_NaN<uint8_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<etl::uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

using PortA = Port<A>;
using eflag = Pin<PortA, 2>; 

using ppmIn =  AVR::Pin<PortA, 6>;

using ppmTimerPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using ppm = External::Ppm::SinglePpmIn<Component::Tcb<0>>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using tcdPosition = Portmux::Position<Component::Tcd<0>, Portmux::Default>;
using pwm = PWM::DynamicPwm<tcdPosition>;

namespace Parameter {
    constexpr uint8_t menuLines = 8;
    constexpr auto fRtc = 512_Hz;
}

using systemTimer = SystemTimer<Component::Pit<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 2>;

#ifdef USE_HOTT
#else
using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;
#endif

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0>>;

//using rtc_channel = Event::Channel<0, Event::Generators::PitDiv<1024>>;
using ppm_channel = Event::Channel<0, Event::Generators::Pin<ppmIn>>; 
//using ac_channel = Event::Channel<2, Event::Generators::Ac0<Event::Generators::Kind::Out>>; 
//using userstrobe_channel = Event::Channel<3, void>; 
using ppm_user = Event::Route<ppm_channel, Event::Users::Tcb<0>>;
//using ac_user = Event::Route<ac_channel, Event::Users::Tcb<2>>;
//using userstrobe_user = Event::Route<userstrobe_channel, Event::Users::Tcb<2>>;
using evrouter = Event::Router<Event::Channels<ppm_channel>, Event::Routes<ppm_user>>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcdPosition, ppmTimerPosition>>;

int main() {
#if 0
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    eflag::template dir<Input>();
    eflag::template pullup<true>();
#endif
    
    evrouter::init();
#if 0
    portmux::init();
    eeprom::init();
    systemTimer::init();
    terminalDevice::init<AVR::BaudRate<9600>>();
    
    pwm::init();
    pwm::frequency(5000_Hz);
    pwm::template duty<PWM::WO<0>>(pwm::max() / 10);
    pwm::template on<Meta::List<PWM::WO<0>>>();

    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    uint8_t counter = 0;

#ifndef USE_HOTT
    etl::outl<terminal>("test00"_pgm);
#endif
    
    bool eepSave = false;
    while(true) {
        eepSave |= eeprom::saveIfNeeded();
#ifdef USE_HOTT
#else
        terminalDevice::periodic();
#endif
        systemTimer::periodic([&]{
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    ++counter;
                    if ((counter % 2) == 0) {
#ifndef USE_HOTT
                        etl::outl<terminal>("ppm: "_pgm, ppm::value().toInt());
#endif
                    }
                    else {
                    }
                }
            });
            appData.expire();
        });
    }
#endif
}

