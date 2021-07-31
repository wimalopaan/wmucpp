#define NDEBUG

#define LEARN_DOWN

#ifndef GITMAJOR
# define VERSION_NUMBER 0001
#endif

#ifndef NDEBUG
static unsigned int assertKey{1234};
#endif

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
#include <mcu/internals/spi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>
#include <mcu/internals/syscfg.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/apa102.h>
#include <external/solutions/series01/sppm_in.h>
#include <external/solutions/rc/busscan.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;
 
#ifndef NDEBUG
namespace xassert {
    etl::StringBuffer<160> ab;
    etl::StringBuffer<10> aline;
    bool on{false};
}
#endif

namespace  {
    constexpr auto fRtc = 1000_Hz;
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;

using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>; // terminal
//using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Servo (in / out)
//using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Alt1>; // Sensor

//using servoPosition = usart1Position; 
//using scanDevPosition = servoPosition;

//using sensorPosition = usart2Position; // Sensor

using scanTermPosition = usart0Position;
using scan_term_dev = Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
using TermDev = scan_term_dev;
using terminal  = etl::basic_ostream<TermDev>;

using dbgPin = Pin<Port<A>, 3>; 

//using daisyChain= Pin<Port<F>, 5>; 

//using pwmInPin = Pin<Port<C>, 3>;

//using WPin = Pin<Port<D>, 0>;
//using IPin = WPin;
//using VPin = Pin<Port<D>, 1>;
//using BPin = VPin;
//using UPin = Pin<Port<D>, 2>;
//using APin = UPin;

using ledPin = Pin<Port<D>, 6>;
using led = ActiveHigh<ledPin, Output>;

using enablePin = Pin<Port<D>, 7>;
using enable = ActiveHigh<enablePin, Output>;

//using scanLedPin = ActiveHigh<ledPin, Output>;

// lut3 out: pf3
//using ccl3Position = Portmux::Position<Component::Ccl<3>, Portmux::Default>;
//using lut3 = Ccl::SimpleLut<3, Ccl::Input::Mask, Ccl::Input::Mask,Ccl::Input::Usart<2>>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltF>;
using pwm = External::PWM::DynamicPwm<tcaPosition>;

//using adc = Adc<Component::Adc<0>, AVR::Resolution<12>, Vref::V2_048>; // 3 = Strom, 4 = VBatt, 5 = Temp
//using adcController = External::Hal::AdcController<adc, Meta::NList<3, 4, 5, 0x42>>; // 0x42 = temp
//using adc_i_t = adcController::index_type;

//using ppmDevPosition = void;

//    using evrouter = Event::Router<Event::Channels<>, Event::Routes<>>;
//using evrouter = void;

//    using eeprom = EEProm::Controller<Data>;

//using portmux = Portmux::StaticMapper<Meta::List<ccl3Position, tcaPosition, servoPosition, sensorPosition, scanTermPosition>>;
using portmux = Portmux::StaticMapper<Meta::List<tcaPosition, scanTermPosition>>;


struct GlobalFsm {
   
    using Timer = systemTimer;

    enum class State : uint8_t {Undefined, Init, Enable};
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> enableTicks{1000_ms};
    static constexpr External::Tick<Timer> debugTicks{1000_ms};
    
    static inline void init() {
        TermDev::init<AVR::BaudRate<115200>>();
        
        enable::init();

        pwm::init();
        
        dbgPin::template dir<Output>();
    }
    
    static inline void periodic() {
        TermDev::periodic();
    }

    static inline void ratePeriodic() {
        dbgPin::toggle();
        
        const auto oldState = mState;
        ++mStateTick;
        
        (++mDebugTick).on(debugTicks, []{
            etl::outl<terminal>("*"_pgm);
        });
        
        switch(mState) {
        case State::Undefined:
            mStateTick.on(startupTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(enableTicks, []{
                mState = State::Enable;
            });
            break;
        case State::Enable:
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("S: Init"_pgm);
                break;
            case State::Enable:
                etl::outl<terminal>("S: Enable"_pgm);
                enable::activate();
                pwm::period(20000);
                pwm::template duty<Meta::List<AVR::PWM::WO<2>>>(1000);
                pwm::template on<Meta::List<AVR::PWM::WO<2>>>();
                break;
            }
        }
    }
private:
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mDebugTick;
    
};

using gfsm = GlobalFsm;

int main() {
    portmux::init();
    
//    PORTMUX.TCAROUTEA = 0x05;
    
    ccp::unlock([]{
        clock::init<Project::Config::fMcuMhz>();
    });
    systemTimer::init();
    
    gfsm::init();
    
    {
        etl::outl<terminal>("test01"_pgm);
        
        while(true) {
            gfsm::periodic();
            systemTimer::periodic([&]{
                gfsm::ratePeriodic();
            });
        }
    }
}
