#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>

#include <external/solutions/button.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>;

//using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltB>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

namespace  {
    constexpr auto fRtc = 71_Hz; // 14ms
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using sbus = External::SBus::Output::Generator<usart1Position>;

using robo_pa = External::RoboRemo::ProtocollAdapter<0>;
using robo = Usart<usart2Position, robo_pa, UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0, 0x1e>>; // 1e = temp

using adi_t = adcController::index_type;

using button0pin = ActiveLow<Pin<Port<A>, 0>, Input>;
using button1pin = ActiveLow<Pin<Port<A>, 1>, Input>;

using button0 = External::Button<button0pin, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(300_ms)>;
using button1 = External::Button<button1pin, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(300_ms)>;

template<typename... BB>
struct Buttons {
    enum class State : uint8_t {Off, On1, On2};
    
    using buttonList = Meta::List<BB...>;
    
    inline static void init() {
        (BB::init(), ...);
    }
    inline static void ratePeriodic() {
        (BB::periodic(), ...);
    }
    
private:
    std::array<State, sizeof...(BB)> mStates;
};

using buttons = Buttons<button0, button1>;

// 3wege Schalter
//       mit pullup        ohne pullup
// 1)      H                  H
// 2)      L                  L
// 3)      H                  L

using sw0pin = Pin<Port<A>, 0>;

template<typename Pin>
struct Switch3State {
    enum class State : uint8_t {Low, Mid, High};
    inline static void init() {
        Pin::template dir<Input>();
    }
    inline static void ratePeriodic() {
        Pin::template pullup<true>();
        bool v1 = Pin::read();
        Pin::template pullup<false>();
        bool v2 = Pin::read();
        
        if (v1 && v2) {
            mState = State::High;
        }
        else if (!(v1 || v2)) {
            mState = State::Low;
        }
        else {
            mState = State::Mid;
        }
    }
private:
    inline static State mState{State::Mid};
};

using sw0 = Switch3State<sw0pin>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    terminalDevice::init<BaudRate<9600>>();
    robo::init<BaudRate<9600>>();    
    sbus::init();
    
    buttons::init();
    
    sw0::init();
    
    systemTimer::init();
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);
    etl::outl<terminal>("rcboard test00"_pgm);

    while(true) {
        adcController::periodic();
        terminalDevice::periodic();
        sbus::periodic();
        robo::periodic();
        systemTimer::periodic([&]{
            buttons::ratePeriodic();
            sw0::ratePeriodic();
            
            sbus::ratePeriodic();
            
            const uint16_t v0 = adcController::value(adi_t{0}).toInt();
            sbus::output[0] = (v0 - 512) + 992;
            
            const uint16_t v1 = robo_pa::propValues[0];
            sbus::output[1] = v1 + 992;
            
            
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::outl<terminal>("c: "_pgm);
                }
            });
        });
    }
}

