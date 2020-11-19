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
#include <external/solutions/rotaryencoder.h>

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
//    constexpr auto fRtc = 71_Hz; // 14ms
    constexpr auto fRtc = 500_Hz; // 14ms
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using sbus = External::SBus::Output::Generator<usart1Position>;

using robo_pa = External::QtRobo::ProtocollAdapter<0>;
using robo = Usart<usart2Position, robo_pa, UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0, 1, 2, 4, 6, 7, 0x1e>>; // 1e = temp

using adi_t = adcController::index_type;

using button0pin = ActiveLow<Pin<Port<D>, 3>, Input>;
using button1pin = ActiveLow<Pin<Port<D>, 5>, Input>;

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
        Meta::visit<buttonList>([]<typename B>(Meta::Wrapper<B>){
                                    using event_t = B::Press;
                                    B::periodic();
                                    constexpr auto i = Meta::index_v<buttonList, B>;
                                    switch(const event_t e = B::event()) {
                                        case event_t::None:
                                        break;
                                        case event_t::Short:
                                        mStates[i] = State::On1;
                                        break;
                                        case event_t::Long:
                                        mStates[i] = State::On2;
                                        break;
                                        case event_t::Release:
                                        mStates[i] = State::Off;
                                        break;
                                    }
                                });
    }
    inline static const auto& states() {
        return mStates;
    }
private:
    inline static std::array<State, sizeof...(BB)> mStates;
};

using buttons = Buttons<button0, button1>;

// 3wege Schalter
//       mit pullup        ohne pullup
// 1)      H                  H
// 2)      L                  L
// 3)      H                  L

using sw0pin = Pin<Port<A>, 2>;

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

using rotary0A = Pin<Port<E>, 0>;
using rotary0B = Pin<Port<E>, 1>;
using rotary1A = Pin<Port<F>, 2>;
using rotary1B = Pin<Port<F>, 3>;

using rot_t = etl::uint_ranged_circular<uint16_t, sbus::sbus_min, sbus::sbus_max>;

using rotary0 = External::RotaryEncoder<rotary0A, rotary0B, rot_t, rot_t{sbus::sbus_mid}>;
using rotary1 = External::RotaryEncoder<rotary1A, rotary1B, rot_t, rot_t{sbus::sbus_mid}>;

uint16_t encode(const auto v, const uint8_t func, const uint8_t module) {
    const uint32_t e = (v + 8 * func + 64 * module); 
    return (e * 1638 + 819) / 1024;
}

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
    
    const auto periodicTimer = alarmTimer::create(300_ms, External::Hal::AlarmFlags::Periodic);
    const auto sbusTimer = alarmTimer::create(14_ms, External::Hal::AlarmFlags::Periodic);
    
    etl::outl<terminal>("rcboard test00"_pgm);
    
    adcController::init();
    
    rotary0::init();
    rotary1::init();
    
    while(true) {
        adcController::periodic();
        terminalDevice::periodic();
        sbus::periodic();
        robo::periodic();
        systemTimer::periodic([&]{
            buttons::ratePeriodic();
            sw0::ratePeriodic();
            
            rotary0::rateProcess();
            rotary1::rateProcess();
            
            robo_pa::whenTargetChanged([](auto t, auto f){
                if (f && (t == robo_pa::Target::Switch)) {
                    const auto index = f.toInt();
                    sbus::output[0] = 172; 
                    sbus::output[0] += encode(robo_pa::switchValues[index], index, 0);
                }
            });            

            alarmTimer::periodic([&](const auto& t){
                if (sbusTimer == t) {
                    sbus::ratePeriodic();
                }
                if (periodicTimer == t) {
                    etl::outl<terminal>("s0: "_pgm, robo_pa::switchValues[0]);
                }
            });
        });
    }
}

