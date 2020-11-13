#define NDEBUG

#include <math.h>

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/pwm.h>

#include <external/hal/alarmtimer.h>
#include <external/sbus/sbus.h>
#include <external/ibus/ibus.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

template<typename PWM, typename AN, typename BN>
struct Stepper4 {
    enum class State : uint8_t {
        Init, A, B, C, D
    };

    inline static constexpr uint16_t steps = 128;
    inline static constexpr uint16_t period= 4000; // 10KHz
    
    inline static constexpr auto sine1 = []{
        std::array<uint16_t, steps> data;
        for(uint16_t i = 0; i < steps; ++i) {
            data[i] = 1.0 * (double)(period - 1) * sin((i * M_PI) / steps);
        }
        return data;
    }();
    inline static constexpr auto sine2 = []{
        std::array<uint16_t, steps> data;
        for(uint16_t i = 0; i < steps; ++i) {
            data[i] = 1.0 * (double)(period - 1) * sin((i * M_PI) / steps);
        }
        return data;
    }();
    
    inline static void init() {
        AN::low();
        BN::low();
        AN::template dir<Output>();
        BN::template dir<Output>();
        PWM::init();
        PWM::period(period);
        PWM::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>>>();
    }  
    inline static void setDuty() {
        PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(sine1[index1]);
        PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(sine2[index2]);
    }
    inline static void ratePeriodic() {
        switch(mState) {
        case State::Init:
            mState = State::B;
            break;
        case State::A:
            setDuty();
            if (index1.isBottom()) {
                forward<AN, 0>();
            }
            if (index2.isTop()){
                mState = State::B;
            }
            break;
        case State::B:
            setDuty();
            if (index2.isBottom()) {
                forward<BN, 1>();
            }
            if (index1.isTop()){
                mState = State::C;
            }
            break;
        case State::C:
            setDuty();
            if (index1.isBottom()) {
                backward<AN, 0>();
            }
            if (index2.isTop()){
                mState = State::D;
            }
            break;
        case State::D:
            setDuty();
            if (index2.isBottom()) {
                backward<BN, 1>();
            }
            if (index1.isTop()){
                mState = State::A;
            }            
            break;
        }
        ++index1;
        ++index2;
    }  
private:    
    template<typename P, auto N>
    static inline void forward() {
        P::template dir<Input>();
        P::low();
        PWM::template noinvert<AVR::PWM::WO<N>>();
        P::template dir<Output>();
    }
    template<typename P, auto N>
    static inline void backward() {
        P::template dir<Input>();
        P::high();
        PWM::template invert<AVR::PWM::WO<N>>();
        P::template dir<Output>();
    }
    static inline State mState = State::B;
    inline static etl::uint_ranged_circular<uint16_t, 0, steps - 1> index1{steps / 2};
    inline static etl::uint_ranged_circular<uint16_t, 0, steps - 1> index2{5};
};

using PortA = Port<A>;
using PortF = Port<F>;
using PortE = Port<E>;

using ap = Pin<PortA, 4>; // Pin 2
using an = Pin<PortA, 5>; // Pin 3 
using bp = Pin<PortA, 6>; // 4 
using bn = Pin<PortA, 7>; // 5

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

namespace  {
    constexpr auto fRtc = 100_Hz; 
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using pwm = AVR::PWM::DynamicPwm<tcaPosition>;

//using stepper = Stepper<ap, an, bp, bn>;
//using stepper = Stepper2<pwm>;
//using stepper = Stepper3<pwm, an, ap, bn, bp>;
using stepper = Stepper4<pwm, an, bn>;

int main() {
//    uint8_t counter = 0;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    terminalDevice::init<BaudRate<9600>>();

    systemTimer::init();
    
    stepper::init();
    
//    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        terminalDevice::periodic();        
        systemTimer::periodic([&]{
            stepper::ratePeriodic();
//            alarmTimer::periodic([&](const auto& t){
//                if (periodicTimer == t) {
////                    etl::outl<terminal>("c: "_pgm, ++counter);
//                }
//            });
        });
    }
}

