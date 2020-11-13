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

template<typename PWM, typename DIR1, typename DIR2>
struct DirPhaseStepper {
    enum class State : uint8_t {
        Init, A, B, C, D
    };

    inline static constexpr uint16_t steps = 128;
    inline static constexpr uint16_t period= 4000; // 10KHz
    
    inline static uint8_t scale = 4; // 1 ... 10
    inline static uint8_t delay = 1;
    inline static uint8_t cycles = 0;
    
    inline static constexpr auto sine1 = []{
        std::array<uint16_t, steps> data;
        for(uint16_t i = 0; i < steps; ++i) {
            data[i] = (double)(period - 1) * sin((i * M_PI) / steps);
        }
        return data;
    }();
//    inline static constexpr auto sine2 = []{
//        std::array<uint16_t, steps> data;
//        for(uint16_t i = 0; i < steps; ++i) {
//            data[i] = (double)(period - 1) * sin((i * M_PI) / steps);
//        }
//        return data;
//    }();
    
    inline static constexpr auto max1 = []{
        auto max = sine1[0];
        for(const auto v : sine1) {
            if (v > max) max = v;
        }
        return max;
    }();
    
//    std::integral_constant<uint16_t, max1>::_;
    
    inline static void init() {
        DIR1::low();
        DIR2::low();
        DIR1::template dir<Output>();
        DIR2::template dir<Output>();
        PWM::init();
        PWM::period(period);
        PWM::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>>>();
    }  
    inline static void setDuty() {
        PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((scale * sine1[index1]) / 10);
        PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((scale * sine1[index2]) / 10);
    }
    inline static void ratePeriodic() {
        if (delay != 0) {
            ++cycles;
            if (cycles >= delay) {
                cycles = 0;
                step();
            }
        } 
        else {
            PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(0);
            PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(0);
        }
    }
    
    inline static void step() {
        switch(mState) {
        case State::Init:
            mState = State::B;
            break;
        case State::A:
            setDuty();
            if (index1.isBottom()) {
                DIR1::high();
            }
            if (index2.isTop()){
                mState = State::B;
            }
            break;
        case State::B:
            setDuty();
            if (index2.isBottom()) {
                DIR2::high();
            }
            if (index1.isTop()){
                mState = State::C;
            }
            break;
        case State::C:
            setDuty();
            if (index1.isBottom()) {
                DIR1::low();
            }
            if (index2.isTop()){
                mState = State::D;
            }
            break;
        case State::D:
            setDuty();
            if (index2.isBottom()) {
                DIR2::low();
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
    static inline State mState = State::B;
    inline static etl::uint_ranged_circular<uint16_t, 0, steps - 1> index1{(steps / 2) + 5};
    inline static etl::uint_ranged_circular<uint16_t, 0, steps - 1> index2{0};
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


using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;
using terminal = etl::basic_ostream<servo>;

namespace  {
    constexpr auto fRtc = 100_Hz; 
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using pwm = AVR::PWM::DynamicPwm<tcaPosition>;

using stepper = DirPhaseStepper<pwm, an, bn>;

int main() {
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    servo::init<AVR::BaudRate<115200>, HalfDuplex>();
    servo::txEnable<false>();

    systemTimer::init();
    
    stepper::init();

    using ch_t = servo_pa::value_type;
    
    while(true) {
        servo::periodic();        
        systemTimer::periodic([&]{
            const ch_t ch0 = servo_pa::value(0);
            if (ch0) {
                const uint16_t rv = ch0.toInt();
                if (rv > ((ch_t::Upper + ch_t::Lower) / 2)) {
                    const uint8_t x = rv / 2;
                    stepper::delay = (255 - x);
                }
                stepper::ratePeriodic();
            }
            else {
                stepper::delay = 0;
            }
        });
    }
}

