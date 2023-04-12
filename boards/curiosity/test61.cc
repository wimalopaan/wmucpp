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
    inline static constexpr uint16_t period= 4000;
    
    inline static constexpr auto sine1 = []{
        std::array<uint16_t, steps> data;
        for(uint16_t i = 0; i < steps; ++i) {
            data[i] = 1.0 * (period - 1) * sin((i * M_PI) / steps);
        }
        return data;
    }();
    inline static constexpr auto sine2 = []{
        std::array<uint16_t, steps> data;
        for(uint16_t i = 0; i < steps; ++i) {
            data[i] = 1.0 * (period - 1) * sin((i * M_PI) / steps);
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
        const auto oldState = mState;
        switch(mState) {
        case State::Init:
            mState = State::B;
            break;
        case State::A:
            setDuty();
            if (index2.isTop()){
                mState = State::B;
            }
            break;
        case State::B:
            setDuty();
            if (index1.isTop()){
                mState = State::C;
            }
            break;
        case State::C:
            setDuty();
            if (index2.isTop()){
                mState = State::D;
            }
            break;
        case State::D:
            setDuty();
            if (index1.isTop()){
                mState = State::A;
            }            
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Init:
                break;
            case State::A:
                forward<AN, 0>();
                break;
            case State::B:
                forward<BN, 1>();
                break;
            case State::C:
                backward<AN, 0>();
                break;
            case State::D:
                backward<BN, 1>();
                break;
            }
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
    inline static etl::uint_ranged_circular<uint16_t, 0, steps - 1> index2{15}; // adaptiv machen
};

template<typename PWM, typename AN, typename AP, typename BN, typename BP>
struct Stepper3 {
    enum class State : uint8_t {
        Init, A, B, C, D
    };

    inline static constexpr uint16_t steps = 128;
    inline static constexpr uint16_t period= 2000;
    
    inline static constexpr auto sine = []{
        std::array<uint16_t, steps> data;
        for(uint16_t i = 0; i < steps; ++i) {
            data[i] = period * sin((i * M_PI) / steps);
        }
        return data;
    }();
    
    inline static void init() {
        AN::template dir<Output>();
        AP::template dir<Output>();
        BN::template dir<Output>();
        BP::template dir<Output>();
        PWM::init();
        PWM::period(period);
        PWM::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>>>();
    }  
    inline static void ratePeriodic() {
        const auto oldState = mState;
        switch(mState) {
        case State::Init:
            mState = State::A;
            break;
        case State::A:
            PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(sine[index1]);
            PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(period - sine[index2]);
            if (phase == ((steps/2) - 1)){
                mState = State::B;
            }
            break;
        case State::B:
            PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(sine[index1]);
            PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(sine[index2]);
            if (phase == ((steps/2) - 1)){
                mState = State::C;
            }
            break;
        case State::C:
            PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(period - sine[index1]);
            PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(sine[index2]);
            if (phase == ((steps/2) - 1)){
                mState = State::D;
            }
            break;
        case State::D:
            PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(period - sine[index1]);
            PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(period - sine[index2]);
            if (phase == ((steps/2) - 1)){
                mState = State::A;
            }            
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Init:
                break;
            case State::A:
                AN::low();
                AP::low();
                BN::high();
                BP::high();
                break;
            case State::B:
                AN::low();
                AP::low();
                BN::low();
                BP::low();
                break;
            case State::C:
                AN::high();
                AP::high();
                BN::low();
                BP::low();
                break;
            case State::D:
                AN::high();
                AP::high();
                BN::high();
                BP::high();
                break;
            }
        }
        ++index1;
        ++index2;
        ++phase;
    }  
    static inline State mState = State::B;
    inline static etl::uint_ranged_circular<uint16_t, 0, steps - 1> index1{steps / 2};
    inline static etl::uint_ranged_circular<uint16_t, 0, steps - 1> index2{0};
    inline static etl::uint_ranged_circular<uint16_t, 0, steps/2 - 1> phase{0};
};


template<typename PWM>
struct Stepper2 {
    inline static constexpr uint16_t steps = 256;
    inline static constexpr uint16_t period= 5000;
    
    inline static constexpr auto sine = []{
        std::array<uint16_t, steps> data;
        
        for(uint16_t i = 0; i < steps; ++i) {
            data[i] = period * (1.0 + sin((i * 2.0 * M_PI) / steps)) / 2.0;
        }
        
        return data;
    }();
    
    inline static void init() {
        PWM::init();
        PWM::period(period);
        PWM::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();
        PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(period / 2);
    }  
    inline static void ratePeriodic() {
        PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(sine[index1]);
        PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(sine[index2]);
        
        ++index1;
        ++index2;
    }  
    
    inline static etl::uint_ranged_circular<uint16_t, 0, sine.size() - 1> index2{0};
    inline static etl::uint_ranged_circular<uint16_t, 0, sine.size() - 1> index1{steps / 4};
};


template<typename AP, typename AN, typename BP, typename BN>
struct Stepper {
    enum class State : uint8_t {
        A, B, C, D
    };
    
    
    static inline void init() {
        AP::template dir<Output>();     
        AN::template dir<Output>();     
        BP::template dir<Output>();     
        BN::template dir<Output>();     
    }
    static inline void ratePeriodic() {
        const auto oldState = mState;
        switch(mState) {
        case State::A:
            mState = State::B;
            break;
        case State::B:
            mState = State::C;
            break;
        case State::C:
            mState = State::D;
            break;
        case State::D:
            mState = State::A;
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::A:
                A1();
                B2();
                break;
            case State::B:
                A2();
                B2();
                break;
            case State::C:
                A2();
                B1();
                break;
            case State::D:
                A1();
                B1();
                break;
            }
            
        }
    }
    
    inline static void A1() {
        AP::high();
        AN::low();
    }
    inline static void A2() {
        AP::low();
        AN::high();
    }
    inline static void B1() {
        BP::high();
        BN::low();
    }
    inline static void B2() {
        BP::low();
        BN::high();
    }
    
    static inline State mState = State::A;
    
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

using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter<>, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

namespace  {
    constexpr auto fRtc = 500_Hz; 
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

