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
#include <mcu/internals/ccl.h>
#include <mcu/internals/event.h>

#include <external/solutions/tick.h>
#include <external/solutions/series01/sppm_in.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 2000_Hz; 
}

template<typename Stepper, typename PPM, typename Timer>
struct GFSM {
    inline static constexpr void init() {
        Stepper::init();
    }
    inline static constexpr void periodic() {
        Stepper::periodic();
        
    }
    inline static constexpr void ratePeriodic() {
        Stepper::ratePeriodic();
    }
private:
    inline static etl::uint_ranged<uint8_t, 1, 8> timer{0};
};

template<typename PWM, typename AN, typename BN>
struct Stepper {
    enum class State : uint8_t {
        Init, A, B, C, D
    };
    enum class Dir : uint8_t {Forward, Backward};
    
    inline static constexpr uint16_t steps = 512;
    inline static constexpr uint16_t period = 1024; // 16KHz
    inline static constexpr uint8_t scaleBits = 5;
    inline static constexpr uint8_t scale = (1 << scaleBits);
//    std::integral_constant<uint8_t, scale>::_;
    inline static constexpr uint16_t maxValue = period - 1;
    
    inline static constexpr uint8_t periodBits = etl::minimumBitsForValue(maxValue);
    inline static constexpr uint8_t maxScaleBits = 16 - periodBits;
    static_assert(maxScaleBits >= scaleBits);

    template<size_t Steps>    
    struct Generator {
        constexpr auto operator()() {
            std::array<uint16_t, Steps> data;
            for(uint16_t i = 0; i < Steps; ++i) {
                data[i] = maxValue * sin((i * M_PI) / steps);
            }
            return data;
        }
    };
    using Sine = AVR::Pgm::Util::Converter<Generator<steps>>::pgm_type;
    
    inline static void init() {
        AN::low();
        BN::low();
        AN::template dir<Output>();
        BN::template dir<Output>();

        PWM::init();
        PWM::period(period);
        PWM::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>>>();
    }  
    inline static void periodic() {
    }
    
    inline static void setDuty() {
        PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((mScale0 * Sine::value(index0)) >> scaleBits);
        PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((mScale1 * Sine::value(index1)) >> scaleBits);
    }
    inline static void ratePeriodic() {
        const auto oldState = mState;
        switch(mState) {
        case State::Init:
            mState = State::A;
            break;
        case State::A:
            setDuty();
            if (index1.isBottom()){
                mState = State::B;
            }
            break;
        case State::B:
            setDuty();
            if (index0.isBottom()){
                mState = State::C;
            }
            break;
        case State::C:
            setDuty();
            if (index1.isBottom()){
                mState = State::D;
            }
            break;
        case State::D:
            setDuty();
            if (index0.isBottom()){
                mState = State::A;
            }            
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Init:
                break;
            case State::A:
                set<Dir::Forward, AN, 0>();
                break;
            case State::B:
                set<Dir::Backward, BN, 1>();
                break;
            case State::C:
                set<Dir::Backward, AN, 0>();
                break;
            case State::D:
                set<Dir::Forward, BN, 1>();
                break;
            }
        }
        ++index0;
        ++index1;
    }  
private:    
    template<auto D, typename P, auto N>
    static inline void set() {
        PWM::onOvlWait([]{
            P::template dir<Input>();
            if constexpr(D == Dir::Forward) {
                PWM::template noinvert<AVR::PWM::WO<N>>();
                P::low();
            }
            else {
                PWM::template invert<AVR::PWM::WO<N>>();
                P::high();
            }
            P::template dir<Output>();
        });
    }
    static inline etl::uint_ranged<uint8_t, 1, scale> mScale0{25};
    static inline etl::uint_ranged<uint8_t, 1, scale> mScale1{scale};

    static inline State mState{State::Init};
    inline static etl::uint_ranged_circular<uint16_t, 0, steps - 1> index0{steps / 2};
    inline static etl::uint_ranged_circular<uint16_t, 0, steps - 1> index1{0};
};

using PortA = Port<A>;
using an = Pin<PortA, 6>; 
using bn = Pin<PortA, 7>; 
using ppmIn = Pin<PortA, 2>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

using pwm = AVR::PWM::DynamicPwm<tcaPosition>;
using stepper = Stepper<pwm, an, bn>;

using sppmPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using evch0 = Event::Channel<0, Event::Generators::Pin<ppmIn>>; 
using ppm_user = Event::Route<evch0, Event::Users::Tcb<0>>;
using sppm_input = External::Ppm::SinglePpmIn<sppmPosition::component_type>;

using evrouter = Event::Router<Event::Channels<evch0>, Event::Routes<ppm_user>>;
using portmux = Portmux::StaticMapper<Meta::List<tcaPosition, sppmPosition>>;

using gfsm = GFSM<stepper, sppm_input, systemTimer>;

int main() {
    portmux::init();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();
    evrouter::init();
    
    gfsm::init();
    
    while(true) {
        gfsm::periodic();
        systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
}

