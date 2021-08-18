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
#include <mcu/internals/adc.h>

#include <external/hal/adccontroller.h>
#include <external/solutions/tick.h>
#include <external/solutions/series01/sppm_in.h>

#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 8000_Hz; 
}

template<typename Stepper, typename ADC, typename PPM, typename Timer, typename TDev>
struct GFSM {
    using terminal = etl::basic_ostream<TDev>;
    using adc_i_t = ADC::index_type;
    
    inline static constexpr External::Tick<Timer> debugTicks{1000_ms}; 
    
    using speed_type = etl::uint_ranged<uint8_t, 1, 8>;
    
    inline static constexpr void init() {
        Stepper::init();
        PPM::init();
        ADC::init();
        if constexpr(!std::is_same_v<TDev, void>) {
            TDev::template init<AVR::BaudRate<115200>>();
        }
    }
    inline static constexpr void periodic() {
        Stepper::periodic();
        PPM::periodic();
        ADC::periodic();
        if constexpr(!std::is_same_v<TDev, void>) {
            TDev::periodic();
        }
    }
    inline static constexpr void ratePeriodic() {
        ++speedDivider;
        if (speedDivider == speed) {
            Stepper::ratePeriodic();
            speedDivider = 0;
        }
        
        const auto balance = ADC::value(adc_i_t{0});
        
        ++stateTick;
        stateTick.on(debugTicks, [&]{
//           etl::outl<terminal>("bal: "_pgm, balance, " s0: "_pgm, Stepper::mScale0, " s1: "_pgm, Stepper::mScale1, " ds: "_pgm, Stepper::deltaShift); 
           etl::outl<terminal>(" i0: "_pgm, Stepper::index0.toInt(), " i1: "_pgm, Stepper::index1.toInt(), " ds: "_pgm, Stepper::deltaShift); 
           
           speed = etl::scaleTo<speed_type>(balance);
        });
        
//        Stepper::balance(balance);
//        Stepper::shift(balance);
    }
private:
    inline static speed_type speed{1};
    inline static uint8_t speedDivider{0};
    inline static External::Tick<Timer> stateTick{}; 
};

template<typename PWM, typename AN, typename BN, etl::Concepts::NamedFlag Invert, typename Lut0, typename Lut1>
struct Stepper {
    enum class State : uint8_t {
        Init, A, B, C, D
    };
    enum class Pol : uint8_t {Positiv, Negativ};
    enum class Rot : uint8_t {Forward, Backward};

    inline static constexpr bool invert = Invert::value;
    
    inline static constexpr uint16_t steps = 512;
    static_assert(etl::isPowerof2(steps));
    
    inline static constexpr uint16_t period = 1024; // 16KHz
    inline static constexpr uint8_t scaleBits = 5;
    inline static constexpr uint8_t scale = (1 << scaleBits);
//    std::integral_constant<uint8_t, scale>::_;
    inline static constexpr uint16_t maxPeriodValue = period - 1;
    
    inline static constexpr uint8_t periodBits = etl::minimumBitsForValue(maxPeriodValue);
    inline static constexpr uint8_t maxScaleBits = 16 - periodBits;
    static_assert(maxScaleBits >= scaleBits);
    
    using scale_type = etl::uint_ranged<uint8_t, 1, scale>;
    using index_type = etl::uint_ranged_circular<uint16_t, 0, steps - 1> ;
    
    template<size_t Steps>    
    struct Generator {
        constexpr auto operator()() {
            std::array<uint16_t, Steps> data;
            for(uint16_t i = 0; i < Steps; ++i) {
                data[i] = maxPeriodValue * sin((i * M_PI) / steps);
            }
            return data;
        }
    };
    using Sine = AVR::Pgm::Util::Converter<Generator<steps>>::pgm_type;
    
    inline static void init() {
        AN::template dir<Output>();
        BN::template dir<Output>();
        AN::off();
        BN::off();

        PWM::init();
        PWM::period(period);
        PWM::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>>>();
    }  
    inline static void periodic() {
    }

    inline static void direction(const Rot d) {
        mDirection = d;
    }
    
    template<typename T, auto L, auto U>
    static inline void shift(const etl::uint_ranged<T, L, U>& b) {
        constexpr T mid = b.mid();
        constexpr T delta = (U - L) / 2;
        
        if (b > mid) {
            const T d = b - mid;            
            const T x = etl::scale(d, etl::Intervall{0u, delta + 1}, etl::Intervall{0u, steps/8});
            deltaShift = x;
        }
        else if (b < mid) {
            const T d = mid - b;            
            const T x = etl::scale(d, etl::Intervall{0u, delta + 1}, etl::Intervall{0u, steps/8});
            deltaShift = -x;
        }
        else {
            deltaShift = 0;
        }   
    }
    
    template<typename T, auto L, auto U>
    static inline void balance(const etl::uint_ranged<T, L, U>& b) {
        constexpr T mid = b.mid();
        constexpr T delta = (U - L) / 2;
        
        if (b > mid) {
            const T d = b - mid;            
            const T x = etl::scale(d, etl::Intervall{0u, delta}, etl::Intervall{0u, 16u});
            mScale0.set(scale - x);
            mScale1.setToTop();
        }
        else if (b < mid) {
            const T d = mid - b;            
            const T x = etl::scale(d, etl::Intervall{0u, delta}, etl::Intervall{0u, 16u});
            mScale0.setToTop();
            mScale1.set(scale - x);
        }
        else {
            mScale0.setToTop();
            mScale1.setToTop();
        }
    }
    
    inline static void setDuty() {
        index0 = index_type((index1.toInt() + steps / 2 + deltaShift));
        
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
            if (mDirection == Rot::Forward) {
                switch(mState) {
                case State::Init:
                    break;
                case State::A:
                    set<Pol::Positiv, AN, 0>();
                    break;
                case State::B:
                    set<Pol::Negativ, BN, 1>();
                    break;
                case State::C:
                    set<Pol::Negativ, AN, 0>();
                    break;
                case State::D:
                    set<Pol::Positiv, BN, 1>();
                    break;
                }
            }
            else {
                switch(mState) {
                case State::Init:
                    break;
                case State::A:
                    set<Pol::Negativ, AN, 0>();
                    break;
                case State::B:
                    set<Pol::Negativ, BN, 1>();
                    break;
                case State::C:
                    set<Pol::Positiv, AN, 0>();
                    break;
                case State::D:
                    set<Pol::Positiv, BN, 1>();
                    break;
                }
            }
        }
        ++index1;
    }  
//private:    
    template<auto D, typename P, auto N>
    static inline void set() {
        PWM::onOvlWait([]{
            P::template dir<Input>();
            if constexpr(D == Pol::Positiv) {
                if constexpr(N == 0) {
                    Lut0::init(0xaa_B);
                }
                else {
                    Lut1::init(0xcc_B);
                }
                PWM::template noinvert<AVR::PWM::WO<N>>();
                P::off();
            }
            else {
                if constexpr(invert) {
                    if constexpr(N == 0) {
                        Lut0::init(~0xaa_B);
                    }
                    else {
                        Lut1::init(~0xcc_B);
                    }
                    PWM::template invert<AVR::PWM::WO<N>>();
                }
                P::on();
            }
            P::template dir<Output>();
        });
    }
    static inline scale_type mScale0{scale};
    static inline scale_type mScale1{scale};

    static inline State mState{State::Init};
    static inline int16_t deltaShift{0};
    
    inline static index_type index0{steps / 2};
    inline static index_type index1{0};
    
    inline static Rot mDirection{Rot::Backward};
};

// tca:wo0 = pb0
// lut0    = pa4
// tca:wo1 = pb1
// lut1    = pa7

// an0      = pa1
// an1      = pa2
// bn0      = pa5
// bn1      = pa6

// ppm      = pb2

using PortA = Port<A>;
using PortB = Port<B>;

using an0 = Pin<PortA, 1>; 
using an1 = Pin<PortA, 2>; 
using gra = PinGroup<Meta::List<an0, an1>>;

using bn0 = Pin<PortA, 5>; 
using bn1 = Pin<PortA, 6>; 
using grb = PinGroup<Meta::List<bn0, bn1>>;

using ppmIn = Pin<PortB, 2>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<3>>;
using adc_i_t = adcController::index_type;

using lut0 = Ccl::SimpleLut<0, Ccl::Input::Tca0<0>, Ccl::Input::Mask,Ccl::Input::Mask>;
using lut1 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Tca0<1>,Ccl::Input::Mask>;

using pwm = AVR::PWM::DynamicPwm<tcaPosition>;
using stepper = Stepper<pwm, gra, grb, etl::NamedFlag<true>, lut0, lut1>;
//using stepper = Stepper<pwm, gra, grb, etl::NamedFlag<false>, lut0, lut1>;

using sppmPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using evch1 = Event::Channel<1, Event::Generators::Pin<ppmIn>>; 
using ppm_user = Event::Route<evch1, Event::Users::Tcb<0>>;
using sppm_input = External::Ppm::SinglePpmIn<sppmPosition::component_type>;

using evrouter = Event::Router<Event::Channels<evch1>, Event::Routes<ppm_user>>;
using portmux = Portmux::StaticMapper<Meta::List<tcaPosition, sppmPosition>>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>; // Sensor
//using term_dev = Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
using term_dev = void;

using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;



using gfsm = GFSM<stepper, adcController, sppm_input, systemTimer, term_dev>;

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

