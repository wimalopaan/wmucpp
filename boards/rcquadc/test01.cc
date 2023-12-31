#define NDEBUG

#define USE_IBUS
#define USE_DAISY

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/ibus/ibus.h>

#include <external/solutions/button.h>
#include <external/solutions/analogsensor.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace IBus::Switch {
    template<typename PA, typename ActorList>
    struct GeneralSwitch;
    
    template<typename PA, typename... Actors>
    struct GeneralSwitch<PA, Meta::List<Actors...>> {
        using actor_list = Meta::List<Actors...>;
        using channel_t = PA::channel_t;
        using value_t = PA::value_type;
        
        using protocol_t = Protocol1;
        
        using addr_t = Protocol1::addr_t;
        using index_t = Protocol1::index_t;
        using mode_t = Protocol1::mode_t;
        using param_t = Protocol1::param_t;
        using pvalue_t = Protocol1::pvalue_t;
        
        static inline void init(const channel_t c = 0) {
            mChannel = c;
        }

        static inline void channel(const channel_t c) {
            if (c) {
                mChannel = c;
            }
        }

        static inline void address(const addr_t a) {
            mAddr = a;
        }
        
        template<typename T>
        inline static constexpr bool isLearnCode(const T& v) {
            return Protocol1::isLearnCode(v);
        }
        static inline bool ratePeriodic() {
            const auto cv = PA::valueMapped(mChannel);

            if (!cv) return true;
            
            const addr_t addr = Protocol1::toAddress(cv);
            const index_t index = Protocol1::toIndex(cv);
            const mode_t mode = Protocol1::toMode(cv);
                            
            if (Protocol1::isControlMessage(cv)) { // control
                const param_t param = Protocol1::toParameter(cv);
                const pvalue_t value = Protocol1::toParameterValue(cv);
                
                lpv = value;
                lpp = param;
                
                if (param == Protocol1::broadCast) {
                    if (value == Protocol1::bCastOff) {
                        lastOnIndex = lastindex_t{};
//                        (Actors::moveToPosition(Protocol1::off), ...);
                    }
                }
                else if (lastOnIndex) {
                    mReceivedControl = false;
                    index_t lastOn{lastOnIndex.toInt()};
                    if (param == Protocol1::reset) {
                        if (value == Protocol1::bCastReset) {
                            Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                          A::reset();
                                                      });
                                          
                        }
                    }
                    else if (param == Protocol1::increment) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setPwm(value);
                                                  });
                    }
                    else if (param == Protocol1::position1) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
//                                                      A::setPosition(mode_t{0}, value);
                                                  });
                    }
                    else if (param == Protocol1::position2) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
//                                                      A::setPosition(mode_t{1}, value);
                                                  });
                    }
                    else if (param == Protocol1::position3) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
//                                                      A::setPosition(mode_t{2}, value);
                                                  });
                    }
                    else if (param == Protocol1::position4) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
//                                                      A::setPosition(mode_t{3}, value);
                                                  });
                    }
                    else if (param == Protocol1::passThruChannel) {
                        if ((value >= 1) && (value <= 8)) {
                            const auto pv = PA::valueMapped(0); // channel1                   
                            lc0 = pv;
//                                decltype(pv)::_;
                            Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
//                                                          A::setPosition(mode_t(value - 1), pv);
                                                      });
                        }
                    }
                    else if (param == Protocol1::testMode) {
                        if ((value >= 1) && (value <= 8)) {
                            Meta::visit<actor_list>([&]<typename A>(const Meta::Wrapper<A>&) {
//                                                        A::setTest(mode_t(value - 1));
                                                      });
                        }   
                    }
                }
            }
            else { // command
                mReceivedControl = false;
                if (addr != mAddr) {
                    return false;
                }
                lmv = mode;
                if (mode != Protocol1::off) {
                    lastOnIndex = index.toInt();
                }
                else {
                    lastOnIndex = lastindex_t{};
                }
                Meta::visitAt<actor_list>(index.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
//                                              A::moveToPosition(mode);
                                          });
            }
            return true;                
        }
        inline static bool receivedControl() {
            return mReceivedControl;
        }
        static inline PA::value_type lc0;
        static inline mode_t lmv;
        static inline pvalue_t lpv;
        static inline param_t lpp;
//        private: 
        static inline bool mReceivedControl{false};
        using lastindex_t = etl::uint_ranged_NaN<uint8_t, 0, Meta::size_v<actor_list> - 1>; 
        static inline lastindex_t lastOnIndex;
        static inline channel_t mChannel{14}; // ch 16
        static inline addr_t    mAddr{0};
    };
    
}

namespace  {
#ifdef USE_HOTT
    constexpr auto fRtc = 500_Hz;
#endif
#ifdef USE_SBUS
    constexpr auto fRtc = 1000_Hz;
#endif
#ifdef USE_IBUS
    constexpr auto fRtc = 2000_Hz;
#endif
#ifdef USE_PPM
    constexpr auto fRtc = 128_Hz;
#endif
    
    constexpr uint16_t Ri = 2700;
}

template<typename Sensor>
struct CProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::BAT_CURR;
    inline static void init() {
    }
    inline static uint16_t value() {
        return Sensor::value();
    }
};


template<typename Timer, uint8_t N, typename PWM, typename ADC, typename InA, typename InB, typename PA, typename Term>
struct ChannelFsm {
    using adc_index_t = typename ADC::index_type;
    static inline constexpr adc_index_t adci{N};
    
    using pa_value_t = typename PA::value_type;
    using channel_t = typename PA::channel_t;
    
    static inline constexpr channel_t channel{N};
    
    using pwm_value_t = typename PWM::value_type;
    
    enum class State : uint8_t {Init, Off, Forward, Backward};

    static inline constexpr uint16_t pa_mid = (pa_value_t::Upper + pa_value_t::Lower) / 2;
    static inline constexpr uint16_t pa_half = (pa_value_t::Upper - pa_value_t::Lower) / 2;
    static inline constexpr uint16_t pa_hysterese = 10;
    
    inline static void init() {
        InA::template dir<Output>();
        InA::off();
        InB::template dir<Output>();
        InB::off();
        PWM::template on<N>();
    }
    
    inline static void reset() {
        off();
    }
    
    inline static void setPwm(const auto) {
//        decltype(v)::_;
    }
    
    inline static void periodic() {
    }

    inline static void ratePeriodic() {
        const auto cv = ADC::value(adci);                
        lcv = cv;
        const pa_value_t pv = PA::value(channel);
//        pa_value_t::_;
        lpv = pv.toInt();
        
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Init:
            mState = State::Off;
            break;
        case State::Off:
            if (pv.toInt() > (pa_mid + pa_hysterese)) {
                mState = State::Forward;
            } 
            else if (pv.toInt() < (pa_mid - pa_hysterese)) {
                mState = State::Backward;
            }
            break;
        case State::Forward:
            if (pv.toInt() <= (pa_mid + pa_hysterese)) {
                mState = State::Off;
            }
            else {
                pwm_value_t pvs(((uint32_t)pv.toInt() - pa_mid) * PWM::pwmMax / pa_half); 
                PWM::template pwm<N>(pvs);
            }
            break;
        case State::Backward:
            if (pv.toInt() >= (pa_mid - pa_hysterese)) {
                mState = State::Off;
            } 
            else {
                pwm_value_t pvs(((uint32_t)pa_mid - pv.toInt()) * PWM::pwmMax / pa_half); 
                PWM::template pwm<N>(pvs);
            }
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Init:
                etl::outl<Term>("S i"_pgm);
                break;
            case State::Off:
                etl::outl<Term>("S o"_pgm);
                off();
                break;
            case State::Forward:
                etl::outl<Term>("S f"_pgm);
                forward();
                break;
            case State::Backward:
                etl::outl<Term>("S b"_pgm);
                backward();
                break;
            }
        }
    }
    inline static uint16_t lcv{};
    inline static uint16_t lpv{};
    
private:
    inline static pwm_value_t v0;
    inline static void forward() {
        InA::on();
        InB::off();
    }
    inline static void backward() {
        InA::off();
        InB::on();
    }
    inline static void off() {
        InA::off();
        InB::off();
    }
    inline static External::Tick<Timer> mStateTick;
    inline static State mState{State::Init};
};

template<typename Timer, typename PWM, typename ChList, typename Led, typename Adc, 
         typename Servo, typename Baud, 
         typename Sensor, 
         typename SW, 
         typename Term = void>
struct GlobalFsm;

template<typename Timer, typename PWM, typename Led, typename... Chs, typename Adc, 
         typename Servo, auto baud, 
         typename Sensor, 
         typename SW, 
         typename Term>
struct GlobalFsm<Timer, PWM, Meta::List<Chs...>, Led, Adc, Servo, BaudRate<baud>, Sensor, SW, Term> {
    using channel_list = Meta::List<Chs...>;
    static_assert(Meta::size_v<channel_list> <= 4, "too much channels");
    static_assert(Meta::is_set_v<channel_list>, "channels must be different");

    using adi_t = Adc::index_type;
    
    enum class State : uint8_t {Init, InitWait, Run, Debug1};

    static inline constexpr External::Tick<Timer> initTimeout{500_ms};
    static inline constexpr External::Tick<Timer> debugTimeout{1000_ms};
    
    inline static void init() {
        Led::template dir<Output>();
        Servo::template init<BaudRate<baud>>();
        Sensor::init();
        Adc::init();
        PWM::init();    
        (Chs::init(), ...);
    }
    inline static void periodic() {
        Adc::periodic();
        Servo::periodic();
        Sensor::periodic();
        (Chs::periodic(), ...);
    }
    inline static void ratePeriodic() {
        etl::Scoped([]{Led::on();}, []{Led::off();});
        
        const auto oldState = mState;
        ++mStateTick;
        Sensor::ratePeriodic();
        SW::ratePeriodic();
        (Chs::ratePeriodic(), ...);
        switch(mState) {
        case State::Init:
            mState = State::InitWait;
            break;
        case State::InitWait:
            mStateTick.match(initTimeout, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            mStateTick.match(debugTimeout, []{
                mState = State::Debug1;
            });
            break;
        case State::Debug1:
            mState = State::Run;
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Init:
                break;
            case State::InitWait:
                etl::outl<Term>("rcQ 01"_pgm);
                break;
            case State::Run:
                break;
            case State::Debug1:
                etl::outl<Term>("cv0: "_pgm, Meta::front<channel_list>::lcv, " pv0: "_pgm, Meta::front<channel_list>::lpv);
                break;
            }
        }
    }
private:
    static inline External::Tick<Timer> mStateTick;
    static inline State mState{State::Init};
};

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Servo / DBG
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Sensor

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using pwm = PWM::DynamicPwm8Bit<tcaPosition>;

using portmux = Portmux::StaticMapper<Meta::List<usart1Position, usart2Position>>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<servo>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using ledPin  = Pin<Port<D>, 2>; 

using pwmPin1 = Pin<Port<A>, 0>; 
using pwmPin2 = Pin<Port<A>, 1>; 
using pwmPin3 = Pin<Port<A>, 2>; 
using pwmPin4 = Pin<Port<A>, 3>; 

using end1Pin = Pin<Port<F>, 5>; 
using end2Pin = Pin<Port<D>, 1>; 
using end3Pin = Pin<Port<A>, 4>; 
using end4Pin = Pin<Port<A>, 5>; 

using inA1Pin = Pin<Port<F>, 3>; 
using inB1Pin = Pin<Port<F>, 1>; 
using inA2Pin = Pin<Port<D>, 3>; 
using inB2Pin = Pin<Port<D>, 6>; 
using inA3Pin = Pin<Port<C>, 3>; 
using inB3Pin = Pin<Port<C>, 2>; 
using inA4Pin = Pin<Port<A>, 6>; 
using inB4Pin = Pin<Port<A>, 7>; 

using dgbPin = Pin<Port<C>, 0>; // tx 
using ibusPin = Pin<Port<C>, 1>; // rx 

using csf1Pin = Pin<Port<F>, 2>; // ADC 12
using csf2Pin = Pin<Port<D>, 7>; // ADC 7
using csf3Pin = Pin<Port<D>, 5>; // ADC 5
using csf4Pin = Pin<Port<D>, 0>; // ADC 0 

using sensorUartPin = Pin<Port<F>, 0>; // tx 

using daisyChain= Pin<Port<F>, 4>; 

#ifdef USE_DAISY
struct IBusThrough {
    inline static void init() {
        daisyChain::template dir<Output>();
    }
    inline static void on() {
        daisyChain::on();
    }
    inline static void off() {
        daisyChain::off();
    }
};
using ibt = IBusThrough;
#else
using ibt = void;
#endif

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<12, 7, 5, 0, 0x1e>>; // 1e = temp

using vn1 = External::AnalogSensor<adcController, 0, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;
using vn2 = External::AnalogSensor<adcController, 1, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;
using vn3 = External::AnalogSensor<adcController, 2, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;
using vn4 = External::AnalogSensor<adcController, 3, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;

using cp1 = CProvider<vn1>;
using cp2 = CProvider<vn2>;
using cp3 = CProvider<vn3>;
using cp4 = CProvider<vn4>;

using ch0 = ChannelFsm<systemTimer, 0, pwm, adcController, inA1Pin, inB1Pin, servo_pa, terminal>;
using ch1 = ChannelFsm<systemTimer, 1, pwm, adcController, inA2Pin, inB2Pin, servo_pa, terminal>;
using ch2 = ChannelFsm<systemTimer, 2, pwm, adcController, inA3Pin, inB3Pin, servo_pa, terminal>;
using ch3 = ChannelFsm<systemTimer, 3, pwm, adcController, inA4Pin, inB4Pin, servo_pa, terminal>;

using ibus_sensor = IBus::Sensor<usart2Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<cp1, cp2, cp3, cp4>, systemTimer, ibt
//                          , etl::NamedFlag<true>
//                           , etl::NamedFlag<true>
                          >;

using gswitch = IBus::Switch::GeneralSwitch<servo_pa, Meta::List<ch0, ch1, ch2, ch3>>;

using gfsm = GlobalFsm<systemTimer, pwm, Meta::List<ch0, ch1, ch2, ch3>, ledPin, adcController, servo, BaudRate<115200>, ibus_sensor, gswitch, terminal>;

int main() {
    portmux::init();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();
    gfsm::init();
    
//    adc::nsamples(2);
    
    while(true) {
        gfsm::periodic();
        systemTimer::periodic([&]{
            gfsm::ratePeriodic();
            alarmTimer::periodic([&](const auto&){
            });
        });
    }
}

