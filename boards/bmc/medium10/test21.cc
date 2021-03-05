//#define NDEBUG

#define LEARN_DOWN

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
#include <external/hott/experimental/adapter.h>
#include <external/hott/experimental/sensor.h>

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

namespace  {
    constexpr auto fRtc = 1000_Hz;

    constexpr uint16_t R1vd = 10'000;
    constexpr uint16_t R2vd = 1'200;
}

struct Data final : public EEProm::DataBase<Data> {
    uint8_t mMagic{};
    
    enum class Param : uint8_t {Dead, ThrD1, ThrD2, PwmMax, PwmMin, KickThr, KickTicks, PwmLow, _Number};
    
    using pa_t = std::array<uint16_t, (uint8_t)Param::_Number>;
    using index_t = etl::index_type_t<pa_t>;
    using cyclic_t = etl::cyclic_type_t<pa_t>;

    uint16_t& param(const index_t i) {
        return mParameter[i.toInt()];
    }
    uint16_t& param(const Param i) {
        return mParameter[(uint8_t)i];
    }
private:
    pa_t mParameter{};
};

template<typename L>
struct ColorLedAdapter {
    using index_t = L::index_type;
    static inline void init() {
        L::init();
    }
    static inline void activate() {
        L::set(index_t{0}, mColor);
        L::out();
    }
    static inline void inactivate() {
        L::set(index_t{0}, External::cRGB{});
        L::out();        
    }
    static inline void periodic() {
        L::periodic();
    }    
    static inline void color(const External::Crgb& c) {
        mColor = c;
    }
private:
    static constexpr External::Crgb red{External::Red{128}, External::Green{0}, External::Blue{0}};
    inline static External::Crgb mColor{red};
};


template<typename Pin>
struct IBusThrough {
    inline static void init() {
        Pin::template dir<Output>();
    }
    inline static void on() {
        Pin::on();
    }
    inline static void off() {
        Pin::off();
    }
};

template<typename Timer, typename ValueType> 
struct TimeRegulator {
    using value_type = ValueType;
    using ranged_type = ValueType::ranged_type;
    using raw_type = ValueType::value_type;
    
    static constexpr raw_type halfSpan = (value_type::Upper - value_type::Lower) / 2;
    
    static constexpr External::Tick<Timer> deltaTicks{20_ms}; 

    static inline raw_type value(const value_type& v) {
        if (v) {
            mTarget = v.toInt();
        }
        return mActual;
    }
    
    static inline void ratePeriodic() {
        (++deltaTick).on(deltaTicks, []{
            if (mTarget > mActual) {
                const uint16_t d = std::min(mMaxDiff, mTarget - mActual);
                mActual += d;            
            }
            else if (mTarget < mActual) {
                const uint16_t d = std::min(mMaxDiff, mActual - mTarget);
                mActual -= d;            
            }
        });
    }
    static inline void off(const raw_type& ov) {
        mActual = ov;
        mTarget = ov;
    }
    static inline void slow(bool on) {
        if (on) {
            mMaxDiff = std::max(raw_type{2}, (halfSpan / 300));
        }
        else {
            mMaxDiff = std::max(raw_type{10}, (halfSpan / 40));
        }
    }
private:
    static inline External::Tick<Timer> deltaTick;
    static inline raw_type mMaxDiff{4};
    static inline raw_type mTarget{};
    static inline raw_type mActual{};
};

template<typename Timer, typename PWM, typename InhPinList, typename PA, typename NVM>
struct EscFsm {
    
    using Param = NVM::data_t::Param;
    
    enum class State : uint8_t {Undefined = 0, Init, 
                                Off = 10, OffForwardWait, OffBackwardWait, 
                                Forward = 20, ForwardWait, ForwardKick,  
                                Backward = 30, BackwardWait};
    
    enum class RunState : uint8_t {Undefined = 0, Off, Low, Medium, High};
    
    enum class Event : uint8_t {None, Start, Reset};
    
    using throttle_type = typename PA::value_type;
//    throttle_type::_;
    using thr_t = typename throttle_type::value_type;
//        thr_t::_;
    using timeRegulator  = TimeRegulator<Timer, throttle_type>;
    
    using pwm_t = PWM::value_type;
    //    pwm_t::_;
    
    static inline constexpr thr_t thrMax{throttle_type::Upper};
    static inline constexpr thr_t thrMin{throttle_type::Lower};
    static inline constexpr thr_t thrMedium{(throttle_type::Upper + throttle_type::Lower) / 2};
    static inline constexpr thr_t thrHalf{(throttle_type::Upper - throttle_type::Lower) / 2};
    
    static inline thr_t thrStartForward() {
        return thrMedium + NVM::data().param(Param::Dead);
    };
    static inline thr_t thrStartBackward() {
        return thrMedium - NVM::data().param(Param::Dead);
    };

    inline static thr_t throttle_delta_1() {
        return NVM::data().param(Param::ThrD1);
    };
    inline static thr_t throttle_delta_2() {
        return NVM::data().param(Param::ThrD2);
    };

    inline static PWM::value_type mPwmPeriodMin() {
        return NVM::data().param(Param::PwmMin);
    };  
    inline static PWM::value_type mPwmPeriodMax() {
        return NVM::data().param(Param::PwmMax);
    };  
    
//    static inline constexpr thr_t thrStartForward3{thrMedium  + (thrHalf / 10)};
    
    static inline thr_t thrForwardKick() {
        return thrMedium  + NVM::data().param(Param::KickThr);
    };
    static inline uint16_t thrForwardKickTisck() {
        return NVM::data().param(Param::KickTicks);
    };

    static inline constexpr thr_t deadStep = thrHalf / 40;
    static inline void dead(const etl::uint_ranged<uint8_t, 0, 9>& v) {
        NVM::data().param(Param::Dead) = deadStep * (v + 1);               
        NVM::data().change();
    }

    static inline constexpr uint16_t pwmStep1 = 1800;
    static inline void pwmFreq1(const etl::uint_ranged<uint8_t, 0, 9>& v) {
        NVM::data().param(Param::PwmMin) = (10 - v ) * pwmStep1;
        NVM::data().change();
    }
    
    static inline constexpr uint16_t pwmStep2 = 5281;
    static inline void pwmFreq2(const etl::uint_ranged<uint8_t, 0, 9>& v) {
        NVM::data().param(Param::PwmMax) = 65535 - v * pwmStep2;    
        NVM::data().change();
    }

    static inline void thr1(const etl::uint_ranged<uint8_t, 0, 9>& v) {
        NVM::data().param(Param::ThrD1) = (thrHalf * v) / 9;    
        NVM::data().param(Param::ThrD2) = std::max(NVM::data().param(Param::ThrD1) + 1, NVM::data().param(Param::ThrD2));    
        NVM::data().change();
    }
    static inline void thr2(const etl::uint_ranged<uint8_t, 0, 9>& v) {
        thr_t nv = (thrHalf * v) / 9;    
        NVM::data().param(Param::ThrD2) = std::max(NVM::data().param(Param::ThrD1) + 1, nv);    
        NVM::data().change();
    }
    static inline constexpr thr_t kickStep = thrHalf / 40;
    static inline void kickThr(const etl::uint_ranged<uint8_t, 0, 9>& v) {
        NVM::data().param(Param::KickThr) = kickStep * (v + 1);    
        NVM::data().change();
    }
    static inline void kickTicks(const etl::uint_ranged<uint8_t, 0, 9>& v) {
        NVM::data().param(Param::KickTicks) = External::Tick<Timer>{20_ms}.value * (v + 1);
        NVM::data().change();
    }
    static inline void pwmLow(const etl::uint_ranged<uint8_t, 0, 9>& v) {
        NVM::data().param(Param::PwmLow) = 50* v;
        NVM::data().change();
    }
    
    static inline void nvmInit() {
        NVM::data().param(Param::Dead) = deadStep * 2;   
        NVM::data().param(Param::ThrD1) = thrHalf / 2;
        NVM::data().param(Param::ThrD2) = (5 * thrHalf) / 6;
        NVM::data().param(Param::PwmMin) = 1800;
        NVM::data().param(Param::PwmMax) = 65535;
        NVM::data().param(Param::KickThr) = thrHalf / 8;
        NVM::data().param(Param::KickTicks) = External::Tick<Timer>{75_ms}.value;
        NVM::data().param(Param::PwmLow) = 0;
        NVM::data().change();
    }

    static constexpr External::Tick<Timer> initTicks{100_ms}; 
    static constexpr External::Tick<Timer> deadTicks{100_ms}; // fw <-> bw
    static constexpr External::Tick<Timer> startTicks{10_ms}; 
    
    static inline External::Tick<Timer> kickTicks() {
        return External::Tick<Timer>::fromRaw(NVM::data().param(Param::KickTicks));
    }; 

    using inh1Pin = Meta::nth_element<0, InhPinList>;
    using inh2Pin = Meta::nth_element<1, InhPinList>;
    
    inline static void init() {
        inh1Pin::template dir<Output>();
        inh2Pin::template dir<Output>();
        PWM::init();
        PWM::period(mPwmPeriodMin());
        PWM::template off<Meta::List<AVR::PWM::WO<0>>>();
        PWM::template off<Meta::List<AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();
    }
    
    inline static void event(const Event e) {
        mEvent = e;
    }
    inline static Event event() {
        Event e{Event::None};
        std::swap(e, mEvent);
        return e;
    }
    inline static bool isThrottleValid() {
        return !!mThrottle;
    }
    inline static bool isThrottleForward() {
        return mThrottle && (mThrottle.toInt() >= thrStartForward());
    }
    inline static bool isThrottleBackward() {
        return mThrottle && (mThrottle.toInt() <= thrStartBackward());
    }
    inline static bool isThrottleOn() {
        return isThrottleBackward() || isThrottleForward();
    }
    inline static void ratePeriodic() {
        timeRegulator::ratePeriodic();
        mThrottle = PA::value(0);
        ++mStateTicks;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            if (const auto e = event(); e == Event::Start) {
                mState = State::Off;
            }
            break;
        case State::ForwardWait:
            mStateTicks.on(startTicks, []{
                mState = State::ForwardKick;
            });
            if (!isThrottleForward()) {
                mState = State::Off;
            }
            break;
        case State::ForwardKick:
            mStateTicks.on(kickTicks(), []{
                mState = State::Forward;
            });
        {
            const auto tv = thrForwardKick();
            timeRegulator::off(tv);
//            mTv = tv;
            PWM::period(pwmPeriod(tv));
            const auto d = etl::distance(tv, thrStartForward());
            const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax - thrStartForward()}, etl::Intervall{NVM::data().param(Param::PwmLow), PWM::max()});
            PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(pv);
        }
            break;
        case State::Forward:
            if (const auto e = event(); e == Event::Reset) {
                mState = State::Init;
            }
            else if (!isThrottleForward()) {
                mState = State::OffForwardWait;
            }
            else {
                const auto tv = timeRegulator::value(mThrottle.toInt());
//                mTv = tv;
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartForward());
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax - thrStartForward()}, etl::Intervall{NVM::data().param(Param::PwmLow), PWM::max()});
                PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(pv);
            }
            break;
        case State::Off:
            if (const auto e = event(); e == Event::Reset) {
                mState = State::Init;
            }
            else if (isThrottleForward()) {
                mState = State::ForwardWait;
            }
            else if (isThrottleBackward()) {
                mState = State::BackwardWait;
            } 
            break;
        case State::OffForwardWait:
            if (isThrottleForward()) {
                mState = State::Forward;
            }
            else if (const auto tv = timeRegulator::value(mThrottle.toInt()); tv > thrStartForward()) {
//                mTv = tv;
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartForward());
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax - thrStartForward()}, etl::Intervall{pwm_t{0}, PWM::max()});
                PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(pv);
            }
            else {
                mState = State::Off;
            }
            break;
        case State::BackwardWait:
            mStateTicks.on(startTicks, []{
                mState = State::Backward;
            });
            if (!isThrottleBackward()) {
                mState = State::Off;
            }
            break;
        case State::Backward:
            if (const auto e = event(); e == Event::Reset) {
                mState = State::Init;
            }
            else if (!isThrottleBackward()) {
                mState = State::OffBackwardWait;
            }
            else {
                const auto tv = timeRegulator::value(mThrottle.toInt());
//                mTv = tv;
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartBackward());
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrStartBackward() - thrMin}, etl::Intervall{pwm_t{0}, PWM::max()});
                PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(pv);
            }
            break;
        case State::OffBackwardWait:
            if (isThrottleBackward()) {
                mState = State::Backward;
            }
            else if (const auto tv = timeRegulator::value(mThrottle.toInt()); tv < thrStartBackward()) {
//                mTv = tv;
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartBackward());
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrStartBackward() - thrMin}, etl::Intervall{pwm_t{0}, PWM::max()});
                PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(pv);
            }
            else {
                mState = State::Off;
            }
            break;
        }
        if (oldState != mState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                mRunState = RunState::Undefined;
                off();
                break;
            case State::ForwardWait:
                break;
            case State::ForwardKick:
                timeRegulator::slow(false);
                forward();
                break;
            case State::Forward:
                timeRegulator::slow(false);
                forward();
                break;
            case State::OffForwardWait:
                timeRegulator::slow(true);
                break;
            case State::OffBackwardWait:
                timeRegulator::slow(true);
                break;
            case State::Off:
                mRunState = RunState::Off;
                off();
                break;
            case State::BackwardWait:
                break;
            case State::Backward:
                timeRegulator::slow(false);
                backward();
                break;
            }   
        }
    }    
    
    static inline auto runstate() {
        return mRunState;
    }
    
    template<typename Term>
    static inline void debug() {
        //        etl::outl<Term>("s: "_pgm, (uint8_t)mState, " rs: "_pgm, (uint8_t)mRunState, " p: "_pgm, PWM::max(), " thr: "_pgm, mThrottle.toInt(), " tv: "_pgm, mTv);
    }
    inline static void activate() {
        Meta::visit<InhPinList>([]<typename W>(W) {
                                    W::type::on();
                                });
    }
    inline static void deactivate() {
        Meta::visit<InhPinList>([]<typename W>(W) {
                                    W::type::off();
                                });
    }
private:
    inline static void off() {
        deactivate();
        timeRegulator::off(thrMedium);
        PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(0);
        PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(0);
        PWM::template off<Meta::List<AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();
    }
    inline static void forward() {
        activate();
        PWM::template off<Meta::List<AVR::PWM::WO<2>>>();
        PWM::template on<Meta::List<AVR::PWM::WO<1>>>();
    }
    inline static void backward() {
        activate();
        PWM::template off<Meta::List<AVR::PWM::WO<1>>>();
        PWM::template on<Meta::List<AVR::PWM::WO<2>>>();
    }
    
    inline static uint16_t pwmPeriod(const uint16_t& v) {
        const auto d = etl::distance(v, thrMedium);
        if (d < throttle_delta_1()) {
            mRunState = RunState::Low;
        }
        else if (d > throttle_delta_2()) {
            mRunState = RunState::High;
        }
        else {
            mRunState = RunState::Medium;
        }
        return etl::scale(d, etl::Intervall{throttle_delta_1(), throttle_delta_2()}, etl::Intervall{mPwmPeriodMin(), mPwmPeriodMax()});
    }
    static inline RunState mRunState{RunState::Undefined};
//    static inline uint16_t mTv{};
    static inline throttle_type mThrottle;
    inline static External::Tick<Timer> mStateTicks;
    inline static State mState{State::Undefined};
    inline static Event mEvent{Event::None};
};

template<typename Adc, Adc::index_type I>
struct AnalogPin {
    using value_type = Adc::value_type;
    static inline constexpr auto thresh = value_type::Upper / 2;
    static inline void init() {
        Adc::template pullup<I, true>();
    }
    static inline bool isActive() {
        return (Adc::value(I) < thresh);
    }
};

template<typename BusDevs>
struct GlobalFsm {
    using bus_type = BusDevs::bus_type;
    using devs = BusDevs::devs;
    using Led = devs::led;
    using Timer = devs::systemTimer;
    using NVM = devs::eeprom;
    using data_t = devs::eeprom::data_t;
    using Param = devs::eeprom::data_t::Param;
    using param_cyclic_t = devs::eeprom::data_t::cyclic_t;
    using param_index_t = devs::eeprom::data_t::index_t;
    
    using lut1 = devs::lut1;
    using Adc = devs::adcController;
    
    using Esc = BusDevs::escfsm;
    using Rpm = devs::rpm;

//    using Robo = devs::roboUsart;
    
    using ProviderList = BusDevs::calibratePs;
    
    using blinkLed = External::Blinker2<Led, Timer, 100_ms, 2000_ms>;
    using count_type = blinkLed::count_type;
    static_assert(count_type::Upper >= (param_cyclic_t::Upper + 1));
    
    
    using Servo = BusDevs::servo;
    using servo_pa = BusDevs::servo_pa;
    using search_t = etl::uint_ranged_circular<uint8_t, 0, 15>;
    using value_t = servo_pa::value_type;
    static inline constexpr value_t chThreshH = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshL = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 6};    
    
    using Sensor = BusDevs::sensor;
    using sensor_pa = Sensor::ProtocollAdapter;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;
    
    using adc_i_t = Adc::index_type;
  
    using aPin = AnalogPin<Adc, devs::setupAdcIndex>;
    using button = External::ButtonRelese<aPin, Timer, External::Tick<Timer>{100_ms}, External::Tick<Timer>{1000_ms}>;
    
    enum class State : uint8_t {Undefined, Init, SignalWait, PreCheckStart, CheckStart, 
                                CalibrateOn, CalibrateMes, PreRun, Run,
                                SetupWaitButtonRelease, SetupScan, SetupGotChannel, SetupTestRun, SetupSetParameter, SetupSetParameterWait, 
                               };
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> stateChangeTicks{500_ms};
    static constexpr External::Tick<Timer> debugTicks{500_ms};
    static constexpr External::Tick<Timer> nvmTicks{1000_ms};
    static constexpr External::Tick<Timer> rpmResetTicks{250_ms};
    static constexpr External::Tick<Timer> learnTimeoutTicks{4000_ms};
    static constexpr External::Tick<Timer> pWaitTicks{1000_ms};
    
    static inline auto& data = NVM::data();
    
    inline static void init(const bool inverted = false) {
        NVM::init();
        if (data.mMagic != BusDevs::magic) {
            data.mMagic = BusDevs::magic;
            Esc::nvmInit();
            data.change();
            etl::outl<terminal>("e init"_pgm);
        }
        
        if constexpr(External::Bus::isIBus<bus_type>::value) {
            lut1::init(std::byte{0x00}); // low on lut3-out 
            Sensor::init();
            Sensor::uart::txOpenDrain();
            Servo::template init<BaudRate<115200>>();
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            lut1::init(std::byte{0x0f}); // route TXD (inverted) to lut3-out 
            Sensor::init();
            Sensor::uart::txPinDisable();
            
//            sensor_pa::id(data.physicalId());
            
            Servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
            if (inverted) {
                Servo::rxInvert(true);
            }
            else {
            }
        }
        else if constexpr(External::Bus::isSumD<bus_type>::value) {
            lut1::init(std::byte{0x00}); // low on lut3-out 
            Sensor::init();
            Sensor::uart::txOpenDrain();
            Servo::template init<BaudRate<115200>>();
        }
        else if constexpr(External::Bus::isPpm<bus_type>::value) {
            TermDev::template init<BaudRate<115200>>();
            TermDev::template rxEnable<false>();
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
        
        blinkLed::init();
        Esc::init(); 
        
        Adc::template init<false>(); // no pullups
        Adc::mcu_adc_type::nsamples(4);

        button::init(); // need pullup
        
        Rpm::init();
//        Robo::template init<AVR::HalfDuplex>();
    }
    inline static void periodic() {
        NVM::saveIfNeeded([&]{
            etl::outl<terminal>("save eep"_pgm);
        });
        Sensor::periodic();
        Servo::periodic();
        Led::periodic();
        Adc::periodic();
        if constexpr(External::Bus::isPpm<bus_type>::value) {
            TermDev::periodic();
        }
    }
    inline static void ratePeriodic() {
        button::ratePeriodic();
        servo_pa::ratePeriodic();
        blinkLed::ratePeriodic();
        Sensor::ratePeriodic();
        Esc::ratePeriodic();
        const auto oldState = mState;
        ++mStateTick;
        
        (++mNvmTick).on(nvmTicks, data.expire);
        (++mDebugTick).on(debugTicks, debug);
        (++mResetTick).on(rpmResetTicks, []{
            Rpm::reset();
            if (servo_pa::packages() == 0) {
                mState = State::Undefined;
                RunState::reset();
                Esc::event(Esc::Event::Reset);
            }
            servo_pa::resetStats();
        });
        
        switch(mState) {
        case State::Undefined:
            mStateTick.on(startupTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            if (button::pressed()) {
                mState = State::SetupWaitButtonRelease;
            }
            mStateTick.on(stateChangeTicks, []{
                mState = State::SignalWait;
            });
            break;
        case State::SetupWaitButtonRelease:
            if (const auto be = button::event(); be != button::Press::None) {
                if constexpr(External::Bus::isPpm<bus_type>::value) {
                    mState = State::SetupTestRun;                    
                }
                else {
                    mState = State::SetupScan;
                }
            }
            break;
        case State::SetupScan:
            if (const auto v = servo_pa::value(searchCh.toInt()); v) {
                if (v.toInt() > chThreshH.toInt()) {
                    mState = State::SetupGotChannel;
                }
                else {
                    ++searchCh;
                }
            }
            break;
        case State::SetupGotChannel:
            if (const auto v = servo_pa::value(searchCh.toInt()); v) {
                if (v.toInt() < chThreshL.toInt()) {
                    mState = State::SetupTestRun;
                }
            }
            break;
        case State::SetupSetParameter:
            if (const auto be = button::event(); be == button::Press::Short) {
                setParam(mParam, servo_pa::value(searchCh.toInt()));
                NVM::data().param(mParam);
                mState = State::SetupSetParameterWait;
            }
            else {
                led(Crgb(5 + 21 * scaleTo10(servo_pa::value(searchCh.toInt()))));
            }
            break;
        case State::SetupSetParameterWait:
            mStateTick.on(pWaitTicks, []{
                mState = State::SetupTestRun;
            });
            break;
        case State::SetupTestRun:
            if (const auto es = Esc::runstate(); es == Esc::RunState::Off) {
                if (const auto be = button::event(); be == button::Press::Short) {
                    mState = State::SetupSetParameter;
                }
                else if (be == button::Press::Long) {
                    ++mParam;
                    paramBlink(mParam);
//                    blinkLed::blink(count_type(mParam + 1)); 
                }
            }
            break;
        case State::SignalWait:
            if (Esc::isThrottleValid()) {
                mState = State::PreCheckStart;
            }
            break;
        case State::PreCheckStart:
            mStateTick.on(stateChangeTicks, []{
                mState = State::CheckStart;
            });
            break;
        case State::CheckStart:
            if (!(Esc::isThrottleForward() || Esc::isThrottleBackward())) {
                mState = State::CalibrateOn;
            }
            break;
        case State::CalibrateOn:
            mStateTick.on(stateChangeTicks, []{
                mState = State::CalibrateMes;
            });
            break;
        case State::CalibrateMes:
            mStateTick.on(stateChangeTicks, []{
                mState = State::PreRun;
            });
            break;
        case State::PreRun:
            mStateTick.on(stateChangeTicks, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            RunState::led(Esc::runstate());
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("init"_pgm);
                break;
            case State::SetupWaitButtonRelease:
                etl::outl<terminal>("S WBR"_pgm);
                led(magenta);
                break;
            case State::SetupScan:
                etl::outl<terminal>("S Start"_pgm);
                led(white);
                break;
            case State::SetupGotChannel:
                etl::outl<terminal>("S Got Ch"_pgm);
                break;
            case State::SetupSetParameter:
                etl::outl<terminal>("S Set P"_pgm);
                Esc::event(Esc::Event::Reset);
                break;
            case State::SetupSetParameterWait:
                led(yellow);
                break;
            case State::SetupTestRun:
                etl::outl<terminal>("S Test"_pgm);
                paramBlink(mParam); 
                Esc::event(Esc::Event::Start);
                break;
            case State::SignalWait:
                etl::outl<terminal>("sigw"_pgm);
                led(yellow);
                break;
            case State::CheckStart:
                etl::outl<terminal>("check"_pgm);
                led(magenta);
                break;
            case State::CalibrateOn:
                Esc::activate();
                Meta::visit<ProviderList>([]<typename W>(W){
                                              using provider = W::type;
                                              provider::calibrateStart();
                                          });
                break;
            case State::CalibrateMes:
                Meta::visit<ProviderList>([]<typename W>(W){
                                              using provider = W::type;
                                              provider::calibrate();
                                          });
                break;
            case State::Run:
                etl::outl<terminal>("run"_pgm);
                Esc::event(Esc::Event::Start);
                break;
            case State::PreCheckStart:
                break;
            case State::PreRun:
                Esc::deactivate();
                break;                
            }
        }
    }
private:
    using r_t = etl::uint_ranged<uint8_t, 0, 9>; 
    static inline r_t scaleTo10(const value_t& v) {
        constexpr uint16_t mid = (value_t::Upper + value_t::Lower) / 2;
        constexpr uint16_t top = value_t::Upper;
        return r_t(etl::scale(v.toInt(), etl::Intervall{mid, top}, etl::Intervall<uint16_t>{r_t::Lower, r_t::Upper}));
    }

    static inline void setParam(const param_cyclic_t& p, const value_t& v) {
        const auto sv = scaleTo10(v);
        etl::outl<terminal>("p: "_pgm, p.toInt(), "sv: "_pgm, sv);
        switch(p) {
        case 0:
            Esc::dead(sv);
            break;
        case 1:
            Esc::pwmFreq1(sv);
            break;
        case 2:
            Esc::pwmFreq2(sv);
            break;
        case 3:
            Esc::thr1(sv);
            break;
        case 4:
            Esc::thr2(sv);
            break;
        case 5:
            Esc::kickThr(sv);
            break;
        case 6:
            Esc::kickTicks(sv);
            break;
        case 7:
            Esc::pwmLow(sv);
            break;
        }
    }

    static inline void paramBlink(const param_cyclic_t& p) {
        if (p < 3) {
            Led::color(red);
            blinkLed::blink(count_type(p + 1)); 
        }
        else if (p < 6) {
            Led::color(green);
            blinkLed::blink(count_type(p + 1 - 3)); 
        }
        else if (p < 9) {
            Led::color(blue);
            blinkLed::blink(count_type(p + 1 - 6)); 
        }
    }
    
    static inline void debug() {
//        Robo::put(std::byte{'a'});
//        if (const auto i = Robo::get(); i) {
//            etl::out<terminal>("i: "_pgm, *i);
//        }
        
        etl::out<terminal>("p: "_pgm, servo_pa::packages(), " r: "_pgm, Rpm::value(), " c: "_pgm, Rpm::captures());
        //        etl::out<Term>(" a0: "_pgm, Adc::value(adc_i_t{0}));
        //        etl::out<Term>(" a1: "_pgm, Adc::value(adc_i_t{1}));
        //        etl::out<Term>(" a2: "_pgm, Adc::value(adc_i_t{2}));
        //        etl::out<Term>(" a3: "_pgm, Adc::value(adc_i_t{3}));
        //        etl::out<Term>(" a4: "_pgm, Adc::value(adc_i_t{4}));
        //        etl::out<Term>(" a5: "_pgm, Adc::value(adc_i_t{5}));
        
        Meta::visit<ProviderList>([]<typename W>(W){
                                      etl::out<terminal>(" off: "_pgm, W::type::offset());
                                  });
        
        etl::outl<terminal>(" a4: "_pgm, Adc::value(adc_i_t{4}));
        //        etl::out<Term>(" ga: "_pgm, SigRow<>::tgain());
        //        etl::out<Term>(" of: "_pgm, SigRow<>::toffset());
        //        etl::outl<Term>(" a7: "_pgm, Adc::value(adc_i_t{7}));
        Esc::template debug<terminal>();

    } 
    
    using Crgb = External::Crgb;
    static constexpr Crgb white{External::Red{64}, External::Green{64}, External::Blue{64}};
    static constexpr Crgb red{External::Red{128}, External::Green{0}, External::Blue{0}};
    static constexpr Crgb green{External::Red{0}, External::Green{128}, External::Blue{0}};
    static constexpr Crgb blue{External::Red{0}, External::Green{0}, External::Blue{128}};
    static constexpr Crgb yellow{External::Red{128}, External::Green{128}, External::Blue{0}};
    static constexpr Crgb magenta{External::Red{128}, External::Green{0}, External::Blue{128}};
    static constexpr Crgb cyan{External::Red{0}, External::Green{128}, External::Blue{128}};
    
    struct RunState {
        inline static void led(const Esc::RunState rs) {
            if (mState == State::Run) {
                if (rs != oldState) {
                    oldState = rs;
                    switch(rs) {
                    case Esc::RunState::Undefined:
                        break;
                    case Esc::RunState::Off:
                        etl::outl<terminal>("rs off"_pgm);
                        Led::color(green);
                        blinkLed::blink(count_type{1});
                        break;
                    case Esc::RunState::Low:
                        etl::outl<terminal>("rs low"_pgm);
                        Led::color(green);
                        blinkLed::steady();
                        break;
                    case Esc::RunState::Medium:
                        etl::outl<terminal>("rs med"_pgm);
                        Led::color(yellow);
                        blinkLed::steady();
                        break;
                    case Esc::RunState::High:
                        etl::outl<terminal>("rs high"_pgm);
                        Led::color(red);
                        blinkLed::steady();
                        break;
                    }
                }
            }
        }
        inline static void reset() {
            oldState = Esc::RunState::Undefined;
        }
    private:
        inline static auto oldState{Esc::RunState::Undefined};
    };
    
    inline static void led(const Crgb& c) {
        Led::color(c);
        blinkLed::steady();
    }

    inline static param_cyclic_t mParam{};
    
    inline static search_t searchCh{0};
    
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mDebugTick;
    static inline External::Tick<Timer> mResetTick;
    static inline External::Tick<Timer> mNvmTick;
};

struct VersionProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::Temp1;
    inline static constexpr auto ibus_type = IBus2::Type::type::ARMED;
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return 1012;
    }
};


template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;

    using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Sensor
    using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Servo / Debug
        
    using servoPosition = usart2Position; 
    using scanDevPosition = servoPosition;
    
    using sensorPosition = usart1Position; // Sensor
    using scanTermPosition = sensorPosition;
#ifdef NDEBUG
    using scan_term_dev = void;
#else
    using scan_term_dev = Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#endif
    
#ifndef NDEBUG
    using assertPin = Pin<Port<A>, 0>; 
#endif
    
    using timing0Pin = Pin<Port<D>, 3>; 
    using timing1Pin = Pin<Port<D>, 4>; 
    using timing2Pin = Pin<Port<D>, 5>; 
    
    using daisyChain= Pin<Port<A>, 7>; 
    
    using inh1Pin = Pin<Port<A>, 3>; 
    using inh2Pin = Pin<Port<A>, 4>; 
    
    using rpmPin = Pin<Port<F>, 2>; 
    
    // lut1 out: pc3
    using ccl1Position = Portmux::Position<Component::Ccl<1>, Portmux::Default>;
    using lut1 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Usart<1>, Ccl::Input::Mask>;
    
    using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
    
    // WO0: unbenutzt
    // WO1: pwm1
    // WO2: pwm2
    using pwm = PWM::DynamicPwmPrescale<tcaPosition, AVR::Prescaler<4>>;
    
    using ibt = IBusThrough<daisyChain>;
    
    using adc = Adc<Component::Adc<0>, AVR::Resolution<12>, Vref::V2_048>;
    // ADC Channels: T1, BecI1, BecI2, V+, Text, T2, Curr
    using adcController = External::Hal::AdcController<adc, Meta::NList<0, 1, 6, 7, 19, 20, 21, 0x42>>; // 0x42 = temp
    using adc_i_t = adcController::index_type;
    inline static constexpr adc_i_t setupAdcIndex{4};
    
    using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;
    using spi = AVR::Spi<spiPosition, AVR::QueueLength<16>,  AVR::UseInterrupts<false>>;
    using ledStripe = External::LedStripe<spi, External::APA102, 1>;
    using led = ColorLedAdapter<ledStripe>;
    using scanLedPin = led;
    
    using rpmPositionL = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
    using rpmPositionH = Portmux::Position<Component::Tcb<1>, Portmux::Default>;
    
    using sppmPosition = Portmux::Position<Component::Tcb<2>, Portmux::Default>;
    using ppmIn =  AVR::Pin<AVR::Port<F>, 1>;
    using evch5 = Event::Channel<5, Event::Generators::Pin<ppmIn>>; 
    using ppm_user = Event::Route<evch5, Event::Users::TcbCapt<2>>;
    using ppmDevPosition = AVR::Component::Tcb<2>;

    using evch4 = Event::Channel<4, Event::Generators::Pin<rpmPin>>;
    
    using clockProvider = pwm::clock_provider;
    using rpm = External::Rpm::RpmFreq<evch4, Meta::List<rpmPositionL, rpmPositionH>, clockProvider>;
    
    using rpmOvfChannel = rpm::overflow_channel<0>;
    using rpmRoutes = rpm::event_routes<rpmOvfChannel>;

    using allRoutes = Meta::push_back<rpmRoutes::routes, ppm_user>;
    using evrouter = Event::Router<Event::Channels<evch4, rpmOvfChannel, evch5>, allRoutes>;
    
    using temp1 = External::AnalogSensor<adcController, 0, std::ratio<500,1000>, 
    std::ratio<10, 1000>, 
    std::ratio<10,1>>;
    using temp2 = External::AnalogSensor<adcController, 5, std::ratio<500,1000>, 
    std::ratio<10, 1000>, 
    std::ratio<10,1>>;
    using text = External::AnalogSensor<adcController, 4, std::ratio<500,1000>, 
    std::ratio<10, 1000>, 
    std::ratio<10,1>>;
    
    using vdiv = External::AnalogSensor<adcController, 3, std::ratio<0,1>, 
    std::ratio<R2vd, R2vd + R1vd>, 
    std::ratio<100,1>>;
    
    using bec1S = External::AnalogSensor<adcController, 1, std::ratio<0,1>, 
    std::ratio<1900, 1000>, 
    std::ratio<100,1>>;
    
    using currS = External::AnalogSensor<adcController, 6, std::ratio<0,1>, 
    std::ratio<550, 1000>, 
    std::ratio<1000,1>>;
    
    using eeprom = EEProm::Controller<Data>;
    
    using portmux = Portmux::StaticMapper<Meta::List<spiPosition, ccl1Position, tcaPosition, servoPosition, sensorPosition, 
    rpmPositionL, rpmPositionH, sppmPosition>>;
    
    // exclusiv to rpmL/rpmH
//    using rxPin = Pin<Port<F>, 3>;
//    using txPin = Pin<Port<F>, 2>;
//    using roboUsart = External::SoftSerial::Usart<Meta::List<rxPin, txPin>, Component::Tcb<0>, 
//    External::Hal::NullProtocollAdapter, AVR::BaudRate<9600>, AVR::ReceiveQueueLength<16>, AVR::SendQueueLength<64>>;

//    using isrRegistrar = IsrRegistrar<roboUsart::StartBitHandler, roboUsart::BitHandler>;
    
    static inline void init() {
        portmux::init();
        evrouter::init();
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDa32<MCU>) {
                clock::template init<Project::Config::fMcuMhz>();
            }
            else if constexpr(AVR::Concepts::At01Series<MCU>) {
                clock::template prescale<1>();
            }
            else {
                static_assert(std::false_v<MCU>);
            }
        });
        systemTimer::init(); 
    }
    static inline void periodic() {
        led::periodic();
    }
};

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    static inline uint8_t magic = 42;
    using bus_type = External::Bus::IBusIBus<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
        inline static constexpr auto stateProviderId = IBus2::Type::type::FLIGHT_MODE;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, 
    AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using escfsm = EscFsm<systemTimer, typename Devs::pwm, Meta::List<typename Devs::inh1Pin, typename Devs::inh2Pin>, servo_pa, typename Devs::eeprom>;
    
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    template<typename Sensor>
    struct CProvider {
        inline static constexpr auto valueId = External::SPort::ValueId::Current;
        inline static uint32_t value() {
            return Sensor::value();
        }
    };

    template<typename R>
    struct RpmProvider {
    #ifdef FS_I6S    
        inline static constexpr auto ibus_type = IBus2::Type::type::RPM_FLYSKY;
    #else
        inline static constexpr auto ibus_type = IBus2::Type::type::RPM;
    #endif
        inline static constexpr void init() {
        }
        inline static constexpr uint16_t value() {
            if (const auto rpm = R::value(); rpm) {
                return rpm.value();
            }
            return 0;
        }
    };
    
    template<typename Sensor>
    struct TempProvider {
        inline static constexpr auto ibus_type = IBus2::Type::type::TEMPERATURE;
        inline static constexpr void init() {}
        
        inline static constexpr void calibrateStart() {
            Sensor::template pullup<true>();
        }
        inline static constexpr void calibrate() {
            constexpr uint16_t U = decltype(Sensor::raw())::Upper;
            //        std::integral_constant<uint16_t, U>::_;
            mCalibration = Sensor::raw();
            if (mCalibration == U) {
                mActive = false;
            }
            else {
                mActive = true;
            }
            Sensor::template pullup<false>();
        }
        inline static constexpr uint16_t value() {
            if (mActive) {
                return Sensor::value() + 400;
            }
            else {
                return 400;
            }
        }
        inline static uint16_t offset() {
            return mCalibration;
        }
    private:
        inline static uint16_t mCalibration{};
        inline static bool mActive{false};
    };
    template<typename Sensor>
    struct VoltageProvider {
        inline static constexpr auto ibus_type = IBus2::Type::type::EXTERNAL_VOLTAGE;
        inline static constexpr void init() {}
        
        inline static constexpr uint16_t value() {
            return Sensor::value();
        }
    };
    template<typename Sensor>
    struct CurrentProvider {
        inline static constexpr auto ibus_type = IBus2::Type::type::BAT_CURR;
        inline static constexpr void init() {}
        
        inline static constexpr void calibrateStart() {
        }
        inline static constexpr void calibrate() {
            mOffset = Sensor::value();
        }
        
        inline static constexpr uint16_t value() {
            const auto value = Sensor::value();
            if (value > mOffset) {
                return (value - mOffset);
            }
            return 0;
        }
        inline static uint16_t offset() {
            return mOffset;
        }
    private:
        inline static uint16_t mOffset{};
    };
    
    template<typename ADC, uint8_t Channel, typename SigRow>
    struct InternalTempProvider {
        using index_t = ADC::index_type;
        static_assert(Channel <= index_t::Upper);
        inline static constexpr auto channel = index_t{Channel};
        inline static constexpr auto ibus_type = IBus2::Type::type::TEMPERATURE;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            return SigRow::template adcValueToTemperature<std::ratio<1,10>, 40, typename ADC::VRef_type>(ADC::value(channel)).value;
        }
    };
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 7, typename Devs::sigrow>;
    using temp1P = TempProvider<typename Devs::temp1>;
    using temp2P = TempProvider<typename Devs::temp1>;
    using textP = TempProvider<typename Devs::text>;
    
    using voltageP = VoltageProvider<typename Devs::vdiv>;
    using cBecP = CurrentProvider<typename Devs::bec1S>;
    using currP = CurrentProvider<typename Devs::currS>;
    using rpmP = RpmProvider<typename Devs::rpm>;
    
    using calibratePs = Meta::List<currP>;

    using sensor = IBus2::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<115200>, 
                                Meta::List<VersionProvider, 
                                           temp1P, temp2P, tempiP, textP, 
                                           voltageP, cBecP, currP, rpmP>, 
                                systemTimer, typename devs::ibt
    //                          , etl::NamedFlag<true>
    //                           , etl::NamedFlag<true>
    >;


};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    static inline uint8_t magic = 43;
    using bus_type = External::Bus::SBusSPort<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
        inline static constexpr auto stateProviderId = External::SPort::ValueId::DIY;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, 
    AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using escfsm = EscFsm<systemTimer, typename Devs::pwm, Meta::List<typename Devs::inh1Pin, typename Devs::inh2Pin>, servo_pa, typename Devs::eeprom>;
    
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    template<typename PA>
    using sensorUsart = AVR::Usart<typename Devs::sensorPosition, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
    
    template<typename Sensor>
    struct CurrentProvider {
        inline static constexpr auto valueId = External::SPort::ValueId::Current;
        inline static constexpr void init() {}
        
        inline static constexpr void calibrateStart() {
        }
        inline static constexpr void calibrate() {
            mOffset = Sensor::value();
        }
        
        inline static constexpr uint32_t value() {
            const auto value = Sensor::value();
            if (value > mOffset) {
                return (value - mOffset);
            }
            return 0;
        }
        inline static uint16_t offset() {
            return mOffset;
        }
    private:
        inline static uint16_t mOffset{};
    };
    
    using currP = CurrentProvider<typename Devs::currS>;
    
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, Meta::List<VersionProvider>>;

    using calibratePs = Meta::List<currP>;

};

template<typename Devs>
struct BusDevs<External::Bus::SumDHott<Devs>> {
    static inline uint8_t magic = 44;
    using bus_type = External::Bus::SumDHott<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, 
    AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using escfsm = EscFsm<systemTimer, typename Devs::pwm, Meta::List<typename Devs::inh1Pin, typename Devs::inh2Pin>, servo_pa, typename Devs::eeprom>;
    
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    using sensor = Hott::Experimental::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, systemTimer>;

    using calibratePs = Meta::List<>;
};

template<typename Devs>
struct BusDevs<External::Bus::Ppm<Devs>> {
    static inline uint8_t magic = 45;
    using bus_type = External::Bus::Ppm<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using sppm_input = External::Ppm::SinglePpmIn<typename devs::sppmPosition::component_type>;
     
    using servo = External::Ppm::Adapter<sppm_input>;
    using servo_pa = servo::protocoll_adapter_type;
    
    using escfsm = EscFsm<systemTimer, typename Devs::pwm, Meta::List<typename Devs::inh1Pin, typename Devs::inh2Pin>, servo_pa, typename Devs::eeprom>;
    
#ifndef NDEBUG
    using term_dev = Usart<typename devs::servoPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
    using terminal = etl::basic_ostream<term_dev>;
#else
    using terminal = etl::basic_ostream<void>;
#endif

    struct NullSensor {
        struct ProtocollAdapter {
            
            inline static void ratePeriodic() {}
        };
        inline static void periodic() {}
        inline static void ratePeriodic() {}
    };
    
    using sensor = NullSensor;
    
    using calibratePs = Meta::List<>;
};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isSBus<BusSystem>::value || External::Bus::isIBus<BusSystem>::value || External::Bus::isSumD<BusSystem>::value || External::Bus::isPpm<BusSystem>::value) {
            using terminal = devs::terminal;
            using systemTimer = devs::systemTimer;
            using gfsm = GlobalFsm<devs>;
            
            gfsm::init(inverted);
            etl::outl<terminal>("test21"_pgm);

//            etl::Scoped<etl::EnableInterrupt<>> ei;
            
            while(true) {
                gfsm::periodic(); 
                systemTimer::periodic([&]{
                    gfsm::ratePeriodic();
                });
            }
        }
    }
};

using devices = Devices<>;
using scanner = External::Scanner<devices, Application>;

int main() {
    scanner::run();
}

//ISR(PORTF_PORT_vect) {
//    devices::isrRegistrar::isr<AVR::ISR::Port<devices::rxPin::name_type>>();
//}
//ISR(TCB0_INT_vect) {
//    devices::isrRegistrar::isr<AVR::ISR::Tcb<0>::Capture>();
//}

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
        while(true) {
            devices::assertPin::toggle();
        }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
