#define NDEBUG

#define LEARN_DOWN

#ifndef GITMAJOR
# define VERSION_NUMBER 0001
#endif

#define HOTT_TITLE "BMC " GITTAG_PGM

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

    constexpr uint16_t R1vd = 22'000; // 220k (wrong in schematics)
    constexpr uint16_t R2vd = 1'200;
}

template<typename Esc> struct RCMenu;

struct Data final : public EEProm::DataBase<Data> {
    uint8_t mMagic{};

    using search_t = etl::uint_ranged_circular<uint8_t, 0, 15>;
    search_t mChannel{};
    
    uint16_t mThrMin{0};
    uint16_t mThrMax{65535};
    uint16_t mThrMid{32000};
    uint16_t mThrHalf{16000};
    
    enum class Param : uint8_t {Dead, ThrD1, ThrD2, PwmMax, PwmMin, KickThr, KickTicks, PwmLow, SlowDown, SPortID, NSensors, _Number};
    
    using pa_t = std::array<uint16_t, (uint8_t)Param::_Number>;
//    pa_t::_;
    using index_t = etl::index_type_t<pa_t>;
//    index_t::_
    using cyclic_t = etl::cyclic_type_t<pa_t>;
//    cyclic_t::_;
//    uint16_t& param(const index_t i) {
//        return mParameter[i.toInt()];
//    }
    uint16_t& param(const Param i) {
        return mParameter[(uint8_t)i];
    }
private:
    uint16_t dummy;
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
            mMaxDiff = std::max(raw_type{2}, mSlowDiff);
//            mMaxDiff = std::max(raw_type{2}, (halfSpan / 300));
        }
        else {
            mMaxDiff = std::max(raw_type{10}, (halfSpan / halfSpanDivider));
//            mMaxDiff = std::max(raw_type{10}, (halfSpan / 40));
        }
    }
    static inline void slowValue(const raw_type& d) {
        mSlowDiff = std::min(d, (halfSpan / halfSpanDivider));
    }
    static inline raw_type slowValue() {
        return mSlowDiff;
    }
    static inline constexpr uint16_t halfSpanDivider = 25;
//    std::integral_constant<uint16_t, halfSpan/halfSpanDivider>::_;
//    std::integral_constant<uint16_t, halfSpan/(10*halfSpanDivider)>::_;
private:
    static inline External::Tick<Timer> deltaTick;
    static inline raw_type mSlowDiff{halfSpan / (10 * halfSpanDivider)};
    static inline raw_type mMaxDiff{4};
    static inline raw_type mTarget{};
    static inline raw_type mActual{};
};

template<typename Timer, typename PWM, typename InhPin, typename PA, typename NVM>
struct EscFsm {
    
    using Param = NVM::data_t::Param;
    
    enum class State : uint8_t {Undefined = 0, Init, 
                                Off = 10,  
                                Forward = 20, ForwardWait, ForwardKick, OffForwardWait, 
                                Backward = 30, BackwardWait, BackwardKick, OffBackwardWait };
    
    enum class RunState : uint8_t {Undefined = 0, Off, Low, Medium, High};
    
    enum class Event : uint8_t {None, Start, Reset};
    
    using throttle_type = typename PA::value_type;
//    throttle_type::_;
    using thr_t = typename throttle_type::value_type;
//        thr_t::_;
    using timeRegulator  = TimeRegulator<Timer, throttle_type>;
    
    using pwm_t = PWM::value_type;
    //    pwm_t::_;
    
    using pvalue_t = etl::uint_ranged<uint8_t, 0, 9>;
    
    static inline constexpr thr_t CthrMax{throttle_type::Upper};
    static inline constexpr thr_t CthrMin{throttle_type::Lower};
    static inline constexpr thr_t CthrHalf{(throttle_type::Upper - throttle_type::Lower) / 2};
    static inline constexpr thr_t CthrMedium{(throttle_type::Upper + throttle_type::Lower) / 2};

    static inline auto state() {
        return mState;
    }
    static inline bool isForward() {
        return (mState >= State::Forward) && (mState <= State::OffForwardWait);
    }
    static inline bool isBackward() {
        return (mState >= State::Backward) && (mState <= State::OffBackwardWait);
    }

    static inline thr_t thrMax() {
//        return CthrMax;
        return NVM::data().mThrMax;
    }
    static inline thr_t thrMin() {
//        return CthrMin;
        return NVM::data().mThrMin;
    }
    static inline thr_t thrHalf() {
//        return CthrHalf;
        return NVM::data().mThrHalf;
    }
    static inline thr_t thrMedium() {
//        return CthrMedium;
        return NVM::data().mThrMid;
    }
    
    static inline thr_t thrStartForward() {
        return thrMedium() + NVM::data().param(Param::Dead);
    };
    static inline thr_t thrStartBackward() {
        return thrMedium() - NVM::data().param(Param::Dead);
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
    
    static inline thr_t thrForwardKick() {
        return thrMedium()  + NVM::data().param(Param::KickThr);
    };
    static inline thr_t thrBackwardKick() {
        return thrMedium()  - NVM::data().param(Param::KickThr);
    };
    static inline uint16_t thrForwardKickTisck() {
        return NVM::data().param(Param::KickTicks);
    };

    static inline constexpr thr_t CdeadStep = CthrHalf / 40;
//    std::integral_constant<uint16_t, deadStep>::_;
    static inline thr_t deadStep() {
        return CdeadStep;
//        return thrHalf() / 40;
    };
        
    static inline void dead(const pvalue_t& v) {
        NVM::data().param(Param::Dead) = deadStep() * (v + 1);               
        NVM::data().change();
    }
    static inline pvalue_t p_dead() {
        return pvalue_t(NVM::data().param(Param::Dead) / deadStep() - 1);
    }

    static inline void setSlowDown(const pvalue_t& v) {
        timeRegulator::slowValue((timeRegulator::halfSpan * (v + 1)) / (10 * timeRegulator::halfSpanDivider));               
    }
    static inline void slowdown(const pvalue_t& v) {
        setSlowDown(v);
        NVM::data().param(Param::SlowDown) = v;               
        NVM::data().change();
    }
    static inline pvalue_t p_slowdown() {
        return pvalue_t(NVM::data().param(Param::SlowDown));
    }
    
    static inline constexpr uint16_t pwmStep1 = PWM::f_timer.value / 18000;
//    std::integral_constant<uint16_t, pwmStep1>::_;
    
    static inline void pwmFreq1(const pvalue_t& v) {
        NVM::data().param(Param::PwmMin) = (10 - v ) * pwmStep1;
        NVM::data().change();
    }
    static inline pvalue_t p_pwmFreq1() {
        return pvalue_t(10 - NVM::data().param(Param::PwmMin) / pwmStep1);
    }
    
    static inline constexpr uint16_t pwmStep2 = (65535 - 10 * pwmStep1) / 9;
    static_assert((10 * pwmStep1) <= pwmStep2);    
//        std::integral_constant<uint16_t, pwmStep2>::_;
    static inline void pwmFreq2(const pvalue_t& v) {
        NVM::data().param(Param::PwmMax) = 65535 - v * pwmStep2; // 9: 10 * pwmStep1     
        NVM::data().change();
    }
    static inline pvalue_t p_pwmFreq2() {
        return pvalue_t((65535 - NVM::data().param(Param::PwmMax)) / pwmStep2);
    }

    static inline void thr1(const pvalue_t& v) {
        NVM::data().param(Param::ThrD1) = (thrHalf() * v) / 9;    
        NVM::data().param(Param::ThrD2) = std::max(NVM::data().param(Param::ThrD1) + 1, NVM::data().param(Param::ThrD2));    
        NVM::data().change();
    }
    static inline pvalue_t p_thr1() {
        return pvalue_t((NVM::data().param(Param::ThrD1) * 9) / thrHalf());
    }    
    
    static inline void thr2(const pvalue_t& v) {
        thr_t nv = (thrHalf() * v) / 9;    
        NVM::data().param(Param::ThrD2) = std::max(NVM::data().param(Param::ThrD1) + 1, nv);    
        NVM::data().change();
    }
    static inline pvalue_t p_thr2() {
        return pvalue_t((NVM::data().param(Param::ThrD2) * 9) / thrHalf());
    }

    static inline constexpr thr_t CkickStep = CthrHalf / 40;
    static inline void kickThr(const pvalue_t& v) {
        NVM::data().param(Param::KickThr) = CkickStep * (v + 1);    
        NVM::data().change();
    }
    static inline pvalue_t p_kickThr() {
        return pvalue_t((NVM::data().param(Param::KickThr) / CkickStep) - 1);
    }

    static inline void kickTicks(const pvalue_t& v) {
        NVM::data().param(Param::KickTicks) = External::Tick<Timer>{20_ms}.value * (v + 1);
        NVM::data().change();
    }
    static inline pvalue_t p_kickTicks() {
        return pvalue_t((NVM::data().param(Param::KickTicks) / External::Tick<Timer>{20_ms}.value ) - 1);
    }

    static inline void pwmLow(const pvalue_t& v) {
        NVM::data().param(Param::PwmLow) = (PWM::max() / 40) * v;
        NVM::data().change();
    }
    static inline pvalue_t p_pwmLow() {
        return pvalue_t(NVM::data().param(Param::PwmLow) / (PWM::max() / 40));
    }
    
    static inline void nvmInit() {
        NVM::data().mThrMid = (throttle_type::Upper + throttle_type::Lower) / 2;
        NVM::data().mThrMin = throttle_type::Lower;
        NVM::data().mThrMax = throttle_type::Upper;       
        NVM::data().mThrHalf = (throttle_type::Upper - throttle_type::Lower) / 2;
        
        NVM::data().param(Param::Dead) = deadStep() * 2;   
        NVM::data().param(Param::ThrD1) = thrHalf() / 2;
        NVM::data().param(Param::ThrD2) = (5 * thrHalf()) / 6;
        NVM::data().param(Param::PwmMin) = pwmStep1;
        NVM::data().param(Param::PwmMax) = pwmStep2;
        NVM::data().param(Param::KickThr) = thrHalf() / 8;
        NVM::data().param(Param::KickTicks) = External::Tick<Timer>{75_ms}.value;
        NVM::data().param(Param::PwmLow) = 0;
        NVM::data().param(Param::SlowDown) = 9;
        
        NVM::data().change();
    }

    static constexpr External::Tick<Timer> initTicks{100_ms}; 
    static constexpr External::Tick<Timer> deadTicks{100_ms}; // fw <-> bw
    static constexpr External::Tick<Timer> startTicks{10_ms}; 
    
    static inline External::Tick<Timer> kickTicks() {
        return External::Tick<Timer>::fromRaw(NVM::data().param(Param::KickTicks));
    }; 

    using inhPin = InhPin;
    
    inline static void init() {
        inhPin::template dir<Output>();
        PWM::init();
        PWM::period(mPwmPeriodMin());
        PWM::template off<Meta::List<AVR::PWM::WO<0>>>();
        PWM::template off<Meta::List<AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();

        setSlowDown(pvalue_t(NVM::data().param(Param::SlowDown)));
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
    inline static auto& searchCh() {
        return NVM::data().mChannel;
    }
    inline static void ratePeriodic() {
        timeRegulator::ratePeriodic();
        mThrottle = PA::value(searchCh().toInt());
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
            PWM::period(pwmPeriod(tv));
            const auto d = etl::distance(tv, thrStartForward());
            const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax() - thrStartForward()}, etl::Intervall{NVM::data().param(Param::PwmLow), PWM::max()});
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
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartForward());
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax() - thrStartForward()}, etl::Intervall{NVM::data().param(Param::PwmLow), PWM::max()});
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
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartForward());
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax() - thrStartForward()}, etl::Intervall{pwm_t{0}, PWM::max()});
                PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(pv);
            }
            else {
                mState = State::Off;
            }
            break;
        case State::BackwardWait:
            mStateTicks.on(startTicks, []{
                mState = State::BackwardKick;
            });
            if (!isThrottleBackward()) {
                mState = State::Off;
            }
            break;
        case State::BackwardKick:
            mStateTicks.on(kickTicks(), []{
                mState = State::Backward;
            });
        {
            const auto tv = thrBackwardKick();
            timeRegulator::off(tv);
            PWM::period(pwmPeriod(tv));
            const auto d = etl::distance(tv, thrStartBackward());
            const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrStartBackward() - thrMin()}, etl::Intervall{NVM::data().param(Param::PwmLow), PWM::max()});
            PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(pv);
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
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartBackward());
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrStartBackward() - thrMin()}, etl::Intervall{NVM::data().param(Param::PwmLow), PWM::max()});
                PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(pv);
            }
            break;
        case State::OffBackwardWait:
            if (isThrottleBackward()) {
                mState = State::Backward;
            }
            else if (const auto tv = timeRegulator::value(mThrottle.toInt()); tv < thrStartBackward()) {
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartBackward());
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrStartBackward() - thrMin()}, etl::Intervall{pwm_t{0}, PWM::max()});
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
            case State::BackwardKick:
                timeRegulator::slow(false);
                backward();
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
//                etl::outl<Term>("s: "_pgm, (uint8_t)mState, " rs: "_pgm, (uint8_t)mRunState, " p: "_pgm, PWM::max(), " thr: "_pgm, mThrottle.toInt());
                etl::outl<Term>("s: "_pgm, (uint8_t)mState, " sd: "_pgm, timeRegulator::slowValue());
    }
    inline static void activate() {
        inhPin::on();
    }
    inline static void deactivate() {
        inhPin::off();
    }
private:
    inline static void off() {
        deactivate();
        timeRegulator::off(thrMedium());
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
        const auto d = etl::distance(v, thrMedium());
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
    static inline constexpr auto thresh = value_type::Upper / 4;
    static inline void init() {
        Adc::template pullup<I, true>();
    }
    static inline bool isActive() {
        return (Adc::value(I) < thresh);
    }
};

template<typename BusDevs>
struct GlobalFsm {
    using gfsm = GlobalFsm<BusDevs>;
    
    using bus_type = BusDevs::bus_type;
    using devs = BusDevs::devs;
    using Led = devs::led;
    using Timer = devs::systemTimer;
    using NVM = devs::eeprom;
    using data_t = devs::eeprom::data_t;
    using Param = devs::eeprom::data_t::Param;
    using param_cyclic_t = devs::eeprom::data_t::cyclic_t;
    using param_index_t = devs::eeprom::data_t::index_t;
    
    inline static constexpr uint8_t maxParameter = 11;
    static_assert(maxParameter >= param_index_t::Upper);
    inline static constexpr uint8_t maxParamValue = 9;
    
    using lut1 = devs::lut1;
    using Adc = devs::adcController;
    
    using Esc = BusDevs::escfsm;
    using Rpm = devs::rpm;

    using ProviderList = BusDevs::calibratePs;
    
    using blinkLed = External::Blinker2<Led, Timer, 100_ms, 2500_ms>;
    using count_type = blinkLed::count_type;
    static_assert(count_type::Upper >= (param_cyclic_t::Upper + 1));
    
    using Servo = BusDevs::servo;
    using servo_pa = BusDevs::servo_pa;
    using search_t = etl::uint_ranged_circular<uint8_t, 0, 15>;
    using value_t = servo_pa::value_type;
    static inline constexpr value_t chThreshHPos = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshLPos = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 6};    
    static inline constexpr value_t chThreshHNeg = value_t{value_t::Mid - (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshLNeg = value_t{value_t::Mid - (value_t::Upper - value_t::Lower) / 6};    
    
//    std::integral_constant<uint16_t, chThreshHPos.toInt()>::_;
    
    using Sensor = BusDevs::sensor;
    using sensor_pa = Sensor::ProtocollAdapter;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;
    
    using adc_i_t = Adc::index_type;
  
    using aPin = AnalogPin<Adc, devs::setupAdcIndex>;
    using button = External::ButtonRelese<aPin, Timer, External::Tick<Timer>{100_ms}, External::Tick<Timer>{1000_ms}>;
    
    using cp = BusDevs::configProvider;

    using menu = std::conditional_t<External::Bus::isSumD<bus_type>::value, Hott::BasePage<Sensor, RCMenu<gfsm>>, void>;
    
    enum class State : uint8_t {Undefined, Init, SignalWait, PreCheckStart, CheckStart, 
                                CalibrateOn, CalibrateMes, PreRun, Run,
                                SetupWaitButtonRelease, SetupScan, SetupGotChannel, SetupTestRun, SetupSetParameter, SetupSetParameterWait, SetupEnd,
                                DigitalSetupChoose, DigitalSetupSet, DigitalSetupTestRun,
                                DigitalSetupSel, DigitalSetupSel2, DigitalSetupLeave,
                                SetupCalibrateMes, SetupCalibrateOn,
                                SetupPpmScale1, SetupPpmScale2, SetupPpmScaleStart
                               };
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> stateChangeTicks{500_ms};
    static constexpr External::Tick<Timer> debugTicks{500_ms};
    static constexpr External::Tick<Timer> nvmTicks{1000_ms};
    static constexpr External::Tick<Timer> rpmResetTicks{250_ms};
    static constexpr External::Tick<Timer> learnTimeoutTicks{4000_ms};
    static constexpr External::Tick<Timer> pWaitTicks{1000_ms};
    
    static inline auto& data = NVM::data();

    template<uint8_t N>
    static inline auto scaleToN(const value_t& v) {
        using r_t = etl::uint_ranged<uint8_t, 0, N>; 
        constexpr uint16_t mid = (value_t::Upper + value_t::Lower) / 2;
        constexpr uint16_t top = value_t::Upper;
        return r_t(etl::scale(v.toInt(), etl::Intervall{mid, top}, etl::Intervall<uint16_t>{r_t::Lower, r_t::Upper}), etl::RangeCheck<false>{});
    }

    template<bool full = false>
    inline static void nvmDefaults() {
        Esc::nvmInit();
        if constexpr(full) {
            searchCh().setToBottom();
        }
        data.param(Param::SPortID) = 3;
        data.param(Param::NSensors) = maxParamValue;
        
        data.change();
        updateSensorId();
    } 

    inline static void updateSensorId() {
        if constexpr(External::Bus::isSBus<bus_type>::value) {
            const auto value = data.param(Data::Param::SPortID);
            const External::SPort::SensorId id = External::SPort::idFromIndex(value);
            sensor_pa::id(id);
        }
    }
    
    inline static void init(const bool inverted = false) {
        NVM::init();
        if (data.mMagic != BusDevs::magic) {
            data.mMagic = BusDevs::magic;
            nvmDefaults<true>();
            etl::outl<terminal>("e init"_pgm);
        }
        
        if constexpr(External::Bus::isIBus<bus_type>::value) {
            lut1::init(std::byte{0x00}); // low on lut3-out 
            Sensor::init();
            Sensor::uart::txOpenDrain();
            Servo::template init<BaudRate<115200>>();

            etl::outl<terminal>("IB"_pgm);
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            lut1::init(std::byte{0x33}); // route TXD (inverted) to lut1-out 
            Sensor::init();
            Sensor::uart::txPinDisable();

            devs::ibt::init();
            devs::ibt::off();
            
            updateSensorId();
            
            Servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
            if (inverted) {
                Servo::rxInvert(true);
                etl::outl<terminal>("SB I"_pgm);
            }
            else {
                etl::outl<terminal>("SB"_pgm);
            }
        }
        else if constexpr(External::Bus::isSumD<bus_type>::value) {
            lut1::init(std::byte{0x00}); // low on lut1-out 
            Sensor::init();
            Sensor::uart::template txPinPullup<false>(); 
            
            Servo::template init<BaudRate<115200>>();
            
            devs::ibt::init();
            devs::ibt::off();
            
            menu::init();
            
            etl::outl<terminal>("SD"_pgm);
        }
        else if constexpr(External::Bus::isPpm<bus_type>::value) {
            if constexpr(!std::is_same_v<TermDev, void>) {
                TermDev::template init<BaudRate<115200>>();
                TermDev::template rxEnable<false>();
            }
            etl::outl<terminal>("PPm"_pgm);
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
            if constexpr(!std::is_same_v<TermDev, void>) {
                TermDev::periodic();
            }
        }
        if constexpr(External::Bus::isSumD<bus_type>::value) {
            menu::periodic();
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
                    mState = State::SetupPpmScaleStart;                    
//                    mState = State::SetupTestRun;                    
                }
                else {
                    mState = State::SetupScan;
                }
            }
            break;
        case State::SetupPpmScaleStart:
            mStateTick.on(pWaitTicks, []{
                mState = State::SetupPpmScale1;
            });
            break;
        case State::SetupPpmScale1:
            if (const auto be = button::event(); be == button::Press::Short) {
                mState = State::SetupPpmScale2;                
            }
            else {
                if (const auto v = servo_pa::value(searchCh().toInt())) {
                    data.mThrMax = std::max(v.toInt(), data.mThrMax);
                    data.mThrMin = std::min(v.toInt(), data.mThrMin);
                    const auto d = etl::distance(v.toInt(), value_t::Mid);
                    const auto b = etl::scale(d, etl::Intervall{0U, (value_t::Upper - value_t::Mid)}, etl::Intervall{0U, 255U});
                    led(Crgb(b));
                }
            }
            break;
        case State::SetupPpmScale2:
            if (!(Esc::isThrottleForward() || Esc::isThrottleBackward())) {
                mState = State::SetupTestRun;
            }
            break;
        case State::SetupScan:
            if (const auto v = servo_pa::value(searchCh().toInt()); v && (v.toInt() > chThreshHPos.toInt())) {
                mState = State::SetupGotChannel;
            }
            else {
                ++searchCh();
                if constexpr(External::Bus::isSumD<bus_type>::value) {
                    if (searchCh() >= 7) {
                        searchCh().setToBottom();
                    }                    
                }
            }
            break;
        case State::SetupGotChannel:
            if (const auto v = servo_pa::value(searchCh().toInt()); v) {
                if (v.toInt() < chThreshLPos.toInt()) {
                    if constexpr(External::Bus::isSumD<bus_type>::value) {
                        mState = State::SignalWait;                            
                    }
                    else {
                        if constexpr(!std::is_same_v<cp, void>) {
                            mState = State::SetupCalibrateOn;
                        }
                        else {
                            mState = State::SetupTestRun;
                        }
                    }
                }
            }
            break;
        case State::DigitalSetupChoose:
            if (const auto param = servo_pa::value(searchCh().toInt() + 1); param) {
                if (const auto newValue = servo_pa::value(searchCh().toInt() + 2); newValue) {
                    const auto p = scaleToN<maxParameter>(param);
                    if (const auto be = button::event(); be == button::Press::Short) {
                        setParam(p, newValue);
                        mState = State::DigitalSetupSet;
                    }
                    else if (be == button::Press::Long) {
                        mState = State::Init;
                    }
                    if (const auto sel = servo_pa::value(searchCh().toInt() + 3); sel) {
                        if (sel.toInt() > chThreshHPos.toInt()) {
                            setParam(p, newValue);
                            mState = State::DigitalSetupSel;
                        }
                        if (sel.toInt() < chThreshHNeg.toInt()) {
                            mState = State::DigitalSetupLeave;
                        }
                    }
                    showParam(p, newValue);
                }
            }
            break;
        case State::DigitalSetupSel:
            if (const auto sel = servo_pa::value(searchCh().toInt() + 3); sel) {
                if (sel.toInt() < chThreshLPos.toInt()) {
                    mState = State::DigitalSetupSet;
                }
            }
            break;
        case State::DigitalSetupLeave:
            if (const auto sel = servo_pa::value(searchCh().toInt() + 3); sel) {
                if (sel.toInt() > chThreshLNeg.toInt()) {
                    mState = State::Init;
                }
            }
            break;
        case State::DigitalSetupSet:
            mStateTick.on(pWaitTicks, []{
                mState = State::DigitalSetupTestRun;
            });
            break;
        case State::DigitalSetupTestRun:
            if (const auto be = button::event(); be == button::Press::Short) {
                mState = State::DigitalSetupChoose;
            }
            else if (be == button::Press::Long) {
                mState = State::Init;
            }
            if (const auto sel = servo_pa::value(searchCh().toInt() + 3); sel) {
                if (sel.toInt() > chThreshHPos.toInt()) {
                    mState = State::DigitalSetupSel2;
                }
                if (sel.toInt() < chThreshHNeg.toInt()) {
                    mState = State::DigitalSetupLeave;
                }
            }
            RunState::led(Esc::runstate());
            break;
        case State::DigitalSetupSel2:
            if (const auto sel = servo_pa::value(searchCh().toInt() + 3); sel) {
                if (sel.toInt() < chThreshLPos.toInt()) {
                    mState = State::DigitalSetupChoose;
                }
            }
            break;
        case State::SetupSetParameter:
            if (const auto be = button::event(); be == button::Press::Short) {
                setParam(mParam, servo_pa::value(searchCh().toInt()));
//                NVM::data().param(mParam);
                mState = State::SetupSetParameterWait;
            }
            else if (be == button::Press::Long) {
                mState = State::SetupEnd;
            }
            else {
                const auto v = scaleToN<maxParameter>(servo_pa::value(searchCh().toInt())).toNaN();
                ValueBlinker::show(v);
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
                }
            }
            break;
        case State::SetupEnd:
            mStateTick.on(pWaitTicks, []{
                mState = State::Init;
            });
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
        case State::SetupCalibrateOn:
            mStateTick.on(stateChangeTicks, []{
                mState = State::SetupCalibrateMes;
            });
            break;
        case State::SetupCalibrateMes:
            mStateTick.on(stateChangeTicks, []{
                mState = State::DigitalSetupChoose;
            });
            break;
        case State::PreRun:
            mStateTick.on(stateChangeTicks, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            RunState::led(Esc::runstate());
            if constexpr(!std::is_same_v<cp, void>) {
                const auto rs = Esc::runstate();
                using v_t = cp::value_type;
                if (Esc::isBackward()) {
                    cp::set(v_t((uint8_t)rs + 3));                
                }
                else {
                    cp::set(v_t((uint8_t)rs));                
                }
            }
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("init"_pgm);
                RunState::reset();
                break;
            case State::SetupWaitButtonRelease:
                etl::outl<terminal>("S WBR"_pgm);
                led(magenta);
                break;
            case State::SetupScan:
                etl::outl<terminal>("S Scan"_pgm);
                led(white);
                break;
            case State::SetupGotChannel:
                etl::outl<terminal>("S Got Ch: "_pgm, searchCh().toInt());
                led(cyan);
                NVM::data().change();
                break;
            case State::DigitalSetupChoose:
                etl::outl<terminal>("S D c"_pgm);
                Esc::event(Esc::Event::Reset);
                led(blue);
                if constexpr(!std::is_same_v<cp, void>) {
                    cp::clear();
                    cp::state(cp::State::Display);
                }
                break;
            case State::DigitalSetupSet:
                etl::outl<terminal>("S D s"_pgm);
                if constexpr(!std::is_same_v<cp, void>) {
                    cp::state(cp::State::Set);
                }
                break;
            case State::DigitalSetupTestRun:
                etl::outl<terminal>("S D r"_pgm);
                Esc::event(Esc::Event::Start);
                if constexpr(!std::is_same_v<cp, void>) {
                    cp::state(cp::State::TestRun);
                    cp::clear();
                }
                break;
            case State::DigitalSetupSel:
                etl::outl<terminal>("S D ts"_pgm);
                break;
            case State::DigitalSetupSel2:
                etl::outl<terminal>("S D ts2"_pgm);
                break;
            case State::DigitalSetupLeave:
                etl::outl<terminal>("S D ls"_pgm);
                break;
            case State::SetupPpmScaleStart:
                etl::outl<terminal>("S ppm ss"_pgm);
                data.mThrMid = (servo_pa::value_type::Upper + servo_pa::value_type::Lower) / 2;
                data.mThrHalf = (servo_pa::value_type::Upper - servo_pa::value_type::Lower) / 2;
                data.mThrMin = chThreshHNeg.toInt();
                data.mThrMax = chThreshHPos.toInt();       
                led(blue);
                break;
            case State::SetupPpmScale1:
                etl::outl<terminal>("S ppm s1"_pgm);
                break;
            case State::SetupPpmScale2:
                etl::outl<terminal>("S ppm s2"_pgm);
                data.mThrMid = (data.mThrMax + data.mThrMin) / 2;
                data.mThrHalf = (data.mThrMax - data.mThrMin) / 2;
                data.change();
                break;
            case State::SetupSetParameter:
                etl::outl<terminal>("S Set P"_pgm);
                Esc::event(Esc::Event::Reset);
                break;
            case State::SetupSetParameterWait:
                etl::outl<terminal>("S Set PW"_pgm);
                led(yellow);
                break;
            case State::SetupTestRun:
                etl::outl<terminal>("S Test"_pgm);
                paramBlink(mParam); 
                Esc::event(Esc::Event::Start);
                break;
            case State::SetupEnd:
                etl::outl<terminal>("S Set End"_pgm);
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
            case State::SetupCalibrateOn:
                Esc::activate();
                Meta::visit<ProviderList>([]<typename W>(W){
                                              using provider = W::type;
                                              provider::calibrateStart();
                                          });
                break;
            case State::SetupCalibrateMes:
                Meta::visit<ProviderList>([]<typename W>(W){
                                              using provider = W::type;
                                              provider::calibrate();
                                          });
                break;
            case State::Run:
                etl::outl<terminal>("run"_pgm);
                Esc::event(Esc::Event::Start);
                if constexpr(!std::is_same_v<cp, void>) {
                    cp::clear();
                    cp::state(cp::State::Run);
                }
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
    template<typename PT>
    static inline void showParam(const PT& p, const value_t& v) {
        static_assert(PT::Upper <= 11);
        if constexpr(!std::is_same_v<cp, void>) {
            cp::param(p);
            cp::set(scaleToN<maxParamValue>(v));
            switch(p) {
            case 0:
                cp::actual(Esc::p_dead());
                break;
            case 1:
                cp::actual(Esc::p_pwmFreq1());
                break;
            case 2:
                cp::actual(Esc::p_pwmFreq2());
                break;
            case 3:
                cp::actual(Esc::p_thr1());
                break;
            case 4:
                cp::actual(Esc::p_thr2());
                break;
            case 5:
                cp::actual(Esc::p_kickThr());
                break;
            case 6:
                cp::actual(Esc::p_kickTicks());
                break;
            case 7:
                cp::actual(Esc::p_pwmLow());
                break;
            case 8: // slowdown
                cp::actual(Esc::p_slowdown());
                break;
            case 9:
                using cpv_t = BusDevs::configProvider::value_type;
//                cpv_t::_;
                cp::actual(cpv_t(data.param(Param::SPortID)));
                break;
            case 10: // NSensors
                cp::actual(cpv_t(data.param(Param::NSensors)));
                break;
            case 11: // Reset
                break;
            }
        }
    }

    template<typename PT>
    static inline void setParam(const PT& p, const value_t& v) {
        static_assert(PT::Upper <= 11);
        const auto sv = scaleToN<maxParamValue>(v);
        etl::outl<terminal>("p: "_pgm, p.toInt(), " sv: "_pgm, sv, " v: "_pgm, v.toInt());
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
        case 8: // slowdown
            Esc::slowdown(sv);
            break;
        case 9:
            NVM::data().param(Param::SPortID) = sv;
            updateSensorId();
            break;
        case 10:
            NVM::data().param(Param::NSensors) = sv;
            if constexpr(External::Bus::isSBus<bus_type>::value || External::Bus::isIBus<bus_type>::value) {
                Sensor::maxProvider(sv + 4);
            }
            break;
        case 11: // Reset
            nvmDefaults<false>();
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
    
    struct ValueBlinker {
        using value_type = etl::uint_ranged_NaN<uint8_t, 0, maxParameter>;
        
        static inline void show(const value_type& v) {
            if (v) {
                if (!mValue || (v.toInt() != mValue.toInt())) {
                    if (v.toInt() == 0) {
                        led(Crgb{5}); // white            
                    }
                    else if (v.toInt() <= 3) {
                        Led::color(yellow);
                        blinkLed::blink(count_type(v.toInt())); 
                    }
                    else if (v.toInt() <= 6) {
                        Led::color(magenta);
                        blinkLed::blink(count_type(v.toInt() - 3)); 
                    }
                    else if (v.toInt() <= 9) {
                        Led::color(cyan);
                        blinkLed::blink(count_type(v.toInt() - 6)); 
                    }
                    mValue = v;
                }
            }
            else {
                mValue.setNaN();
                blinkLed::off();
            }
        }
        private:
        static inline value_type mValue{0};
    };
    
    
    static inline void debug() {
#ifndef NDEBUG
        if (xassert::on) {
//            etl::outl<terminal>("a: "_pgm, assertKey);
            etl::outl<terminal>("a: "_pgm, assertKey, " : "_pgm, xassert::ab);
            xassert::on = false;
            assertKey = 0;
        }
#endif
//        etl::out<terminal>("p: "_pgm, servo_pa::packages(), " r: "_pgm, Rpm::value(), " c: "_pgm, Rpm::captures());
//        etl::out<terminal>(" a0: "_pgm, Adc::value(adc_i_t{0}));
//        etl::out<terminal>(" a1: "_pgm, Adc::value(adc_i_t{1}));
//        etl::out<terminal>(" a2: "_pgm, Adc::value(adc_i_t{2}));
//        etl::out<terminal>(" a3: "_pgm, Adc::value(adc_i_t{3}));
//        etl::out<terminal>(" a4: "_pgm, Adc::value(adc_i_t{4}));
//        etl::out<terminal>(" a5: "_pgm, Adc::value(adc_i_t{5}));
        
//        Meta::visit<ProviderList>([]<typename W>(W){
//                                      etl::out<terminal>(" off: "_pgm, W::type::offset());
//                                  });
//        etl::outl<terminal>(" sch: "_pgm, searchCh().toInt());
        etl::outl<terminal>("min: "_pgm, data.mThrMin, " max: "_pgm, data.mThrMax, " mid: "_pgm, data.mThrMid, " h: "_pgm, data.mThrHalf);
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
            if (rs != oldState) {
                oldState = rs;
                switch(rs) {
                case Esc::RunState::Undefined:
                    blinkLed::off();
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
    
    inline static search_t& searchCh() {
        return NVM::data().mChannel;
    }
    
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mDebugTick;
    static inline External::Tick<Timer> mResetTick;
    static inline External::Tick<Timer> mNvmTick;
};

struct VersionProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::DIY;
    inline static constexpr auto ibus_type = IBus2::Type::type::ARMED;
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
#if defined(GITMAJOR) && defined(GITMINOR)
        static_assert(GITMINOR < 100);
        return GITMAJOR * 100 + GITMINOR;
#else
        return VERSION_NUMBER;
#endif
    }
};

template<uint8_t Number = 11>
struct ConfigProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::DIY2;
    inline static constexpr auto ibus_type = IBus2::Type::type::ARMED;
    enum class State : uint8_t {Display = 1, Set, TestRun, Run};
    using value_type = etl::uint_ranged<uint8_t, 0, 9>;
    using param_type = etl::uint_ranged<uint8_t, 0, Number>;
    
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return (((uint16_t)mState * 100 + mParameter) * 10 + mActual) * 10 + mNew; 
    }
    static inline void param(const param_type& p) {
        mParameter = p;
    }
    static inline void actual(const value_type& a) {
        mActual = a;
    }
    static inline void set(const value_type& v) {
        mNew = v;
    }
    static inline void state(const State& s) {
        mState = s;
    }
    static inline void clear() {
        mParameter.setToBottom();
        mActual.setToBottom();
        mNew.setToBottom();
    }
private:
    static inline State mState{State::Display}; 
    static inline param_type mParameter{}; 
    static inline value_type mActual{}; 
    static inline value_type mNew{}; 
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
    
    using inhPin = Pin<Port<A>, 3>; 
    
    using rpmPin = Pin<Port<F>, 2>; 
    
    // lut1 out: pc3
    using ccl1Position = Portmux::Position<Component::Ccl<1>, Portmux::Default>;
    using lut1 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Usart<1>, Ccl::Input::Mask>;
    
    using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
    
    // WO0: unbenutzt
    // WO1: pwm1
    // WO2: pwm2
    using pwm = PWM::DynamicPwmPrescale<tcaPosition, AVR::Prescaler<4>>;
    static_assert(pwm::f_timer <= 8000000_Hz);
    static_assert(pwm::f_timer >= 1000000_Hz);
    
    using ibt = IBusThrough<daisyChain>;
    
    using adc = Adc<Component::Adc<0>, AVR::Resolution<12>, Vref::V2_048>;
    // ADC Channels: 1:T1, 6:BecI, 7:V+, 19:Text, 20:T2, 21:Curr
    using adcController = External::Hal::AdcController<adc, Meta::NList<1, 6, 7, 19, 20, 21, 0x42>>; // 0x42 = temp
    using adc_i_t = adcController::index_type;
    inline static constexpr adc_i_t setupAdcIndex{3};
    
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
//    using ppmDevPosition = void;

    using evch4 = Event::Channel<4, Event::Generators::Pin<rpmPin>>;
    
    using clockProvider = pwm::clock_provider;
    using rpm = External::Rpm::RpmFreq<evch4, Meta::List<rpmPositionL, rpmPositionH>, clockProvider>;
    
    using rpmOvfChannel = rpm::overflow_channel<0>;
    using rpmRoutes = rpm::event_routes<rpmOvfChannel>;

    using allRoutes = Meta::push_back<rpmRoutes::routes, ppm_user>;
    using evrouter = Event::Router<Event::Channels<evch4, rpmOvfChannel, evch5>, allRoutes>;
    
    using eeprom = EEProm::Controller<Data>;
    
    using portmux = Portmux::StaticMapper<Meta::List<spiPosition, ccl1Position, tcaPosition, servoPosition, sensorPosition, 
    rpmPositionL, rpmPositionH, sppmPosition>>;
    
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
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, 
    AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using escfsm = EscFsm<systemTimer, typename Devs::pwm, typename Devs::inhPin, servo_pa, typename Devs::eeprom>;
    
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    template<typename R>
    struct RpmProvider {
        inline static constexpr auto ibus_type = IBus2::Type::type::RPM;
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
        using value_type = Sensor::value_type;
        inline static constexpr void init() {}
        inline static constexpr void calibrateStart() {
            Sensor::template pullup<true>();
        }
        inline static constexpr void calibrate() {
            mActive = !Sensor::raw().isTop();
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
        inline static constexpr uint16_t offset() {
            return mActive;
        }
    private:
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
        inline static constexpr void calibrateStart() {}
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

    using adcController = devs::adcController;
    
    using temp1 = External::AnalogSensor<typename devs::adcController, 0, std::ratio<500,1000>, 
    std::ratio<10, 1000>, 
    std::ratio<10,1>>;
    using temp2 = External::AnalogSensor<adcController, 4, std::ratio<500,1000>, 
    std::ratio<10, 1000>, 
    std::ratio<10,1>>;
    using text = External::AnalogSensor<adcController, 3, std::ratio<500,1000>, 
    std::ratio<10, 1000>, 
    std::ratio<10,1>>;
    
    using vdiv = External::AnalogSensor<adcController, 2, std::ratio<0,1>, 
    std::ratio<R2vd, R2vd + R1vd>, 
    std::ratio<100,1>>;
    
    using bec1S = External::AnalogSensor<adcController, 1, std::ratio<0,1>, 
    std::ratio<650, 1000>, 
    std::ratio<100,1>>;
    
    using currS = External::AnalogSensor<adcController, 5, std::ratio<0,1>, 
    std::ratio<550, 1000>, 
    std::ratio<1000,1>>;
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 6, typename Devs::sigrow>;
    using temp1P = TempProvider<temp1>;
    using temp2P = TempProvider<temp2>;
    using textP = TempProvider<text>;
    
    using voltageP = VoltageProvider<vdiv>;
    using cBecP = CurrentProvider<bec1S>;
    
    using currP = CurrentProvider<currS>;
    using rpmP = RpmProvider<typename Devs::rpm>;
    
    using calibratePs = Meta::List<currP, textP, temp1P, temp2P>;

    using configProvider = ConfigProvider<>;
    
    using sensor = IBus2::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<115200>, 
                                Meta::List<configProvider, voltageP, currP, rpmP, textP, cBecP,  
                                           temp1P, temp2P, tempiP, VersionProvider>, 
                                systemTimer, typename devs::ibt>;
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    static inline uint8_t magic = 43;
    using bus_type = External::Bus::SBusSPort<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using escfsm = EscFsm<systemTimer, typename Devs::pwm, typename Devs::inhPin, servo_pa, typename Devs::eeprom>;
    
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    template<typename PA>
    using sensorUsart = AVR::Usart<typename Devs::sensorPosition, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
    
    template<typename R>
    struct RpmProvider {
        inline static constexpr auto valueId = External::SPort::ValueId::Rpm;
        inline static constexpr void init() {
        }
        inline static constexpr uint32_t value() {
            if (const auto rpm = R::value(); rpm) {
                return rpm.value();
            }
            return 0;
        }
    };
    
    template<typename Sensor>
    struct CurrentProvider {
        inline static constexpr auto valueId = External::SPort::ValueId::Current;
        inline static constexpr void init() {}
        inline static constexpr void calibrateStart() {}
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

    template<typename ADC, uint8_t Channel, typename SigRow>
    struct InternalTempProvider {
        using index_t = ADC::index_type;
        static_assert(Channel <= index_t::Upper);
        inline static constexpr auto channel = index_t{Channel};
        inline static constexpr auto valueId = External::SPort::ValueId::Temp2;
        inline static constexpr void init() {}
        inline static constexpr uint32_t value() {
            return SigRow::template adcValueToTemperature<std::ratio<1,1>, 0, typename ADC::VRef_type>(ADC::value(channel)).value;
        }
    };

    template<typename Sensor>
    struct TempProvider {
        inline static constexpr auto valueId = External::SPort::ValueId::Temp1;
        using value_type = Sensor::value_type;
        inline static constexpr void init() {}
        inline static constexpr void calibrateStart() {
            Sensor::template pullup<true>();
        }
        inline static constexpr void calibrate() {
            mActive = !Sensor::raw().isTop();
            Sensor::template pullup<false>();
        }
        inline static constexpr uint32_t value() {
            if (mActive) {
                return Sensor::value();
            }
            else {
                return 0;
            }
        }
        inline static constexpr uint16_t offset() {
            return mActive;
        }
    private:
        inline static bool mActive{false};
    };

    template<typename Sensor>
    struct VoltageProvider {
        inline static constexpr auto valueId = External::SPort::ValueId::Voltage;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            return Sensor::value();
        }
    };

    using adcController = devs::adcController;
    
    using temp1 = External::AnalogSensor<adcController, 0, std::ratio<500,1000>, 
    std::ratio<10, 1000>, 
    std::ratio<1,1>>;
    using temp2 = External::AnalogSensor<adcController, 4, std::ratio<500,1000>, 
    std::ratio<10, 1000>, 
    std::ratio<1,1>>;
    using text = External::AnalogSensor<adcController, 3, std::ratio<500,1000>, 
    std::ratio<10, 1000>, 
    std::ratio<1,1>>;
    
    using vdiv = External::AnalogSensor<adcController, 2, std::ratio<0,1>, 
    std::ratio<R2vd, R2vd + R1vd>, 
    std::ratio<100,1>>;
    
    using bec1S = External::AnalogSensor<adcController, 1, std::ratio<0,1>, 
    std::ratio<650, 1000>, 
    std::ratio<10,1>>;
    
    using currS = External::AnalogSensor<adcController, 5, std::ratio<0,1>, 
    std::ratio<550, 1000>, 
    std::ratio<100,1>>;
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 6, typename Devs::sigrow>;
    using temp1P = TempProvider<temp1>;
    using temp2P = TempProvider<temp2>;
    using textP = TempProvider<text>;
    
    using voltageP = VoltageProvider<vdiv>;
    using cBecP = CurrentProvider<bec1S>;
    using currP = CurrentProvider<currS>;
    using rpmP = RpmProvider<typename Devs::rpm>;

    using configProvider = ConfigProvider<>;
    
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, 
                                           Meta::List<configProvider, VersionProvider, temp1P, temp2P, tempiP, textP, voltageP, cBecP, currP, rpmP>>;

    using calibratePs = Meta::List<currP, textP, temp1P, temp2P>;
};

struct ThrottleMenu final : public Hott::Menu<8, false, 4> {
    struct Adapter {
        uint8_t& operator[](const uint8_t) {
            return mD;
        }
        void change() {
        }
        void select(const uint8_t) {
        }
        uint8_t mD{};
    };
    
    ThrottleMenu(auto* const parent) : Menu{parent, "Throttle"_pgm, &mPwm0, &mPwm1, &mPwm2, &mPwm3} {}    
    Hott::TextWithValue<uint8_t, Adapter, 3, void, 5> mPwm0{"PWM HF:"_pgm, a0, 0, 9};
    Hott::TextWithValue<uint8_t, Adapter, 3, void, 5> mPwm1{"PWM LF:"_pgm, a1, 1, 9};
    Hott::TextWithValue<uint8_t, Adapter, 3, void, 5> mPwm2{"Throttle 1:"_pgm, a2, 2, 9};
    Hott::TextWithValue<uint8_t, Adapter, 3, void, 5> mPwm3{"Throttle 2:"_pgm, a3, 3, 9};
    
    Adapter a0;
    Adapter a1;
    Adapter a2;
    Adapter a3;
};

struct KickMenu final : public Hott::Menu<8, false, 3> {
    struct Adapter {
        uint8_t& operator[](const uint8_t) {
            return mD;
        }
        void change() {
        }
        void select(const uint8_t) {
        }
        uint8_t mD{};
    };
    
    KickMenu(auto* const parent) : Menu{parent, "Kickstart"_pgm, &mPwm0, &mPwm1, &mPwm2} {}    
    Hott::TextWithValue<uint8_t, Adapter, 3, void, 5> mPwm0{"K Throttle:"_pgm, a0, 0, 9};
    Hott::TextWithValue<uint8_t, Adapter, 3, void, 5> mPwm1{"K Duration:"_pgm, a1, 1, 9};
    Hott::TextWithValue<uint8_t, Adapter, 3, void, 5> mPwm2{"Throttle Low:"_pgm, a2, 2, 9};
    
    Adapter a0;
    Adapter a1;
    Adapter a2;
};

template<typename CB, auto ValueWidth = 7>
class Button final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
public:
    static inline constexpr uint8_t valueBeginColumn = Hott::BufferString::size() - ValueWidth;
    static inline constexpr uint8_t valueWidth = ValueWidth;
    
    using value_span_type = etl::span<valueWidth, etl::Char>;
    
    Button(const AVR::Pgm::StringView& text) : mTitle{text} {}
    
    void valueToText(value_span_type buffer) const {
        buffer.insertLeft("<press>"_pgm);
    }
    
    virtual void putTextInto(Hott::BufferString& buffer) const override {
        buffer[0] = etl::Char{' '};
        buffer.insertAtFill(1, mTitle);
        valueToText(etl::make_span<valueBeginColumn, valueWidth>(buffer));
        if (mSelected) {
            etl::apply(etl::make_span<valueBeginColumn, valueWidth>(buffer), [](auto& c) {c |= etl::Char{0x80};});
        }
    }
    virtual MenuItem* processKey(const Hott::key_t key) override {
        switch (key) {
        case Hott::key_t::up:
        case Hott::key_t::down:
        case Hott::key_t::left:
        case Hott::key_t::right:
        case Hott::key_t::nokey:
            break;
        case Hott::key_t::set:
            if (mSelected) {
                CB::process();
            }
            mSelected = !mSelected;
            break;
        }
        return this;
    }
private:
    const AVR::Pgm::StringView mTitle;
};

template<typename GFsm>
struct RCMenu final : public Hott::Menu<8, true, 6> {
    RCMenu() : Hott::Menu<8, true, 6>{nullptr, HOTT_TITLE, &mDead, &mSlow, &mSystem, &mKick, &mReset} {}
private:
    using esc = GFsm::Esc;
    using eeprom = GFsm::devs::eeprom;
    
    struct ResetAdapter {
        static inline void process() {
            GFsm::nvmDefaults();
        }
    };
    struct Adapter {
        auto& operator[](const uint8_t p) {
            switch(p) {
            case 0:
                mD[0] = esc::p_dead();
                break;
            case 1:
                mD[1] = esc::p_slowdown();
                break;
            }
            return mD[0];
        }
        void change() {
            esc::dead(mD[0]);
            esc::slowdown(mD[1]);
            eeprom::data().change();
        }
        void select(const uint8_t) {
//            lastSelected = p;
        }
    private:
//        uint8_t lastSelected{};
//        etl::uint_ranged<uint8_t, 0, 9> lastState{};
        std::array<etl::uint_ranged<uint8_t, 0, 9>, 2> mD{};
    };
    Adapter a;
    Hott::TextWithValue<uint8_t, Adapter, 3, void, 5> mDead{"Deadband:"_pgm, a, 0, 9};
    Hott::TextWithValue<uint8_t, Adapter, 3, void, 5> mSlow{"Slowdown:"_pgm, a, 1, 9};
    ThrottleMenu mSystem{this};
    KickMenu mKick{this};
    Button<ResetAdapter> mReset{"Reset"_pgm};
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
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using escfsm = EscFsm<systemTimer, typename Devs::pwm, typename Devs::inhPin, servo_pa, typename Devs::eeprom>;
    
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif

    using configProvider = void;
    
    using sensor = Hott::Experimental::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;

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
    
    using escfsm = EscFsm<systemTimer, typename Devs::pwm, typename Devs::inhPin, servo_pa, typename Devs::eeprom>;
    
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

    using configProvider = void;
    
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

            etl::outl<terminal>("esc10_avr128_hw03"_pgm);
            
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

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    xassert::ab.clear();
    xassert::ab.insertAt(0, expr);
    etl::itoa(line, xassert::aline);
    xassert::ab.insertAt(20, xassert::aline);
    xassert::ab.insertAt(30, file);
    xassert::on = true;
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
