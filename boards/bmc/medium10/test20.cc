//#define NDEBUG

//#define USE_IBUS

#define USE_SBUS
#ifdef USE_SBUS
# define USE_SPORT
#endif

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
#include <external/ibus/ibus.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/experimental/sensor.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/apa102.h>
#include <external/solutions/series01/sppm_in.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
#ifdef USE_HOTT
    constexpr auto fRtc = 500_Hz;
#endif
#ifdef USE_SBUS
    constexpr auto fRtc = 1000_Hz;
#endif
#ifdef USE_IBUS
    constexpr auto fRtc = 1000_Hz;
#endif
    constexpr uint16_t R1vd = 10'000;
    constexpr uint16_t R2vd = 1'200;
}

namespace Input {
    struct SBus;
    struct IBus;
    struct SumD;
    struct Sppm;
}

struct Data final : public EEProm::DataBase<Data> {
    auto magic() const {
        return mMagic;
    }
    void clear() {
        mMagic = 42;    
    }
private:
    uint8_t mMagic{};
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
    inline static External::Crgb mColor;
    
};

template<typename R>
struct RpmProvider {
#ifdef FS_I6S    
    inline static constexpr auto ibus_type = IBus::Type::type::RPM_FLYSKY;
#else
    inline static constexpr auto ibus_type = IBus::Type::type::RPM;
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
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
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
    inline static constexpr auto ibus_type = IBus::Type::type::EXTERNAL_VOLTAGE;
    inline static constexpr void init() {}
    
    inline static constexpr uint16_t value() {
        return Sensor::value();
    }
};
template<typename Sensor>
struct CurrentProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::BAT_CURR;
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
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return SigRow::template adcValueToTemperature<std::ratio<1,10>, 40, typename ADC::VRef_type>(ADC::value(channel)).value;
    }
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

template<typename Timer> 
struct TimeRegulator {
#ifdef USE_IBUS
    static constexpr External::Tick<Timer> deltaTicks{40_ms}; 
#endif
#ifdef USE_SBUS
    static constexpr External::Tick<Timer> deltaTicks{25_ms}; 
#endif
    static inline uint16_t value(const uint16_t& v) {
        mTarget = v;
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
    static inline void off(const uint16_t& ov) {
        mActual = ov;
        mTarget = ov;
    }
    static inline void slow(bool on) {
        if (on) {
            mMaxDiff = 2;
        }
        else {
            mMaxDiff = 10;
        }
    }
private:
    static inline External::Tick<Timer> deltaTick;
    static inline uint16_t mMaxDiff{4};
    static inline uint16_t mTarget{};
    static inline uint16_t mActual{};
};


template<typename Timer, typename PWM, typename InhPinList, typename PA>
struct EscFsm {
    using timeRegulator  = TimeRegulator<Timer>;
    
    enum class State : uint8_t {Undefined = 0, Init, 
                                Off = 10, OffForwardWait, OffBackwardWait, 
                                Forward = 20, ForwardWait, ForwardKick,  
                                Backward = 30, BackwardWait};
    
    enum class RunState : uint8_t {Undefined = 0, Off, Low, Medium, High};
    
    enum class Event : uint8_t {None, Start, Reset};
    
    using throttle_type = typename PA::value_type;
    using thr_t = typename throttle_type::value_type;
    //    thr_t::_;
    
    using pwm_t = PWM::value_type;
    //    pwm_t::_;
    
    static inline constexpr thr_t thrMax{throttle_type::Upper};
    static inline constexpr thr_t thrMin{throttle_type::Lower};
    static inline constexpr thr_t thrMedium{(throttle_type::Upper + throttle_type::Lower) / 2};
    static inline constexpr thr_t thrHalf{(throttle_type::Upper - throttle_type::Lower) / 2};
    
    static inline constexpr thr_t thrStartForward{thrMedium  + 20};
    static inline constexpr thr_t thrStartBackward{thrMedium - 20};
    
    static constexpr External::Tick<Timer> initTicks{100_ms}; 
    static constexpr External::Tick<Timer> deadTicks{100_ms}; // fw <-> bw
    static constexpr External::Tick<Timer> startTicks{10_ms}; 
    static constexpr External::Tick<Timer> kickTicks{200_ms}; 
    
    inline static void init() {
        PWM::init();
        PWM::period(mPwmPeriodMin);
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
        return mThrottle && (mThrottle.toInt() >= thrStartForward);
    }
    inline static bool isThrottleBackward() {
        return mThrottle && (mThrottle.toInt() <= thrStartBackward);
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
                mState = State::Forward;
            });
            if (!isThrottleForward()) {
                mState = State::Off;
            }
            break;
        case State::ForwardKick:
            mStateTicks.on(kickTicks, []{
                mState = State::Forward;
            });
        {
            const auto tv = thrMedium + thrHalf / 10;
            timeRegulator::off(tv);
            mTv = tv;
            PWM::period(pwmPeriod(tv));
            const auto d = etl::distance(tv, thrStartForward);
            const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax - thrStartForward}, etl::Intervall{pwm_t{0}, PWM::max()});
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
                mTv = tv;
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartForward);
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax - thrStartForward}, etl::Intervall{pwm_t{0}, PWM::max()});
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
            else if (const auto tv = timeRegulator::value(mThrottle.toInt()); tv > thrStartForward) {
                mTv = tv;
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartForward);
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax - thrStartForward}, etl::Intervall{pwm_t{0}, PWM::max()});
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
                mTv = tv;
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartBackward);
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrStartBackward - thrMin}, etl::Intervall{pwm_t{0}, PWM::max()});
                PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(pv);
            }
            break;
        case State::OffBackwardWait:
            if (isThrottleBackward()) {
                mState = State::Backward;
            }
            else if (const auto tv = timeRegulator::value(mThrottle.toInt()); tv < thrStartBackward) {
                mTv = tv;
                PWM::period(pwmPeriod(tv));
                const auto d = etl::distance(tv, thrStartBackward);
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrStartBackward - thrMin}, etl::Intervall{pwm_t{0}, PWM::max()});
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
    //    inline static PWM::value_type mPwmPeriod{2500};
    inline static PWM::value_type mPwmPeriodMin{2500};
    inline static PWM::value_type mPwmPeriodMax{65535};
    
    inline static thr_t throttle_delta_1{thrHalf / 3};
    inline static thr_t throttle_delta_2{3 * thrHalf / 4};
    
    inline static uint16_t pwmPeriod(const uint16_t& v) {
        const auto d = etl::distance(v, thrMedium);
        if (d < throttle_delta_1) {
            mRunState = RunState::Low;
        }
        else if (d > throttle_delta_2) {
            mRunState = RunState::High;
        }
        else {
            mRunState = RunState::Medium;
        }
        return etl::scale(d, etl::Intervall{throttle_delta_1, throttle_delta_2}, etl::Intervall{mPwmPeriodMin, mPwmPeriodMax});
    }
    static inline RunState mRunState{RunState::Undefined};
    static inline uint16_t mTv{};
    static inline throttle_type mThrottle;
    inline static External::Tick<Timer> mStateTicks;
    inline static State mState{State::Undefined};
    inline static Event mEvent{Event::None};
};

template<typename Kind,
         typename Timer, typename Servo, typename Led, typename Esc, 
         typename Sensor, typename Lut, 
         typename Adc, typename Rpm,
         typename NVM, typename ProviderList,
         typename Term = void>
struct GlobalFsm {
    using blinkLed = External::Blinker2<Led, Timer, 100_ms, 2000_ms>;
    using count_type = blinkLed::count_type;
    
    using pa = Servo::protocoll_adapter_type;
    
    using TermDev = Term::device_type;
    
    using adc_i_t = Adc::index_type;
    
    enum class State : uint8_t {Undefined, Init, SignalWait, PreCheckStart, CheckStart, 
                                CalibrateOn, CalibrateMes, PreRun, Run};
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> stateChangeTicks{500_ms};
    static constexpr External::Tick<Timer> debugTicks{500_ms};
    static constexpr External::Tick<Timer> nvmTicks{1000_ms};
    static constexpr External::Tick<Timer> rpmResetTicks{250_ms};
    static constexpr External::Tick<Timer> learnTimeoutTicks{4000_ms};
    
    static inline auto& appData = NVM::data();
    
    inline static void init() {
        if constexpr(std::is_same_v<Kind, Input::IBus>) {
            if constexpr (std::is_same_v<Servo, TermDev>) {
                Servo::template init<AVR::BaudRate<115200>>();            
            }
            else {
                Servo::template init<AVR::BaudRate<115200>>();
                if constexpr(!std::is_same_v<TermDev, void>) {
                    TermDev::template init<AVR::BaudRate<9600>>();
                }
            }
        }
        else if constexpr(std::is_same_v<Kind, Input::SBus>) {
            if constexpr (std::is_same_v<Servo, TermDev>) {
                Servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
                Servo::rxInvert(true);
            }
            else {
                Servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
                Servo::rxInvert(true);
                if constexpr(!std::is_same_v<TermDev, void>) {
                    TermDev::template init<AVR::BaudRate<9600>>();
                }
            }
        }
        else if constexpr(std::is_same_v<Kind, Input::SumD>) {
        }
        else if constexpr(std::is_same_v<Kind, Input::Sppm>) {
        }
        else {
            static_assert(std::false_v<Kind>);
        }
        
        NVM::init();
        if (!((appData.magic() == 42))) {
            appData.clear();
            appData.change();
        }            
        
        blinkLed::init();
        Esc::init(); 
        
        if constexpr(std::is_same_v<Kind, Input::SBus>) {
            Lut::init(std::byte{0x33}); // route TXD to lut1-out no-inv
            Sensor::init();
            Sensor::uart::txPinDisable();
        }
        else if constexpr(std::is_same_v<Kind, Input::IBus>) {
            Lut::init(std::byte{0x00}); 
            Sensor::init();
            Sensor::uart::txOpenDrain();
        }
        Adc::template init<false>(); // no pullups
        Adc::mcu_adc_type::nsamples(4);
        
        Rpm::init();
    }
    inline static void periodic() {
        NVM::saveIfNeeded([&]{
            etl::outl<Term>("save eep"_pgm);
        });
        Servo::periodic();
        Led::periodic();
        Sensor::periodic();
        Adc::periodic();
    }
    inline static void ratePeriodic() {
        if constexpr(std::is_same_v<Kind, Input::SBus>) {
            pa::ratePeriodic();
        }
        blinkLed::ratePeriodic();
        Sensor::ratePeriodic();
        Esc::ratePeriodic();
        const auto oldState = mState;
        ++mStateTick;
        
        (++mNvmTick).on(nvmTicks, appData.expire);
        (++mDebugTick).on(debugTicks, debug);
        (++mResetTick).on(rpmResetTicks, []{
            Rpm::reset();
            if (pa::packages() == 0) {
                mState = State::Undefined;
                RunState::reset();
                Esc::event(Esc::Event::Reset);
            }
            pa::resetStats();
        });
        
        switch(mState) {
        case State::Undefined:
            mStateTick.on(startupTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(stateChangeTicks, []{
                mState = State::SignalWait;
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
                etl::outl<Term>("init"_pgm);
                led(red);
                break;
            case State::SignalWait:
                etl::outl<Term>("sigw"_pgm);
                led(yellow);
                break;
            case State::CheckStart:
                etl::outl<Term>("check"_pgm);
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
                etl::outl<Term>("run"_pgm);
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
    static inline void debug() {
        etl::out<Term>("p: "_pgm, pa::packages(), " r: "_pgm, Rpm::value(), " c: "_pgm, Rpm::captures());
        //        etl::out<Term>(" a0: "_pgm, Adc::value(adc_i_t{0}));
        //        etl::out<Term>(" a1: "_pgm, Adc::value(adc_i_t{1}));
        //        etl::out<Term>(" a2: "_pgm, Adc::value(adc_i_t{2}));
        //        etl::out<Term>(" a3: "_pgm, Adc::value(adc_i_t{3}));
        //        etl::out<Term>(" a4: "_pgm, Adc::value(adc_i_t{4}));
        //        etl::out<Term>(" a5: "_pgm, Adc::value(adc_i_t{5}));
        
        Meta::visit<ProviderList>([]<typename W>(W){
                                      etl::out<Term>(" off: "_pgm, W::type::offset());
                                  });
        
        etl::outl<Term>(" a6: "_pgm, Adc::value(adc_i_t{6}));
        //        etl::out<Term>(" ga: "_pgm, SigRow<>::tgain());
        //        etl::out<Term>(" of: "_pgm, SigRow<>::toffset());
        //        etl::outl<Term>(" a7: "_pgm, Adc::value(adc_i_t{7}));
        Esc::template debug<Term>();
    } 
    
    using Crgb = External::Crgb;
    static constexpr Crgb red{External::Red{255}, External::Green{0}, External::Blue{0}};
    static constexpr Crgb green{External::Red{0}, External::Green{255}, External::Blue{0}};
    static constexpr Crgb blue{External::Red{0}, External::Green{0}, External::Blue{255}};
    static constexpr Crgb yellow{External::Red{255}, External::Green{255}, External::Blue{0}};
    static constexpr Crgb magenta{External::Red{255}, External::Green{0}, External::Blue{255}};
    static constexpr Crgb cyan{External::Red{0}, External::Green{255}, External::Blue{255}};
    
    struct RunState {
        inline static void led(const Esc::RunState rs) {
            if (rs != oldState) {
                oldState = rs;
                switch(rs) {
                case Esc::RunState::Undefined:
                    break;
                case Esc::RunState::Off:
                    etl::outl<Term>("rs off"_pgm);
                    Led::color(green);
                    blinkLed::blink(count_type{1});
                    break;
                case Esc::RunState::Low:
                    etl::outl<Term>("rs low"_pgm);
                    Led::color(green);
                    blinkLed::steady();
                    break;
                case Esc::RunState::Medium:
                    etl::outl<Term>("rs med"_pgm);
                    Led::color(yellow);
                    blinkLed::steady();
                    break;
                case Esc::RunState::High:
                    etl::outl<Term>("rs high"_pgm);
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
    
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mDebugTick;
    static inline External::Tick<Timer> mResetTick;
    static inline External::Tick<Timer> mNvmTick;
};

struct VersionProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::Temp1;
    inline static constexpr auto ibus_type = IBus::Type::type::ARMED;
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return 1012;
    }
};

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

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Sensor
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Servo / Debug

// lut1 out: pc3
using ccl1Position = Portmux::Position<Component::Ccl<1>, Portmux::Default>;
using lut1 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Usart<1>, Ccl::Input::Mask>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;

// WO0: unbenutzt
// WO1: pwm1
// WO2: pwm2
using pwm = PWM::DynamicPwmPrescale<tcaPosition, AVR::Prescaler<4>>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

using ibt = IBusThrough<daisyChain>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<12>, Vref::V2_048>;
// ADC Channels: T1, BecI1, BecI2, V+, Text, T2, Curr
using adcController = External::Hal::AdcController<adc, Meta::NList<0, 1, 6, 7, 19, 20, 21, 0x42>>; // 0x42 = temp

using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;
using spi = AVR::Spi<spiPosition, AVR::QueueLength<16>,  AVR::UseInterrupts<false>>;
using ledStripe = External::LedStripe<spi, External::APA102, 1>;
using led = ColorLedAdapter<ledStripe>;

using rpmPositionL = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using rpmPositionH = Portmux::Position<Component::Tcb<1>, Portmux::Default>;

using sppmPosition = Portmux::Position<Component::Tcb<2>, Portmux::Default>;

using evch4 = Event::Channel<4, Event::Generators::Pin<rpmPin>>;

using clockProvider = pwm::clock_provider;
using rpm = External::Rpm::RpmFreq<evch4, Meta::List<rpmPositionL, rpmPositionH>, clockProvider>;

using rpmOvfChannel = rpm::overflow_channel<0>;
using rpmRoutes = rpm::event_routes<rpmOvfChannel>;

using evrouter = Event::Router<Event::Channels<evch4, rpmOvfChannel>, rpmRoutes::routes>;

using tempiP = InternalTempProvider<adcController, 7, sigrow>;
 
using temp1 = External::AnalogSensor<adcController, 0, std::ratio<500,1000>, 
std::ratio<10, 1000>, 
std::ratio<10,1>>;
using temp2 = External::AnalogSensor<adcController, 5, std::ratio<500,1000>, 
std::ratio<10, 1000>, 
std::ratio<10,1>>;
using text = External::AnalogSensor<adcController, 4, std::ratio<500,1000>, 
std::ratio<10, 1000>, 
std::ratio<10,1>>;
using temp1P = TempProvider<temp1>;
using temp2P = TempProvider<temp1>;
using textP = TempProvider<text>;

using vdiv = External::AnalogSensor<adcController, 3, std::ratio<0,1>, 
std::ratio<R2vd, R2vd + R1vd>, 
std::ratio<100,1>>;
using voltageP = VoltageProvider<vdiv>;

using bec1S = External::AnalogSensor<adcController, 1, std::ratio<0,1>, 
std::ratio<1900, 1000>, 
std::ratio<100,1>>;
using cBecP = CurrentProvider<bec1S>;

using currS = External::AnalogSensor<adcController, 6, std::ratio<0,1>, 
std::ratio<550, 1000>, 
std::ratio<1000,1>>;
using currP = CurrentProvider<currS>;

using rpmP = RpmProvider<rpm>;

using eeprom = EEProm::Controller<Data>;

using portmux = Portmux::StaticMapper<Meta::List<spiPosition, ccl1Position, tcaPosition, usart1Position, usart2Position, rpmPositionL, rpmPositionH>>;

template<typename A>
struct AppCommon {
    
    using syscfg = AVR::Cpu::SysCfg<>;
    
    inline static void run() {
#ifndef NDEBUG
        timing0Pin::dir<Output>();
        timing1Pin::dir<Output>();
        timing2Pin::dir<Output>();
#endif
       
        portmux::init();
        ccp::unlock([]{
            clock::init<Project::Config::fMcuMhz>();
        });
        systemTimer::init();
        
        inh1Pin::dir<Output>();
        inh2Pin::dir<Output>();
        
        evrouter::init();
        A::gfsm::init();
         
        {
            etl::outl<typename A::terminal>("test20 rev: "_pgm, syscfg::revision(), " id: "_pgm, sigrow::id()); 
            
            while(true) {
                timing0Pin::toggle();
                
                A::gfsm::periodic();
                systemTimer::periodic([&]{
                    A::gfsm::ratePeriodic();
                });
            }
        }
    }
    
    
};

template<typename Kind>
struct App : AppCommon<App<Kind>> {};

template<>
struct App<Input::SBus> : AppCommon<App<Input::SBus>> {
    using kind = Input::SBus;

    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = AVR::Usart<usart2Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;
    
    #ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
    #else
    using terminal = etl::basic_ostream<void>;
    #endif
    
    template<typename PA>
    using sensorUsart = AVR::Usart<usart1Position, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID3, sensorUsart, systemTimer, 
    Meta::List<VersionProvider>>;

    using escfsm = EscFsm<systemTimer, pwm, Meta::List<inh1Pin, inh2Pin>, servo_pa>;
    
    using gfsm = GlobalFsm<kind, systemTimer, servo, led, escfsm, sensor, lut1, adcController, rpm, eeprom, Meta::List<currP, textP, temp1P>, terminal>;
};

template<>
struct App<Input::IBus> : AppCommon<App<Input::IBus>> {
    using kind = Input::IBus;

    using servo_pa = IBus::Servo::ProtocollAdapter<0>;
    using servo = Usart<usart2Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<1024>>;
    
    #ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
    #else
    using terminal = etl::basic_ostream<void>;
    #endif
    
    using sensor = IBus::Sensor<usart1Position, AVR::Usart, AVR::BaudRate<115200>, 
    Meta::List<VersionProvider, 
    temp1P, temp2P, tempiP, textP, 
    voltageP, cBecP, currP, rpmP>, 
    systemTimer, ibt
    //                          , etl::NamedFlag<true>
    //                           , etl::NamedFlag<true>
    >;
    using escfsm = EscFsm<systemTimer, pwm, Meta::List<inh1Pin, inh2Pin>, servo_pa>;
    

    using gfsm = GlobalFsm<kind, systemTimer, servo, led, escfsm, sensor, lut1, adcController, rpm, eeprom, Meta::List<currP, textP, temp1P>, terminal>;
};

template<>
struct App<Input::SumD> : AppCommon<App<Input::SumD>> {
    using kind = Input::SumD;

    using servo_pa = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
    using servo = Usart<usart2Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<1024>>;
    
    #ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
    #else
    using terminal = etl::basic_ostream<void>;
    #endif
    
    using sensor = Hott::Experimental::Sensor<usart1Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;
    using escfsm = EscFsm<systemTimer, pwm, Meta::List<inh1Pin, inh2Pin>, servo_pa>;
    
    using gfsm = GlobalFsm<kind, systemTimer, servo, led, escfsm, sensor, lut1, adcController, rpm, eeprom, Meta::List<currP, textP, temp1P>, terminal>;
};

template<>
struct App<Input::Sppm> : AppCommon<App<Input::Sppm>> {
    using kind = Input::Sppm;
    
    using dbg = Usart<usart2Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<1024>>;
    
    #ifndef NDEBUG
    using terminal = etl::basic_ostream<dbg>;
    #else
    using terminal = etl::basic_ostream<void>;
    #endif
    
    using sensor = IBus::Sensor<usart1Position, AVR::Usart, AVR::BaudRate<115200>, 
    Meta::List<VersionProvider, 
    temp1P, temp2P, tempiP, textP, 
    voltageP, cBecP, currP, rpmP>, 
    systemTimer, ibt
    //                          , etl::NamedFlag<true>
    //                           , etl::NamedFlag<true>
    >;

    using sppm_input = External::Ppm::SinglePpmIn<sppmPosition::component_type, clockProvider>;
     
    using servo = External::Ppm::Adapter<sppm_input>;
    
    using escfsm = EscFsm<systemTimer, pwm, Meta::List<inh1Pin, inh2Pin>, servo::protocoll_adapter_type>;
    
    using gfsm = GlobalFsm<kind, systemTimer, servo, led, escfsm, sensor, lut1, adcController, rpm, eeprom, Meta::List<currP, textP, temp1P>, terminal>;
};

uint8_t a = 0;

int main() {
    if (a == 0) {
        App<Input::SBus>::run();
    }
    else if (a == 1) {
        App<Input::IBus>::run();
    }
    else if (a == 2) {
        App<Input::SumD>::run();
    }
    else if (a == 3) {
        App<Input::Sppm>::run();
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
//    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
        assertPin::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
