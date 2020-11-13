#define USE_IBUS

#define DEBUG2 // RX/TX change -> full duplex (man muss dann Ibus-Input und Telemetrie tauschen)

// analogInput fuer feedbach: 120K/2µ2F RC Tiefpass

#include "board.h"

#include <etl/control.h>

template<typename PPM, typename ADC, typename Timer>
struct Servo {
    enum class State : uint8_t {Undefined = 0, Init, 
                                CalibrateADStart = 10, CalibrateADRun, CalibrateADTest, CalibrateADFinish,
                                CalibrateDeadStartP = 20, CalibrateDeadSetP, CalibrateDeadTestP,
                                CalibrateDeadStartN = 30, CalibrateDeadSetN, CalibrateDeadTestN,
                                NeutralStart = 40, NeutralReachForward, NeutralReachBackward, Neutral,
                                Stop = 50, Forward, Backward};
    enum class Event : uint8_t {None, Reset, Calibrate, Run};

    static constexpr External::Tick<Timer> settleTicks{40_ms};
    static constexpr External::Tick<Timer> adTestTicks{300_ms};
    
    inline static Event mEvent{Event::None};

//    using fpq10 = etl::FixedPoint<int16_t, 10>;
////    static inline Control::PID<int16_t, fpq10> pid{fpq10{1.0}, fpq10{0.0}, fpq10{2.0}, 100, 100};
//    static inline Control::PID<int16_t, float> pid{2.0, 0.0001, 20.0, 100, 200};

    inline static bool mReady{false};
    
    inline static bool ready() {
        return mReady;
    }
    
    inline static Event event() {
        Event e{Event::None};
        swap(mEvent, e);
        return e;
    }    
    
    static inline void reset() {
        mReady = false;
        mEvent = Event::Reset;
    }
    static inline void calibrate() {
        mReady = false;
        mEvent = Event::Calibrate;        
    }
    static inline void run() {
        mEvent = Event::Run;
    }
    
    template<auto L, auto U>
    static inline void position(const etl::uint_ranged_NaN<uint16_t, L, U>& pv) {
        if (pv) {
            const uint16_t sv = pv.toInt();
            targetPos = amin + ((uint32_t)(sv - L) * (amax - amin)) / (U - L);
        }
    }
    
    using ppm_t = PPM::ranged_type;
    using adc_t = ADC::value_type;
    
    inline static constexpr int16_t ppm_mid = (ppm_t::Upper + ppm_t::Lower) / 2;
    inline static constexpr int16_t ppm_delta = (ppm_t::Upper - ppm_t::Lower) / 2;
    
//    std::integral_constant<uint16_t, ppm_mid>::_;
//    std::integral_constant<uint16_t, ppm_delta>::_;
    
    inline static adc_t amin{512};
    inline static adc_t amax{512};
    inline static adc_t aminLast{512};
    inline static adc_t amaxLast{512};
    inline static int16_t adelta{512};
    inline static int16_t adelta2{512};
    
    inline static uint16_t deadP{100};
    inline static uint16_t deadN{deadP};

    inline static int16_t targetPos;
    
    inline static adc_t lastAnalog;
    
    
    inline static int16_t error(const int16_t actual) {
        constexpr uint8_t hysterese = 1;
        if (targetPos > (actual + hysterese)) {
            const int16_t diff = targetPos - actual;
            if (diff > adelta2) {
                return targetPos - (actual + adelta);
            }
            else {
                return targetPos - actual;
            }
        }
        else if (targetPos < (actual - hysterese)) {
            const int16_t diff = targetPos - actual;
            if (diff < -adelta2) {
                return targetPos - (actual - adelta);
            }
            else {
                return targetPos - actual;
            }
        }
        return 0;
    }
//    inline static int16_t aaa(const int16_t actual) {
//        const int16_t diff = targetPos - actual;
//        if (diff > adelta2) {
//            return actual + adelta;
//        }
//        else if (diff < -adelta2) {
//            return actual - adelta;
//        }
//        return actual;
//    }
    
    static inline void ratePeriodic() {
        const auto oldState = mState;
        const auto e = event();        
        const adc_t actual = ADC::value();
        ++stateTicks;
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            if (e == Event::Calibrate) {
                mState = State::CalibrateDeadStartP;
            }
            break;
        case State::CalibrateADStart:
            stateTicks.on(settleTicks, []{
                mState = State::CalibrateADRun;
            });
            break;
        case State::CalibrateADRun:
            stateTicks.on(adTestTicks, []{
                mState = State::CalibrateADTest;
            });
            if (actual > amax) {
                amax = actual;
            }
            if (actual < amin) {
                amin = actual;
            }
            break;
        case State::CalibrateADTest:
            if ((amax == amaxLast) && (amin == aminLast)) {
                mState = State::CalibrateADFinish;
            }
            else {
                mState = State::CalibrateADRun;
            }
            break;
        case State::CalibrateADFinish:
            stateTicks.on(adTestTicks, []{
                mState = State::NeutralStart;
            });
            break;
        case State::CalibrateDeadStartP:
            stateTicks.on(settleTicks, []{
                mState = State::CalibrateDeadSetP;
            });
            break;
        case State::CalibrateDeadSetP:
            stateTicks.on(settleTicks, []{
                mState = State::CalibrateDeadTestP;
            });
            break;
        case State::CalibrateDeadTestP:
            if (actual > (lastAnalog + 1)) {
                mState = State::CalibrateDeadStartN;
            }
            else {
                deadP += 1;
                mState = State::CalibrateDeadSetP;
            }
            break;            
        case State::CalibrateDeadStartN:
            stateTicks.on(settleTicks, []{
                mState = State::CalibrateDeadSetN;
            });
            break;
        case State::CalibrateDeadSetN:
            stateTicks.on(settleTicks, []{
                mState = State::CalibrateDeadTestN;
            });
            break;
        case State::CalibrateDeadTestN:
            if (actual < (lastAnalog - 1)) {
                mState = State::CalibrateADStart;
            }
            else {
                deadN += 1;
                mState = State::CalibrateDeadSetN;
            }
            break;            
        case State::NeutralStart:
            if (error(actual) > 0) {
                mState = State::NeutralReachForward;
            }
            else if (error(actual) < 0) {
                mState = State::NeutralReachBackward;
            }
            break;
        case State::NeutralReachForward:
            if (error(actual) <= 0) {
                mState = State::Neutral;
            }
            break;
        case State::NeutralReachBackward:
            if (error(actual) >= 0) {
                mState = State::Neutral;
            }
            break;
        case State::Neutral:
            if (e == Event::Run) {
                mState = State::Stop;
            }
            break;
        case State::Stop:
//        {
//            const int16_t cv = pid.correctionValue(aaa(actual), targetPos);
//            if (cv < 0) {
//                PPM::set(ppm_t{ppm_mid + deadP + 20 - cv});
//            }
//            else if (cv > 0) {
//                PPM::set(ppm_t{ppm_mid - deadN - 20 - cv});
//            }
//            else {
//                PPM::set(ppm_t{ppm_mid});
//            }
//        }
            if (error(actual) > 0) {
                mState = State::Forward;
            }
            else if (error(actual) < 0) {
                mState = State::Backward;
            }
            break;
        case State::Forward:
            if (error(actual) <= 0) {
                mState = State::Stop;
            }
            else {
                const auto cv = std::min(error(actual) * 2, ppm_delta / 4);
                PPM::set(ppm_t{ppm_mid + deadP + 30 + cv});
            }
            break;
        case State::Backward:
            if (error(actual) >= 0) {
                mState = State::Stop;
            }
            else {
                const auto cv = std::max(error(actual) * 2, -ppm_delta / 4);
                PPM::set(ppm_t{ppm_mid - deadN - 30 + cv});
            }
            break;
        }
        if (oldState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                PPM::set(ppm_t{ppm_mid});
                break;
            case State::CalibrateADStart:
                amax = amin = actual;
                PPM::set(ppm_t{ppm_mid + deadP + 50});
                break;
            case State::CalibrateADRun:
                amaxLast = amax;
                aminLast = amin;
                break;
            case State::CalibrateADTest:
                break;
            case State::CalibrateADFinish:
                adelta = amax - amin;
                adelta2 = adelta / 2;
                PPM::set(ppm_t{ppm_mid});
                targetPos = (amax + amin) / 2;
                break;
            case State::CalibrateDeadStartP:
                deadP = 50;
                PPM::set(ppm_t{ppm_mid});
                break;
            case State::CalibrateDeadSetP:
                lastAnalog = actual;
                PPM::set(ppm_t{ppm_mid + deadP});
                break;
            case State::CalibrateDeadTestP:
                break;
            case State::CalibrateDeadStartN:
                deadN = 50;
                PPM::set(ppm_t{ppm_mid});
                break;
            case State::CalibrateDeadSetN:
                lastAnalog = actual;
                PPM::set(ppm_t{ppm_mid - deadN});
                break;
            case State::CalibrateDeadTestN:
                break;
            case State::NeutralStart:
                targetPos = (amax + amin) / 2;
                break;
            case State::NeutralReachForward:
                PPM::set(ppm_t{ppm_mid + deadP + 50});
                break;
            case State::NeutralReachBackward:
                PPM::set(ppm_t{ppm_mid - deadN - 50});
                break;
            case State::Neutral:
                PPM::set(ppm_t{ppm_mid});
                mReady = true;
                break;
            case State::Stop:
                PPM::set(ppm_t{ppm_mid});
                break;
            case State::Forward:
                break;
            case State::Backward:
                break;
            }
        }
    }  
    inline static External::Tick<Timer> stateTicks;
    inline static State mState{State::Undefined};
};

template<typename PPM, uint8_t N>
struct PPMAdapter {
    using ranged_type = PPM::ranged_type;
    
    using index_t = PPM::index_type;

    inline static void set(const ranged_type& v) {
        PPM::ppmRaw(index_t{N}, v.toInt());
    }
};

template<typename ADC, uint8_t N>
struct ADCAdapter {
    using value_type = ADC::value_type; 
    using index_type = ADC::index_type; 
    inline static value_type value() {
        return ADC::value(index_type{N});
    }
};

template<typename PPM, typename ADC, typename PA, typename Timer>
struct GFSM {
    using index_t = PPM::index_type;
    using channel_t = PA::channel_t;
    using value_t = PA::value_type;
    using adcv_t = ADC::value_type; 
    
    using ppm1 = PPMAdapter<PPM, 0>;
    using adc1 = ADCAdapter<ADC, 0>;
    using ppm2 = PPMAdapter<PPM, 1>;
    using adc2 = ADCAdapter<ADC, 1>;
  
    using servo1 = Servo<ppm1, adc1, Timer>;
    using servo2 = Servo<ppm2, adc2, Timer>;
    
    enum class State : uint8_t {Undefined, Init, Calibrate, Run};

    static constexpr External::Tick<Timer> initTimeoutTicks{100_ms};
    
    static inline void init() {
        PPM::init();
        ADC::init();
        ADC::mcu_adc_type::nsamples(6);
    }
    static inline void periodic() {
        ADC::periodic();        
    }
    static inline void ratePeriodic() {
        servo1::ratePeriodic();
            
        const auto oldState = mState;
        ++stateTicks;        
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            stateTicks.on(initTimeoutTicks, []{
                mState = State::Calibrate;
            });
            break;
        case State::Calibrate:
            if (servo1::ready()) {
                mState = State::Run;
            }
            break;
        case State::Run:
            const auto pv = PA::value(4);
            servo1::position(pv);
            break;
        }
        if (oldState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                servo1::reset();
                break;
            case State::Calibrate:
                servo1::calibrate();
                break;
            case State::Run:
                servo1::run();
                break;
            }
        }
    }
    
    inline static External::Tick<Timer> stateTicks;
    static inline State mState{State::Undefined};
};

using adcController = External::Hal::AdcController<adc, Meta::NList<5, 3>>; // 1e = temp

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using ppmA = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>>>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

#ifndef DEBUG2
using terminal = etl::basic_ostream<void>;
#else
using terminal = etl::basic_ostream<servo>;
#endif

using gfsm = GFSM<ppmA, adcController, servo_pa, systemTimer>;

int main() {
    wdt::init<ccp>();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
     
    reset::noWatchDog([]{
        uninitialzed::reset();
    });
    
    reset::onWatchDog([]{
        uninitialzed::counter = uninitialzed::counter + 1;        
    });
    
    portmux::init();
    systemTimer::init();
    
# ifndef DEBUG2
    servo::init<AVR::BaudRate<115200>, HalfDuplex>();
    servo::txEnable<false>();
# else
    servo::init<AVR::BaudRate<115200>>();
# endif
    
    gfsm::init();
    
    const auto periodicTimer = alarmTimer::create(300_ms, External::Hal::AlarmFlags::Periodic);

    using ai_t = adcController::index_type;
    
    while(true) {
        servo::periodic();
        gfsm::periodic();
        
        systemTimer::periodic([&]{
            wdt::reset();
            gfsm::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::outl<terminal>("a: "_pgm, adcController::value(ai_t{0}).toInt(), " gs: "_pgm, (uint8_t)gfsm::mState, " ss: "_pgm, (uint8_t)gfsm::servo1::mState, " min: "_pgm, gfsm::servo1::amin, " max: "_pgm, gfsm::servo1::amax, " t: "_pgm, gfsm::servo1::targetPos);
                    etl::outl<terminal>("dn: "_pgm, gfsm::servo1::deadN, " dp: "_pgm, gfsm::servo1::deadP);
//                    etl::outl<terminal>("pid: "_pgm, gfsm::servo1::pid);
                }
            });
        });
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
#if !(defined(USE_IBUS) || defined(USE_HOTT))
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
    while(true) {
        servo::periodic();
//        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
