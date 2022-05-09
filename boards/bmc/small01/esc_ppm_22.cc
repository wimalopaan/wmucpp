#define NDEBUG

#ifndef GITMAJOR
# define VERSION_NUMBER 2300
#endif

//#define USE_TERMINAL_NO_LED
#define USE_SBUS_NO_PPM
#define USE_ACTIVE_FREEWHEEL

//#ifndef NDEBUG
//static unsigned int assertKey{1234};
//#endif

#include "devices.h"

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

template<typename Devices = void>
struct Data final : public EEProm::DataBase<Data<Devices>> {
    using d_t = Data<Devices>;
    
    using value_type = Devices::servo_pa::value_type;
//    value_type::_;
    
    static inline constexpr uint8_t magic = 42;
    
    void clear() {
        mMagic = magic;
        mThrMin = value_type::Lower;
        mThrMax = value_type::Upper;
        mThrMid = (value_type::Upper + value_type::Lower) / 2;
        mThrDead = (value_type::Upper - value_type::Lower) / 10;
        d_t::change();    
    }
    
    uint8_t mMagic;
    uint16_t mThrMin;
    uint16_t mThrMax;
    uint16_t mThrMid;
    uint16_t mThrDead;
    
};

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    using systemTimer = devs::systemTimer;

    using nvm = EEProm::Controller<Data<devs>>;
    using data_t = nvm::data_t;
    using pwm = devs::pwm;

//    static inline constexpr auto fPwm = 16000_Hz;
//    static inline constexpr auto fPwm = 400_Hz;
//    static inline constexpr auto pwmPeriod = pwm::f_timer / fPwm;
//    std::integral_constant<uint16_t, pwmPeriod>::_;

    using servo = devs::servo;
    using servo_pa = devs::servo_pa;
    
    using in_value_t = servo_pa::value_type; 
    static inline constexpr auto in_span = (in_value_t::Upper - in_value_t::Lower) / 2;
    static inline constexpr auto in_mid = (in_value_t::Upper + in_value_t::Lower) / 2;
    static inline constexpr auto pwmPeriod = in_span;
    static inline constexpr auto fPwm = Project::Config::fMcu / pwmPeriod;
    
//    std::integral_constant<uint32_t, span>::_;
//    std::integral_constant<uint32_t, fPwm.value>::_;
    
    static_assert((fPwm >= 16000_Hz) && (fPwm <= 25000_Hz));
    
#ifdef USE_TERMINAL_NO_LED
    using terminal = devs::terminal;
    using term_dev = terminal::device_type;
#else
    using blinkLed = devs::blinkLed;
    using count_type = blinkLed::count_type;
#endif
    
    using ef = devs::ef;
    
    using lut0 = devs::lut0;
    using lut1 = devs::lut1;
    
    static inline auto& data = nvm::data();

    enum class State : uint8_t {Undefined, Init, NoSignal, Off, Forward, Backward};
    
    static constexpr External::Tick<systemTimer> stateChangeTicks{500_ms};
    static constexpr External::Tick<systemTimer> resetTicks{1000_ms};
    static constexpr External::Tick<systemTimer> debugTicks{500_ms};

    using throttle_type = typename servo_pa::value_type;
//    throttle_type::_;
    using thr_t = typename throttle_type::value_type;
    
    static inline thr_t thrMax() {
        return nvm::data().mThrMax;
    }
    static inline thr_t thrMin() {
        return nvm::data().mThrMin;
    }
    static inline thr_t thrDead() {
        return nvm::data().mThrDead;
    }
    static inline thr_t thrMedium() {
        return nvm::data().mThrMid;
    }
    
    static inline thr_t thrStartForward() {
        return thrMedium() + thrDead();
    };
    static inline thr_t thrStartBackward() {
        return thrMedium() - thrDead();
    };
    
    inline static bool isThrottleForward() {
        return mThrottle && (mThrottle.toInt() >= thrStartForward());
    }
    inline static bool isThrottleBackward() {
        return mThrottle && (mThrottle.toInt() <= thrStartBackward());
    }
    inline static bool isThrottleOn() {
        return isThrottleBackward() || isThrottleForward();
    }
    
    static inline void init() {
        nvm::init();
        if (nvm::data().mMagic != nvm::data().magic) {
            nvm::data().clear();
            nvm::data().change();
        }
        lut0::init(0xff_B); // in1 = H 
        lut1::init(0xff_B); // in2 = H 
        lut0::enable();
        lut1::enable();
        
#ifdef USE_TERMINAL_NO_LED
# ifndef USE_SBUS_NO_PPM
        term_dev::template init<BaudRate<115200>>();
        term_dev::template rxEnable<false>();
# endif
#else
        blinkLed::init();
#endif
        ef::init();
        
        pwm::init();
        pwm::period(pwmPeriod);

#ifdef USE_SBUS_NO_PPM
        servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
        servo::rxInvert(true);
# ifndef USE_TERMINAL_NO_LED
        servo::template txEnable<false>();
        servo::txPinDisable();
        blinkLed::init();
# endif      
#else
        servo::init();
#endif
    }
    
    static inline void periodic() {
        nvm::saveIfNeeded([&]{
#ifdef USE_TERMINAL_NO_LED
            etl::outl<terminal>("save eep"_pgm);
#endif
        });
        servo::periodic();
#ifdef USE_TERMINAL_NO_LED
        term_dev::periodic();
#endif
    }

    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();

#ifdef USE_TERMINAL_NO_LED
#else
        blinkLed::ratePeriodic();
#endif

        mThrottle = servo_pa::value(0);
        
        const auto oldState = mState;
        ++mStateTick;
        (++mResetTick).on(resetTicks, []{
#ifdef USE_TERMINAL_NO_LED
            etl::outl<terminal>("t: "_pgm, mThrottle.toInt(), " pv: "_pgm, lastPV, " m: "_pgm, pwm::max());            
#endif
            if (servo_pa::packages() == 0) {
                mState = State::NoSignal;
            }
            servo_pa::resetStats();
            data.expire();
        });
        
        switch(mState) {
        case State::Undefined:
            mStateTick.on(stateChangeTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            if (servo_pa::packages() > 10) {
                mState = State::Off;
            }
            break;            
        case State::NoSignal:
            if (servo_pa::packages() > 10) {
                mState = State::Off;
            }
            break;            
        case State::Off:
            if (isThrottleForward()) {
                mState = State::Forward;
            }
            else if (isThrottleBackward()) {
                mState = State::Backward;
            }
            break;            
        case State::Forward:
            if (!isThrottleForward()) {
                mState = State::Off;
            }
            else {
                const auto tv = mThrottle.toInt() - in_mid;
                pwm::template duty<Meta::List<AVR::PWM::WO<0>>>(tv);
                lastPV = tv;
                     
                
//                const auto tv = mThrottle.toInt();
//                const auto d = etl::distance(tv, thrStartForward());
//                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrMax() - thrStartForward()}, etl::Intervall{0u, pwm::max()});
//                pwm::template duty<Meta::List<AVR::PWM::WO<0>>>(pv);
//                lastPV = pv;
            }   
            break;            
        case State::Backward:
            if (!isThrottleBackward()) {
                mState = State::Off;
            }
            else {
                const auto tv = mThrottle.toInt();
                const auto d = etl::distance(tv, thrStartBackward());
                const auto pv = etl::scale(d, etl::Intervall{thr_t{0}, thrStartBackward() - thrMin()}, etl::Intervall{0u, pwm::max()});
                pwm::template duty<Meta::List<AVR::PWM::WO<0>>>(pv);
                lastPV = pv;                
            }
            break;            
        }
        if (mState != oldState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
#ifdef USE_TERMINAL_NO_LED
                etl::outl<terminal>("S Undef"_pgm);
#else
                blinkLed::off();
#endif
                break;
            case State::Init:
#ifdef USE_TERMINAL_NO_LED
                etl::outl<terminal>("S Init"_pgm);
#else
                blinkLed::steady();
#endif
                break;            
            case State::NoSignal:
#ifdef USE_TERMINAL_NO_LED
                etl::outl<terminal>("S NoSig"_pgm);
#else
                blinkLed::blink(count_type{4});
#endif
                break;            
            case State::Off:
#ifdef USE_TERMINAL_NO_LED
                etl::outl<terminal>("S Off"_pgm);
#else
                blinkLed::blink(count_type{1});
#endif
                lut0::init(0xff_B); // in1 = H 
                lut1::init(0xff_B); // in2 = H 
                break;            
            case State::Forward:
#ifdef USE_TERMINAL_NO_LED
                etl::outl<terminal>("S FW"_pgm);
#else
                blinkLed::blink(count_type{2});
#endif
                lut0::init(0x01_B); // in1 = !WO0
#ifdef USE_ACTIVE_FREEWHEEL
                lut1::init(0x00_B); // in2 = 0
#else
                lut1::init(0x01_B); // in2 = !WO
#endif
                break;            
            case State::Backward:
#ifdef USE_TERMINAL_NO_LED
                etl::outl<terminal>("S BW"_pgm);
#else
                blinkLed::blink(count_type{3});
#endif
                lut0::init(0x01_B); // in1 = !WO0
#ifdef USE_ACTIVE_FREEWHEEL
                lut1::init(0x02_B); // in2 = WO0
#else
                lut1::init(0xff_B); // in2 = 1
#endif
                break;            
            }
        }
    }
private:
    static inline uint16_t lastPV;

    static inline throttle_type mThrottle;
    static inline State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mResetTick;
    static inline External::Tick<systemTimer> mDebugTick;
};
 
using devices = Devices<0>;
 
int main() {
    devices::init();
    
    using terminal = devices::terminal;
    using systemTimer = devices::systemTimer;
    using gfsm = GlobalFsm<devices>;
    
    gfsm::init();

    etl::outl<terminal>("start esc mini hw22"_pgm);
    
    while(true) {
        gfsm::periodic(); 
        systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
}

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
