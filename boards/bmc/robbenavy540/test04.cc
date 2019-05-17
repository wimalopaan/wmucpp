/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/cppm.h>
#include <mcu/internals/constantrate.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/timer.h>
#include <mcu/internals/softuart.h>

#include <etl/output.h>
#include <etl/fixedpoint.h>
#include <etl/scoped.h>
#include <etl/meta.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>

#include <external/hott/hott.h>
#include <external/hott/menu.h>

#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>

#include <external/solutions/ifx007.h>
#include <external/solutions/rpm.h>
#include <external/solutions/ppm.h>
#include <external/solutions/button.h>


namespace  {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace External::Units::literals;
    constexpr auto fPWM = 1000_Hz;
}

using PortB = AVR::Port<AVR::B>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;

using pinAn = AVR::ActiveHigh<AVR::Pin<PortD, 2>, AVR::Output>;
using pinBn = AVR::ActiveHigh<AVR::Pin<PortD, 0>, AVR::Output>;
using pinAp = AVR::ActiveLow<AVR::Pin<PortD, 1>, AVR::Output>;
using pinBp = AVR::ActiveLow<AVR::Pin<PortC, 5>, AVR::Output>;

using led = AVR::ActiveLow<AVR::Pin<PortD, 7>, AVR::Output>;

using buttonPin = AVR::ActiveLow<AVR::Pin<PortD, 6>, AVR::Input>;
using button = External::Button<buttonPin>;

using txPin = AVR::Pin<PortD, 5>;
using uart = AVR::SoftDevices::IntUart<AVR::TimerNumber<2>, AVR::BaudRate<9600>, txPin, AVR::UseRx<false>, AVR::UseInterrupts<false>>;
using terminalDevice = uart;
using terminal = etl::basic_ostream<terminalDevice>;

using pwm = AVR::PWM::ISRPwm<AVR::TimerNumber<0>, AVR::PWM::ChannelA<true>, AVR::PWM::ChannelB<true>>; 

template<typename lowA, typename highA, typename lowB, typename highB, typename PWM>
struct Bridge final {
    
    using pwm = PWM;
    using pwm_type = typename PWM::value_type;
    
    inline static constexpr pwm_type beepLevel = 10;
    
    inline static void init() {
        pinAn::init();
        pinAp::init();
        pinBn::init();
        pinBp::init();
    }    
    struct OCAHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<PWM::number>::CompareA> {
        inline static void isr() {
            pinAn::inactivate();
            pinBn::inactivate();
        }
    };
    struct OCBHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<PWM::number>::CompareB> {
        inline static void isr() {
            switch(beepPhase) {
            case 0:
                pinAn::inactivate();
                pinBp::activate();
                break;
            case 1:
                pinAn::inactivate();
                pinBp::inactivate();
                backward = true;
                break;
            case 2:
                pinBn::inactivate();
                pinAp::activate();
                break;
            case 3:
                pinBn::inactivate();
                pinAp::inactivate();
                backward = false;
                break;
            }
            ++beepPhase;
        }
    };
    struct OVFHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<PWM::number>::Overflow>  {
        inline static void isr() {
            if (backward) {
                pinBn::activate();
            }
            else {
                pinAn::activate();
            }
        }
    };
    
    static inline void beep(bool on) {
        etl::Scoped<etl::DisbaleInterrupt<>> di;
        if (on) {
            pwm::enable(AVR::PWM::ChannelA<false>());
            backward = false;
            pwm::enable(AVR::PWM::ChannelB<true>());
            pwm::b(10);
        }
        else {
            pwm::off();
        }
    }
    
    static inline void off() {
        etl::Scoped<etl::DisbaleInterrupt<>> di;
        pwm::off();
        OCAHandler::isr();
    }
    static inline void on() {
        etl::Scoped<etl::DisbaleInterrupt<>> di;
        pwm::enable(AVR::PWM::ChannelA<true>());
        pwm::enable(AVR::PWM::Overflow<true>());
    }
    
    template<typename InputType>
    static inline void set(const InputType& d, bool back) {
        using value_type = typename InputType::value_type;
        constexpr value_type span = (InputType::Upper - InputType::Lower);
        //        InputType::_;
        //        std::integral_constant<uint16_t, pwm::template max<fPWM>()>::_;
        
        if (!d) return;
        
        value_type t = (etl::enclosing_t<value_type>(d.toInt()) * std::numeric_limits<pwm_type>::max()) / span;
        
        value_type lastt;
        
        if (t == lastt) return;
        
        lastt = t;
        
        if (t < 20) {
            off();
        }
        else {
            on();
        }
        if (t < (std::numeric_limits<pwm_type>::max() - 20)){
            etl::Scoped<etl::DisbaleInterrupt<>> di;
            backward = back;        
            pwm::a(t);
            
            if (backward) {
                pinBp::inactivate();
                pinAn::inactivate();
                pinAp::activate();
            }
            else {
                pinAp::inactivate();
                pinBn::inactivate();
                pinBp::activate();
            }
            mFull = false;
        }
        else {
            mFull = true;
            etl::Scoped<etl::DisbaleInterrupt<>> di;
            backward = back;        
            off();
            if (backward) {
                pinBp::inactivate();
                pinAn::inactivate();
                pinBn::activate();
                pinAp::activate();
            }
            else {
                pinBn::inactivate();
                pinAp::inactivate();
                pinBp::activate();
                pinAn::activate();
            }
        }
    }
    
    static inline volatile bool mFull = false;
private:
    static inline volatile bool backward = false;
    static inline volatile etl::uint_ranged_circular<uint8_t, 0, 3> beepPhase;
};

using bridge = Bridge<pinAn, pinAp, pinBn, pinBp, pwm>;

template<typename BR, auto PulseCount = 100u>
struct MotorBeeper {
    using dev = BR;
    static inline void periodic() {
        if (mOn) {
            ++mCount;
            if (mCount == mPulseCount) {
                dev::beep(false);
            }
            if (mCount == (2 * mPulseCount)) {
                dev::beep(true);
                mCount = 0;
            }
        }
    }
    static inline void beep(bool on) {
        mOn = on;
        mCount = 0;
        dev::beep(true);
    }
    static inline void pulse(uint16_t c) {
        mPulseCount = c;
    }
private:
    static inline bool mOn = false;
    static inline uint16_t mCount = 0;
    static inline uint16_t mPulseCount = PulseCount;
};

using beeper = MotorBeeper<bridge>;

using ppm = External::Ppm::IcpPpm<AVR::TimerNumber<1>, External::Ppm::RisingEdge<false>>; 

template<typename Actuator, uint8_t MaxIncrement = 10> 
struct Controller {
    static inline void init() {
        Actuator::init();
    }
    template<typename ValueType>
    static inline void set(const ValueType& v, bool b) {
        using signed_type = std::make_signed_t<typename ValueType::value_type>;
        signed_type diff = std::min<signed_type>(mThresh, v.toInt() - lastValue);
        
        ValueType nv = lastValue + diff;
        Actuator::set(nv, b);
        lastValue = nv.toInt();
    }
    static inline void off() {
        Actuator::off();
        lastValue = 0;
    }
private:
    static inline uint16_t lastValue;
    static inline const uint16_t mThresh = MaxIncrement;
};

using controller = Controller<bridge>;

template<typename Actuator>
struct EscStateFsm {
    enum class State : uint8_t {Undefined = 0, Backward, Off, Forward, _Number};
    
    static inline void init() {
        Actuator::init();
    }    
    template<typename InputType>
    static inline void update(const InputType& v) {
        using output_t = etl::uint_ranged_NaN<std::make_unsigned_t<typename InputType::value_type>, 0, InputType::Upper>;
        
        if (!v) return;
        
        switch(mState) {
        case State::Undefined:
            mState = State::Off;
            break;
        case State::Backward:
            if (v.toInt() >= 0) {
                mState = State::Off;
                mOffStateCounter = 0;
            }
            else {
                Actuator::set(output_t(-v.toInt()), true);
            }
            break;
        case State::Off:
            Actuator::off();
            if (++mOffStateCounter > 5) {
                if (v.toInt() > 0) {
                    mState = State::Forward;
                }
                if (v.toInt() < 0) {
                    mState = State::Backward;
                }
            }
            break;
        case State::Forward:
            if (v.toInt() <= 0) {
                mState = State::Off;
                mOffStateCounter = 0;
            }
            else {
                Actuator::set(output_t(v.toInt()), false);
            }
            break;
        default:
            break;
        }
    }
private:
    static inline State mState = State::Undefined;
    static inline etl::uint_ranged_NaN<uint8_t, 0, 100> mOffStateCounter = 0;
};

using fsm = EscStateFsm<controller>;

template<typename TUnit>
struct GlobalFSM {
    enum class State : uint8_t {Undefined = 0, Startup, ThrottleWarn, ThrottleSet, Run, _Number};
    enum class Beep  : uint8_t {Off, Low, High, _Number};
    
    static inline constexpr auto intervall = TUnit::exact_intervall;
    
    using ppm_type = decltype(ppm::pulse());
    
    static inline void init() {
    }
    
    template<typename T>
    static inline ppm_type scale(const T& v) {
        using tt = etl::enclosing_t<ppm_type::value_type>;
        tt scaled = ((tt)v.toInt() * ppm_type::Upper) / mMaxPpm;
        scaled = std::clamp(scaled, (tt)ppm_type::Lower, (tt)ppm_type::Upper);
        return scaled;
    }
    
    static inline void periodic() {
        auto p = ppm::pulse();
//        decltype(p)::_;
        State oldState = mState;
        switch (mState) {
        case State::Undefined:
            if (mButtonPressed) {
                mState = State::ThrottleSet;
                mButtonPressed = false;
            }
            if (++mStateCounter > 50u) {
                mState = State::Startup;
            }
            break;
        case State::Startup:
            if (++mStateCounter > 50u) {
                if ((p > 50) || ( p < -50)) {
                    mState = State::ThrottleWarn;
                }
                else {
                    mState = State::Run;
                }
            }
            break;
        case State::ThrottleWarn:
            if ((p < 10) && ( p > -10)) {
                mState = State::Run;
            }
            break;
        case State::ThrottleSet:
            if (p > mMaxPpm) {
                mMaxPpm = p.toInt();
            }
            if (p < mMinPpm) {
                mMinPpm = p.toInt();
            }
            if (mButtonPressed) {
                mButtonPressed = false;
                mState = State::Run;
            }
            break;
        case State::Run:
            if (mButtonPressed) {
                mButtonPressed = false;
                mState = State::ThrottleSet;
            }
            mScaled = scale(p);
            fsm::update(mScaled);
            break;
        default:
            break;
        }
        if (oldState != mState) {
            mStateCounter = 0;
            switch (mState) {
            case State::Undefined:
                led::inactivate();
                break;
            case State::Startup:
                led::activate();
                break;
            case State::ThrottleWarn:
                beeper::beep(true);
                break;
            case State::ThrottleSet:
                mMaxPpm = 300;
                mMinPpm = -300;
                break;
            case State::Run:
                beeper::beep(false);
                break;
            default:
                break;
            }
            
        }
    }
    static inline void buttonPressed() {
        mButtonPressed = true;
    }
    
    
    static inline State mState = State::Undefined;
    static inline ppm_type mScaled;;
    static inline ppm_type::value_type mMaxPpm = ppm_type::Upper;
    static inline ppm_type::value_type mMinPpm = ppm_type::Lower;
private:
    static inline bool mButtonPressed = false;
    static inline etl::uint_ranged<uint16_t, 0, 1000> mStateCounter;
};

using gfsm = GlobalFSM<ppm>;

using adc = AVR::Adc<0>;
using adcController = External::Hal::AdcController<adc, 0, 1, 2, 7, 6>;

using alarmTimer = External::Hal::AlarmTimer<ppm>;

struct Storage {
    enum class AVKey : uint8_t {MaxPpm= 0, MinPpm, _Number};
    
    struct ApplData : public EEProm::DataBase<ApplData> {
        etl::uint_NaN<uint16_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<etl::uint_NaN<uint16_t>, static_cast<uint8_t>(AVKey::_Number)> AValues{};
    };
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

using isrRegistrar = AVR::IsrRegistrar<bridge::OCAHandler, bridge::OCBHandler, bridge::OVFHandler>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    button::init();
    gfsm::init();
    
    pwm::init<fPWM>();
    
    fsm:: init();
    
    eeprom::init();
    adcController::init();
    ppm::init();
    uart::init();
    
    led::init();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        uint16_t c = 0;
        
        etl::outl<terminal>("test04"_pgm);
        
        const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
        
        while(true) {
            uart::periodic();
            ppm::periodic();
            ppm::overflow([&]{
                button::periodic([]{
                    //                    led::toggle();
                    gfsm::buttonPressed();
                });            
                beeper::periodic();
                gfsm::periodic();
                alarmTimer::periodic([&](auto timer){
                    if (t == timer) {
                        etl::outl<terminal>("c: "_pgm, ++c, "gs "_pgm, (uint8_t)gfsm::mState, " ma "_pgm, gfsm::mMaxPpm, " mi "_pgm, gfsm::mMinPpm, " sc "_pgm, gfsm::mScaled.toInt(), " bf "_pgm, bridge::mFull);                        
                    }
                });
            });
            if(eeprom::saveIfNeeded()) {
                etl::out<terminal>("."_pgm);
            }    
        }
    }
}

// 0 pwm
ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(TIMER0_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareB>();
}
ISR(TIMER0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::Overflow>();
}

