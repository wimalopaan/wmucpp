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
#include <external/solutions/blinker.h>

#include "bridge.h"

namespace  {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace External::Units::literals;
    constexpr auto fPWM = 4000_Hz;
    constexpr auto fPWM2 = 500_Hz;
    
    constexpr auto ledPulse = 200_ms;
    constexpr auto ledPeriod = 2500_ms;
    
    constexpr float cell_normal = 2.0f;
    constexpr float cell_min = 1.7f;
    
    constexpr float voltage_divider = 0.1488f;
}

struct Storage final {
    enum class AVKey : uint8_t {MaxPpm = 0, MinPpm, Init1, Init2, _Number};
    
    struct ApplData : public EEProm::DataBase<ApplData> {
        uint16_t& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<uint16_t, static_cast<uint8_t>(AVKey::_Number)> AValues{};
    };
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

using PortB = AVR::Port<AVR::B>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;

using pinAn = AVR::ActiveHigh<AVR::Pin<PortD, 2>, AVR::Output>;
using pinBn = AVR::ActiveHigh<AVR::Pin<PortD, 0>, AVR::Output>;
using pinAp = AVR::ActiveLow<AVR::Pin<PortD, 1>, AVR::Output>;
using pinBp = AVR::ActiveLow<AVR::Pin<PortC, 5>, AVR::Output>;

using ledPin = AVR::ActiveLow<AVR::Pin<PortD, 7>, AVR::Output>;

using buttonPin = AVR::ActiveLow<AVR::Pin<PortD, 6>, AVR::Input>;
using button = External::Button<buttonPin>;

using txPin = AVR::Pin<PortD, 5>;
using uart = AVR::SoftDevices::IntUart<AVR::Component::Timer<2>, AVR::BaudRate<9600>, txPin, AVR::UseRx<false>, AVR::UseInterrupts<false>>;
using terminalDevice = uart;
using terminal = etl::basic_ostream<terminalDevice>;

using pwm = AVR::PWM::ISRPwm<AVR::Component::Timer<0>, AVR::PWM::ChannelA<true>, AVR::PWM::ChannelB<true>>; 

using bridge = External::Solutions::Bridge<pinAn, pinAp, pinBn, pinBp, pwm, fPWM, fPWM2>;

using beeper = External::Solutions::MotorBeeper<bridge>;

using ppm = External::Ppm::IcpPpm<AVR::Component::Timer<1>, External::Ppm::RisingEdge<false>>; 

using led = External::Blinker<ledPin, ppm::exact_intervall, ledPulse, ledPeriod>;

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

template<typename TUnit, typename AD>
struct GlobalFSM {
    enum class State : uint8_t {Undefined = 0, Startup, CellEstimate, CellBeep, ThrottleWarn, ThrottleSetMax, ThrottleSetMin, ThrottleSetNeutral, ThrottleSetCheck, 
                                Error, Armed, Run, RunReduced, _Number};
    enum class Beep  : uint8_t {Off, Low, High, _Number};
    
    static inline constexpr auto intervall = TUnit::exact_intervall;
//    std::integral_constant<uint16_t, intervall.value>::_;
    
    static inline constexpr auto minStateTime = 500_ms;
    static inline constexpr auto maxStateTime = ledPeriod;
//    using state_counter_type = etl::typeForValue_t<minStateTime / intervall>;
    using state_counter_type = etl::typeForValue_t<(maxStateTime / intervall) + 1>;
    static inline constexpr state_counter_type minStateCounter = minStateTime / intervall;
    static inline constexpr state_counter_type maxStateCounter = maxStateTime / intervall;
//        std::integral_constant<uint16_t, minStateCounter>::_;
//            std::integral_constant<uint16_t, maxStateCounter>::_;
//    state_counter_type::_;
    
    using ppm_type = decltype(ppm::pulse());
    
    static inline constexpr auto ppmHysterese = 2 * (ppm_type::Upper - ppm_type::Lower) / 100;
    
    static inline constexpr typename AD::value_type cell_normal_raw = (cell_normal * voltage_divider * 1024) / AD::VRef;
//    std::integral_constant<uint16_t, cell_normal_raw.toInt()>::_;
    static inline constexpr typename AD::value_type cell_min_raw = (cell_min * voltage_divider * 1024) / AD::VRef;
//    std::integral_constant<uint16_t, cell_min_raw.toInt()>::_;
    
    static inline void init() {
        mMaxPpm = appData[Storage::AVKey::MaxPpm];
        mMinPpm = appData[Storage::AVKey::MinPpm];
    }

    static inline void saveToEEprom() {
        appData[Storage::AVKey::MaxPpm] = mMaxPpm;
        appData[Storage::AVKey::MinPpm] = mMinPpm;
        appData.change();
    }
    
    template<typename V>
    static inline bool isNearNeutral(const V& v) {
        return ((v >= -ppmHysterese) && (v <= ppmHysterese));
    }
    
    template<typename T>
    static inline ppm_type scale(const T& v) {
        using tt = etl::enclosing_t<ppm_type::value_type>;
        tt scaled = ((tt)v.toInt() * ppm_type::Upper) / mMaxPpm;
        scaled = std::clamp(scaled, (tt)ppm_type::Lower, (tt)ppm_type::Upper);
        return scaled;
    }
    
    static inline void periodic() { // called every 'intervall' ms
        auto p = ppm::pulse();
//        decltype(p)::_;
        State oldState = mState;
        switch (mState) {
        case State::Undefined:
            if (mButtonPressed) {
                mState = State::ThrottleSetMax;
                mButtonPressed = false;
            }
            if (p && (++mStateCounter > minStateCounter)) {
                mState = State::Startup;
            }
            break;
        case State::Startup:
            if (p && (++mStateCounter > minStateCounter)) {
                if (!isNearNeutral(p)) {
                    mState = State::ThrottleWarn;
                }
                else {
                    mState = State::CellEstimate;
                }
            }
            break;
        case State::CellEstimate:
        {
            auto v = AD::value(1);
            mCellCount = v.toInt() / cell_normal_raw;
            mVThresh = mCellCount * cell_min_raw.toInt();
            mState = State::CellBeep;
        }
            break;
        case State::CellBeep:
            if (++mStateCounter > maxStateCounter) {
                mState = State::Armed;
            }
            break;
        case State::ThrottleWarn:
            if (p && isNearNeutral(p)) {
                mState = State::Run;
            }
            break;
        case State::ThrottleSetMax:
            if (p) {
                mMaxPpm = p.toInt();
                if (mButtonPressed) {
                    mButtonPressed = false;
                    mState = State::ThrottleSetNeutral;
                }
            }
            break;
        case State::ThrottleSetNeutral:
            if (mButtonPressed) {
                mButtonPressed = false;
                mState = State::ThrottleSetMin;
            }
            break;
        case State::ThrottleSetMin:
            if (p) {
                mMinPpm = p.toInt();
                if (mButtonPressed) {
                    mButtonPressed = false;
                    mState = State::ThrottleSetCheck;
                }
            }
            break;
        case State::ThrottleSetCheck:
            if ((mMaxPpm > 300) && (mMinPpm < -300)) {
                saveToEEprom();
                if (++mStateCounter > minStateCounter) {
                    mState = State::Startup;
                }
            }
            else {
                mState = State::Error;
            }
            break;
        case State::Error:
            break;
        case State::Armed:
            if (++mStateCounter > minStateCounter) {
                mState = State::Run;
            }
            break;
        case State::Run:
        {
            if (mButtonPressed) {
                mButtonPressed = false;
                mState = State::ThrottleSetMax;
            }
            auto v = AD::value(1);
            if (v < mVThresh) {
                mState = State::RunReduced;
            }               
            if (p) {
                mScaled = scale(p);
                fsm::update(mScaled);
            }
        }
            break;
        case State::RunReduced:
        {
            if (mButtonPressed) {
                mButtonPressed = false;
                mState = State::Run;
            }
            if (p) {
                mScaled = scale(p);
                mScaled /= 2;
                fsm::update(mScaled);
            }
        }
            break;
        default:
            break;
        }
        if (oldState != mState) {
            mStateCounter = 0;
            switch (mState) {
            case State::Undefined:
                led::off();
                break;
            case State::Startup:
                break;
            case State::CellEstimate:
                break;
            case State::CellBeep:
                led::blink(mCellCount);
                break;
            case State::ThrottleWarn:
                beeper::pulse(2);
                beeper::beep(true);
                break;
            case State::ThrottleSetMax:
                beeper::pulse(10);
                beeper::beep(true);
                mMaxPpm = 300;
                mMinPpm = -300;
                led::blink(1);
                break;
            case State::ThrottleSetNeutral:
                led::blink(2);
                break;
            case State::ThrottleSetMin:
                led::blink(3);
                break;
            case State::ThrottleSetCheck:
                break;
            case State::Error:
                led::off();
                beeper::pulse(1);
                beeper::beep(true);
                break;
            case State::Armed:
                led::off();
                beeper::pulse(5);
                beeper::beep(true);
                break;
            case State::Run:
                led::steady();
                beeper::beep(false);
                break;
            case State::RunReduced:
                led::blink(4);
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
    static inline ppm_type mScaled;
    static inline ppm_type::value_type mMaxPpm = ppm_type::Upper;
    static inline ppm_type::value_type mMinPpm = ppm_type::Lower;
    static inline AD::value_type mVThresh = 0;
    static inline uint8_t mCellCount = 0;
private:
    static inline bool mButtonPressed = false;
    static inline etl::uint_ranged<state_counter_type, 0, maxStateCounter + 1> mStateCounter;
};

using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<10>, AVR::AD::VRef<AVR::AD::Vextern<2,523>>>;
using adcController = External::Hal::AdcController<adc, 6, 7, 8>;

using gfsm = GlobalFSM<ppm, adcController>;


using alarmTimer = External::Hal::AlarmTimer<ppm>;

using isrRegistrar = AVR::IsrRegistrar<bridge::OCAHandler, bridge::OCBHandler, bridge::OVFHandler>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    eeprom::init();
    if ((appData[Storage::AVKey::Init1] != 42) || (appData[Storage::AVKey::Init2] != 43)) {
        appData[Storage::AVKey::Init1] = 42;
        appData[Storage::AVKey::Init2] = 43;
        appData[Storage::AVKey::MaxPpm] = 500;
        appData[Storage::AVKey::MinPpm] = -500;
        appData.change();
    }
    
    button::init();
    gfsm::init();
    
    pwm::init<fPWM>();
    
    fsm:: init();
    
    adcController::init();
    ppm::init();
    uart::init();
    
    led::init();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        uint16_t c = 0;
        
        etl::outl<terminal>("test05"_pgm);
        
        const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

        while(true) {
            adcController::periodic();
            uart::periodic();
            ppm::periodic();
            ppm::overflow([&]{ // called every ppm::exact_interval ms;
                button::periodic([]{
                    gfsm::buttonPressed();
                });            
                beeper::periodic();
                gfsm::periodic();
                led::periodic();
                alarmTimer::periodic([&](auto timer){
                    if (t == timer) {
                        etl::outl<terminal>("c: "_pgm, ++c, " gs "_pgm, (uint8_t)gfsm::mState, " ma "_pgm, gfsm::mMaxPpm, " mi "_pgm, gfsm::mMinPpm);                        
                        etl::outl<terminal>("c: "_pgm, ++c, " nc "_pgm, gfsm::mCellCount, " th "_pgm, gfsm::mVThresh.toInt());                        
//                        etl::outl<terminal>("c: "_pgm, ++c, " a6 "_pgm, adcController::value(0).toInt(), " a7 "_pgm, adcController::value(1).toInt(), " te "_pgm, adcController::value(2).toInt());                        
//                        etl::outl<terminal>("c: "_pgm, ++c, " a6 "_pgm, adcController::value(0).toInt(), " a7 "_pgm, adcController::value(1).toInt());                        
                    }
                    appData.expire();
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

