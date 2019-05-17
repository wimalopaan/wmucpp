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
#include <external/hott/hott.h>
#include <external/hott/menu.h>

#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/solutions/ifx007.h>
#include <external/solutions/rpm.h>
#include <external/solutions/ppm.h>

#include <external/hal/adccontroller.h>

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

using led = AVR::Pin<PortD, 7>;

using txPin = AVR::Pin<PortD, 5>;
using uart = AVR::SoftDevices::IntUart<AVR::TimerNumber<2>, AVR::BaudRate<9600>, txPin, AVR::UseRx<false>, AVR::UseInterrupts<false>>;
using terminalDevice = uart;
using terminal = etl::basic_ostream<terminalDevice>;

//using dbg1 = AVR::Pin<PortB, 3>; // MOSI
//using dbg2 = AVR::Pin<PortB, 4>; // MISO
//using dbg3 = AVR::Pin<PortB, 5>; // SCK

using pwm = AVR::PWM::ISRPwm<AVR::TimerNumber<0>, AVR::PWM::ChannelA<true>, AVR::PWM::ChannelB<false>>; // oca only

template<typename lowA, typename highA, typename lowB, typename highB, typename PWM>
struct Bridge final {
    
    // ocb flag für Beep -> Statemachine: Ap on, Bn on, Ap off, Bn off, all off, Bp on, An off, Bp off, An off, all off
    
    enum class Beep : uint8_t {Off, Slow, Fast, _Number};
    
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
    
    static inline void beep(Beep beep) {
        
    }
    
    static inline void turnOff() {
        pwm::off();
        OCAHandler::isr();
    }
    
    template<typename InputType>
    static inline void set(const InputType& d, bool back) {
        using value_type = typename InputType::value_type;
        constexpr value_type span = (InputType::Upper - InputType::Lower);
//        InputType::_;
//        std::integral_constant<uint16_t, pwm::template max<fPWM>()>::_;
        
        if (!d) return;
        
        value_type t = (etl::enclosing_t<value_type>(d.toInt()) * std::numeric_limits<pwm_type>::max()) / span;
        
        if (t < 20) {
            turnOff();
        }
        else {
            pwm::on();
        }
        {
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
        }
    }
    
private:
    static inline volatile bool backward = false;
};

using bridge = Bridge<pinAn, pinAp, pinBn, pinBp, pwm>;

using ppm = External::Ppm::IcpPpm<AVR::TimerNumber<1>, External::Ppm::RisingEdge<false>>; 

template<typename Actuator> 
struct Controller {
    
    static inline void init() {
        Actuator::init();
    }
    template<typename ValueType>
    static inline void set(const ValueType& v, bool b) {
//        ValueType::_;
        lastback = b;
        in = v.toInt();
        diff = std::min<int16_t>(mThresh, v.toInt() - lastValue);
        
        ValueType nv = lastValue + diff;
        
        Actuator::set(nv, b);
        lastValue = nv.toInt();
    }
    static inline void turnOff() {
        Actuator::turnOff();
        lastValue = 0;
    }
    static inline bool lastback;
    static inline uint16_t in;
    static inline uint16_t lastValue;
    static inline int16_t diff;
private:
    static inline uint16_t mThresh = 10;
};

using controller = Controller<bridge>;

template<typename Actuator>
struct EscStateFsm {
    enum class State : uint8_t {Undefined, Backward, Off, Forward, _Number};
    
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
            Actuator::turnOff();
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

struct GlobalFSM {
    enum class State : uint8_t {Undefined, Startup, ThrottleWarn, ThrottleSet, Run, _Number};
    enum class Beep  : uint8_t {Off, Low, High, _Number};
    
    static inline void init() {
    }

    static inline void fast() {
    }
    
    static inline void periodic() {
        auto p = ppm::pulse();
        State oldState = mState;
        switch (mState) {
        case State::Undefined:
            if (++mStateCounter > 10u) {
                mState = State::Startup;
            }
            break;
        case State::Startup:
            if (++mStateCounter > 100u) {
                if ((p > 50) || ( p < -50)) {
                    mState = State::ThrottleWarn;
                }
            }
            break;
        case State::ThrottleWarn:
            break;
        case State::ThrottleSet:
            break;
        case State::Run:
            fsm::update(p);
            break;
        default:
            break;
        }
        if (oldState != mState) {
            mStateCounter = 0;
            switch (mState) {
            case State::Undefined:
                break;
            case State::Startup:
                break;
            case State::ThrottleWarn:
                break;
            case State::ThrottleSet:
                break;
            case State::Run:
                break;
            default:
                break;
            }
                            
        }
    }
    
    
private:
    static inline State mState = State::Undefined;
    static inline etl::uint_ranged<uint16_t, 0, 1000> mStateCounter;
};

using gfsm = GlobalFSM;

using adc = AVR::Adc<0>;
using adcController = External::Hal::AdcController<adc, 0, 1, 2, 7, 6>;

//using alarmTimer = External::Hal::AlarmTimer<systemClock>;

struct Storage {
    enum class AVKey : uint8_t {Input = 0, Channel, _Number};
    
    struct ApplData : public EEProm::DataBase<ApplData> {
        etl::uint_NaN<uint8_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<etl::uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues{};
    };
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

using isrRegistrar = AVR::IsrRegistrar<bridge::OCAHandler, bridge::OVFHandler>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    //    dbg1::dir<AVR::Output>();
    //    dbg2::dir<AVR::Output>();
    //    dbg3::dir<AVR::Output>();
    
    gfsm::init();
    
    pwm::init<fPWM>();
    
    fsm:: init();
    
    eeprom::init();
    adcController::init();
    ppm::init();
    uart::init();
    
    led::dir<Output>();
    led::on();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        uint16_t c = 0;
        
        etl::outl<terminal>("test02"_pgm);
        
        while(true) {
            uart::periodic();
            ppm::periodic();
            gfsm::fast();
            ppm::overflow([&]{
                gfsm::periodic();
                led::toggle();
                auto p = ppm::pulse();
                fsm::update(p);
                if ((++c % 5) == 0){
                    etl::outl<terminal>("Ppm "_pgm, ppm::pulse().toInt(), " Cin "_pgm, controller::in, " Clb "_pgm, controller::lastback, " Cla "_pgm, controller::lastValue, " Cdi "_pgm, controller::diff);
//                    etl::outl<terminal>("Sst "_pgm, uint8_t(fsm::mState), " Soc "_pgm, fsm::mOffStateCounter.toInt());
//                    etl::outl<terminal>("Bb "_pgm, bridge::backward, " Bo "_pgm, bridge::off);
                }
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

ISR(TIMER0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::Overflow>();
}

