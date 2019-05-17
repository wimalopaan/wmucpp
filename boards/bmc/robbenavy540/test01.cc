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

#include <external/hal/adccontroller.h>

namespace  {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace External::Units::literals;
//    constexpr auto interval = 16_ms;
    constexpr auto interval = 16000_us;
    constexpr auto fPWM = 4000_Hz;
}

template<typename HighSide, typename LowSide>
struct HalfBridge {
    inline static constexpr void init() {
        HighSide::init();
        LowSide::init();
    }
    inline static constexpr void high() {
        LowSide::inactivate();
        HighSide::activate();
    }
    inline static constexpr void off() {
        LowSide::inactivate();
        HighSide::inactivate();
    }
    inline static constexpr void low() {
        HighSide::inactivate();
        LowSide::activate();
    }
};

using PortB = AVR::Port<AVR::B>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;

using pinAn = AVR::ActiveHigh<AVR::Pin<PortD, 2>, AVR::Output>;
using pinBn = AVR::ActiveHigh<AVR::Pin<PortD, 0>, AVR::Output>;
using pinAp = AVR::ActiveLow<AVR::Pin<PortD, 1>, AVR::Output>;
using pinBp = AVR::ActiveLow<AVR::Pin<PortC, 5>, AVR::Output>;

using led = AVR::Pin<PortD, 7>;

using txPin = AVR::Pin<PortD, 5>;
using uart = AVR::SoftDevices::IntUart<AVR::TimerNumber<2>, AVR::BaudRate<9600>, txPin>;
using terminalDevice = uart;
using terminal = etl::basic_ostream<terminalDevice>;

//using dbg1 = AVR::Pin<PortB, 3>; // MOSI
//using dbg2 = AVR::Pin<PortB, 4>; // MISO
//using dbg3 = AVR::Pin<PortB, 5>; // SCK

using hba = HalfBridge<pinAp, pinAn>;
using hbb = HalfBridge<pinBp, pinBn>;

using pwm = AVR::PWM::ISRPwm<AVR::TimerNumber<0>>; // oca only

template<typename HBA, typename HBB, typename PWM, typename InputType>
struct Bridge {
    using pwm = PWM;
    using value_type = typename PWM::value_type;
    using input_type = InputType;
    
    inline static constexpr input_type::value_type medium = (input_type::Upper + input_type::Lower) / 2;
    inline static constexpr input_type::value_type span = (input_type::Upper - input_type::Lower) / 2;
    
    inline static void init() {
        HBA::init();
        HBB::init();
        HBA::off();
        HBB::off();
    }    
    struct OCAHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<PWM::number>::CompareA> {
        inline static void isr() {
            if (backward) {
                HBA::off();
            }
            else {
                HBB::off();
            }
        }
    };
    struct OVFHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<PWM::number>::Overflow>  {
        inline static void isr() {
            if (!off) {
                if (backward) {
                    HBA::low();
                }
                else {
                    HBB::low();
                }
            }
        }
    };
    static inline void a(const input_type& d) {
        if (!d) return;
        
        typename input_type::value_type v;
        
        
        etl::Scoped<etl::DisbaleInterrupt<>> ei;
        
        if (d.toInt() >= medium) {
            v = d.toInt() - medium;
            HBA::high();
            backward = false;
        }
        else {
            v = medium - d.toInt();
            HBB::high();
            backward = true;
        }
        
        value_type t = (etl::enclosing_t<typename input_type::value_type>(v) * pwm::template max<fPWM>()) / span;

        if (t < 10) {
            off = true;
        }
        else {
            off = false;
        }
        
        last = t;
        lastv = v;
        
        pwm::a(t);
    }
//private:
    static inline volatile bool backward = false;
    static inline volatile bool off = false;
    static inline input_type::value_type last;
    static inline input_type::value_type lastv;
};

template<typename lowA, typename highA, typename lowB, typename highB, typename PWM, typename InputType>
struct Bridge2 {
    using pwm = PWM;
    using value_type = typename PWM::value_type;
    using input_type = InputType;
    
    inline static constexpr input_type::value_type medium = (input_type::Upper + input_type::Lower) / 2;
    inline static constexpr input_type::value_type span = (input_type::Upper - input_type::Lower) / 2;
    
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
            if (!off) {
                if (backward) {
                    pinAn::inactivate();
                    pinBp::inactivate();
                    pinAp::activate();
                    
                    pinBn::activate();
                }
                else {
                    pinBn::inactivate();
                    pinAp::inactivate();
                    pinBp::activate();

                    pinAn::activate();
                }
            }
        }
    };
    static inline void a(const input_type& d) {
        if (!d) return;
        
        typename input_type::value_type v;
        
        etl::Scoped<etl::DisbaleInterrupt<>> di;
        
        if (d.toInt() >= medium) {
            v = d.toInt() - medium;
            backward = false;
        }
        else {
            v = medium - d.toInt();
            backward = true;
        }
        
        value_type t = (etl::enclosing_t<typename input_type::value_type>(v) * pwm::template max<fPWM>()) / span;

        if (t < 10) {
            pinAn::inactivate();
            pinBn::inactivate();
            pinAp::inactivate();
            pinBp::inactivate();
            off = true;
        }
        else {
            off = false;
        }
        
        pwm::a(t);
    }
//private:
    static inline volatile bool backward = false;
    static inline volatile bool off = false;
    static inline volatile uint8_t off_count = 0;
};


using bridge = Bridge2<pinAn, pinAp, pinBn, pinBp, pwm, etl::uint_ranged_NaN<uint16_t, 1000, 2000>>;
//using bridge = Bridge<hba, hbb, pwm, etl::uint_ranged_NaN<uint16_t, 1000, 2000>>;

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

using namespace std::literals::chrono;
using namespace External::Units;
using namespace AVR::Util::Timer;

template<etl::Concepts::NamedConstant TimerNumber, auto Channels = 8, typename MCU = DefaultMcuType>
struct IcpPpm {
    using value_type = uint16_t;
    using index_type = etl::uint_ranged<uint8_t, 0, Channels - 1>;
    
    static constexpr auto mcu_timer = TimerParameter<TimerNumber::value, MCU>::mcu_timer;
    static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber::value, MCU>::mcu_timer_interrupts;
    
    using flags_type = typename TimerParameter<TimerNumber::value, MCU>::mcu_timer_interrupts_flags_type;

    using tb = typename TimerParameter<TimerNumber::value, MCU>::tb;
    
    static inline constexpr uint16_t prescaler = AVR::Ppm::Util::calculateCPpmInParameter<TimerNumber::value, DefaultMcuType, value_type>(10000);
//    using x1 = std::integral_constant<uint16_t, prescaler>::_;;

    //    static inline constexpr uint16_t prescaler = AVR::Ppm::Util::calculateCPpmInParameter<Timer::number>();

    static inline constexpr hertz f = Project::Config::fMcu / prescaler;
    static inline constexpr value_type ppm_pause = 19_ms * f;
    static inline constexpr value_type cppm_sync = 3_ms * f;
    static inline constexpr value_type ppm_min  = 1_ms * f;
    static inline constexpr value_type ppm_max  = 2_ms * f;
    static inline constexpr value_type ppm_width = ppm_max - ppm_min;
    
//    std::integral_constant<uint16_t, intervall>::_;
//    std::integral_constant<uint16_t, sync>::_;
//    std::integral_constant<uint16_t, prescaler>::_;
//    std::integral_constant<uint16_t, ppm_min>::_;
//    std::integral_constant<uint16_t, ppm_max>::_;
//    std::integral_constant<uint16_t, ppm_pause>::_;
    
    enum class State : uint8_t {Undefined, Raised, Fallen, _Number};
    
    static inline void periodic() {
        if (mcu_timer_interrupts()->tifr.template isSet<flags_type::icf>()) {
            
            uint16_t diff = (*mcu_timer()->icr >= savedValue) ? (*mcu_timer()->icr - savedValue) : (std::numeric_limits<value_type>::max() - savedValue + *mcu_timer()->icr);
            savedValue = *mcu_timer()->icr;

            if (mcu_timer()->tccrb.template isSet<tb::ices>()) {
                pause = diff;
                mcu_timer()->tccrb.template clear<tb::ices>();
            }
            else {
                pulse = diff;
                mcu_timer()->tccrb.template add<tb::ices>();
            }
            mcu_timer_interrupts()->tifr.template reset<flags_type::icf>(); // reset
        } 
    }

    template<etl::Concepts::Callable Callable>
    inline static void overfloaw(const Callable& f) {
        if (mcu_timer_interrupts()->tifr.template isSet<flags_type::tov>()) {
            f();
            mcu_timer_interrupts()->tifr.template reset<flags_type::tov>(); // reset
        } 
    }
    
    static inline void init() {
        constexpr auto bits = bitsFrom<prescaler>(prescaler_bits_v<TimerNumber::value>);            
        static_assert(isset(bits), "wrong prescaler");
        mcu_timer()->tccrb.template set<bits | tb::icnc | tb::ices>();
    }

//private:
    inline static State state = State::Undefined;
    inline static uint8_t syncCounter = 0;
    inline static uint16_t savedValue = 0;
    inline static uint16_t pulse = 0;
    inline static uint16_t pause = 0;
};

using ppm = IcpPpm<AVR::TimerNumber<1>>; 

struct FSM {
    enum class State : uint8_t {Undefined, Error, Sumd, Robo, _Number};

    enum class Event : uint8_t {SumdConnected, RoboConnected, LostConnection, _Number};
    
    inline static void process(Event e) {
        auto state = mState;
        switch(mState) {
        case State::Undefined:
        case State::Error:
        case State::Sumd:
        case State::Robo:
            if (e == Event::SumdConnected) {
                state = State::Sumd;
            }
            else if (e == Event::RoboConnected) {
                state = State::Robo;
            }
            break;
        default:
            break;
        }
        if (state != mState) {
            mState = state;
            if (mState == State::Sumd) {
            }
            else if (mState == State::Robo) {
            }
        }
    }
    
    inline static void periodic() {
        static etl::uint_ranged_circular<uint8_t, 0, 1> updateCount;
        switch(mState) {
        case State::Sumd:
            update_sumd(updateCount);
            break;
        case State::Robo:
            update_robo(updateCount);
            break;
        default:
            break;
        }
        ++updateCount;
    }
    template<typename T>
    inline static void update_sumd(T c) {
        switch(c.toInt()) {
        case 0:
            break;
        case 1:
            break;
        }
    }
    template<typename T>
    inline static void update_robo(T c) {
        switch(c.toInt()) {
        case 0:
            break;
        case 1:
            break;
        }
    }
    
    inline static void periodic_slow() {
    }
    
    inline static State mState = State::Undefined;
};

using fsm = FSM;

using isrRegistrar = AVR::IsrRegistrar<bridge::OCAHandler, bridge::OVFHandler, uart::TxHandler>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;

    hba::init();
    hbb::init();
    
//    dbg1::dir<AVR::Output>();
//    dbg2::dir<AVR::Output>();
//    dbg3::dir<AVR::Output>();

    eeprom::init();
    bridge::init();
    adcController::init();
    ppm::init();
    pwm::init<fPWM>();
    uart::init();
    
    led::dir<Output>();
    led::on();

    {
        Scoped<EnableInterrupt<>> ei;

        hbb::high();
        
        bridge::a(uint_ranged_NaN<uint16_t, 1000, 2000>(1100));

        uint16_t c = 0;
        
        while(true) {
            ppm::periodic();
            ppm::overfloaw([&]{
                led::toggle();
                if ((++c % 5) == 0){
                    bridge::a(uint_ranged_NaN<uint16_t, 1000, 2000>(ppm::pause));
                    etl::outl<terminal>("pa "_pgm, ppm::pause);
                    etl::outl<terminal>("pu "_pgm, ppm::pulse);
                    etl::outl<terminal>("bw "_pgm, bridge::backward);
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

ISR(TIMER0_COMPB_vect) {
}

ISR(TIMER0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::Overflow>();
}

// 1 PPM

// 2 uart
ISR(TIMER2_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
}

