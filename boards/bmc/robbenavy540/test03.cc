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
    //    constexpr auto interval = 16_ms;
    constexpr auto interval = 16000_us;
    constexpr auto fPWM = 4000_Hz;
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

using pwm = AVR::PWM::ISRPwm<AVR::TimerNumber<0>>; // oca only

template<typename lowA, typename highA, typename lowB, typename highB, typename PWM>
struct Bridge final {
    using pwm = PWM;
    using pwm_type = typename PWM::value_type;
    
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
                    pinBp::inactivate();
                    pinAn::inactivate();
                    
                    pinBn::activate();
                    pinAp::activate();
                }
                else {
                    pinAp::inactivate();
                    pinBn::inactivate();
                    
                    pinAn::activate();
                    pinBp::activate();
                }
            }
        }
    };
    
    static inline void turnOff() {
        etl::Scoped<etl::DisbaleInterrupt<>> di;
        off = true;        
        last = 0;
        //        pwm::a(255);
    }
    
    template<typename InputType>
    static inline void set(const InputType& d, bool back) {
        using value_type = typename InputType::value_type;
        constexpr value_type span = (InputType::Upper - InputType::Lower);
        
        //        InputType::_;
        
        //        std::integral_constant<uint16_t, pwm::template max<fPWM>()>::_;
        
        if (!d) return;
        
        value_type t = (etl::enclosing_t<value_type>(d.toInt()) * pwm::template max<fPWM>()) / span;
        
        if (t == last) {
            return;
        }
        
        last = t;
        
        
        {
            //            etl::Scoped<etl::DisbaleInterrupt<>> di;
            if (t == 0) {
                off = true;
            }
            else {
                off = false;
            }
            backward = back;        
            pwm::a(t);
        }
    }
    
    static inline void setraw(uint8_t v, bool b) {
        cli();
        //        etl::Scoped<etl::DisbaleInterrupt<>> di;
        pwm::a(v);
        backward = b;
        sei();
    }
    
    static inline uint16_t last = 0;
    
    
private:
    static inline volatile bool backward = false;
    static inline volatile bool off = false;
};

using bridge = Bridge<pinAn, pinAp, pinBn, pinBp, pwm>;

using ppm = External::Ppm::IcpPpm<AVR::TimerNumber<1>, External::Ppm::RisingEdge<false>>; 




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

//using isrRegistrar = AVR::IsrRegistrar<bridge::OCAHandler, bridge::OVFHandler, uart::TxHandler>;
using isrRegistrar = AVR::IsrRegistrar<bridge::OCAHandler, bridge::OVFHandler>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    //    dbg1::dir<AVR::Output>();
    //    dbg2::dir<AVR::Output>();
    //    dbg3::dir<AVR::Output>();
    
    pwm::init<fPWM>();
    
    bridge::init();
    
    eeprom::init();
    ppm::init();
    uart::init();
    
    led::dir<Output>();
    led::on();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        uint16_t c = 0;
        
        etl::outl<terminal>("test03"_pgm);
        bridge::setraw(10, false);
        bridge::turnOff();
        
        while(true) {
            uart::periodic();
            ppm::periodic();
            ppm::overflow([&]{
                led::toggle();
                if ((++c % 5) == 0){
                    auto p = ppm::pulse();
                    outl<terminal>("p "_pgm, p.toInt());
                    bridge::setraw(10, true);
                    //                    bridge::set(p, true);
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
//ISR(TIMER2_COMPA_vect) {
////    sei();
////    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
//}

