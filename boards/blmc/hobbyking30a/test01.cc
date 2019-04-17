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
    constexpr auto interval = 16_ms;
    constexpr auto fPWM = 4000_Hz;
}


template<typename... PA>
struct MetaPA {
    using index_type = etl::uint_ranged<uint8_t, 0, sizeof...(PA) - 1>;
    using adapters = Meta::List<PA...>;
    using s = Meta::front<adapters>;

    inline static void activatePA(index_type index) {
        state = index;
    }
    inline static bool process(std::byte b) {
        Meta::visitAt<adapters>(state, [&]<typename P>(Meta::Wrapper<P>){
              P::process(b);
        });
        return true;
    }
private:
    inline static index_type state{0};
};

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

using pinAp = AVR::ActiveLow<AVR::Pin<PortC, 3>, AVR::Output>;
using pinAn = AVR::ActiveHigh<AVR::Pin<PortB, 0>, AVR::Output>;
using pinBp = AVR::ActiveLow<AVR::Pin<PortC, 5>, AVR::Output>;
using pinBn = AVR::ActiveHigh<AVR::Pin<PortC, 4>, AVR::Output>;
using pinCp = AVR::ActiveLow<AVR::Pin<PortD, 4>, AVR::Output>;
using pinCn = AVR::ActiveHigh<AVR::Pin<PortD, 5>, AVR::Output>;

using dbg1 = AVR::Pin<PortB, 3>; // MOSI
using dbg2 = AVR::Pin<PortB, 4>; // MISO
using dbg3 = AVR::Pin<PortB, 5>; // SCK

using hba = HalfBridge<pinAp, pinAn>;
using hbb = HalfBridge<pinBp, pinBn>;
using hbc = HalfBridge<pinCp, pinCn>;

using pwm = AVR::ISRPwm<1>;

template<typename HBA, typename HBB, typename HBC, typename PWM, typename InputType>
struct Bridge {
    using pwm = PWM;
    using value_type = typename PWM::value_type;
    using input_type = InputType;
    
    inline static constexpr value_type medium = (input_type::Upper + input_type::Lower) / 2;
    inline static constexpr value_type span = (input_type::Upper - input_type::Lower) / 2;
    
    inline static void init() {
        HBA::init();
        HBB::init();
        HBC::init();
    }    
    struct OCAHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<1>::CompareA> {
        inline static void isr() {
            HBA::low();
        }
    };
    struct OCBHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<1>::CompareB>  {
        inline static void isr() {
            HBB::low();
        }
    };
    struct OVFHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<1>::Overflow>  {
        inline static void isr() {
            HBA::high();
            HBB::high();
        }
    };
    static inline void a(const input_type& d) {
        if (!d) return;
        
        value_type v = (d.toInt() >= medium) ? (d.toInt() - medium) : (medium - d.toInt());
        value_type t = (etl::enclosing_t<value_type>(v) * pwm::template max<fPWM>()) / span;
        
        if (d.toInt() >= medium) {
            HBC::low();
        }
        else {
            HBC::high();
        }
        pwm::a(t);
    }
    static inline void b(const input_type& d) {
        if (!d) return;
        
        value_type v = (d.toInt() >= medium) ? (d.toInt() - medium) : (medium - d.toInt());
        value_type t = (etl::enclosing_t<value_type>(v) * pwm::template max<fPWM>()) / span;
        
        pwm::b(t);
    }
};


using sumdPA = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
using qtroboPA = External::QtRobo::ProtocollAdapter<0>;
using metaPA = MetaPA<sumdPA, qtroboPA>;

using bridge = Bridge<hba, hbb, hbc, pwm, sumdPA::value_type>;

using rcUsart = AVR::Usart<0, metaPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using terminalDevice = rcUsart;
using terminal = etl::basic_ostream<terminalDevice>;

using pong = External::QtRobo::Pong<terminal>;

using adc = AVR::Adc<0>;
using adcController = External::Hal::AdcController<adc, 0, 1, 2, 7, 6>;

using systemClock = AVR::SystemTimer<0, interval>;
using alarmTimer = External::Hal::AlarmTimer<systemClock>;

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

struct PPMInput : public AVR::IsrBaseHandler<AVR::ISR::Int<0>> {
    inline static void isr() {}
};


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
                metaPA::activatePA(0);                            
            }
            else if (mState == State::Robo) {
                metaPA::activatePA(1);                                        
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
            bridge::a(sumdPA::value(0));
            break;
        case 1:
            bridge::b(sumdPA::value(1));
            break;
        }
    }
    template<typename T>
    inline static void update_robo(T c) {
        switch(c.toInt()) {
        case 0:
            bridge::a(qtroboPA::propValues[0]);
            break;
        case 1:
            bridge::b(qtroboPA::propValues[0]);
            break;
        }
    }
    
    inline static void periodic_slow() {
        if (sumdPA::packageCount() > 10) {
            sumdPA::resetCount();
            process(Event::SumdConnected);
        }
        else {
            qtroboPA::whenPinged([]{
                pong::put();
                process(Event::RoboConnected);
            });
        }
    }
    
    inline static State mState = State::Undefined;
};

using fsm = FSM;

using isrRegistrar = AVR::IsrRegistrar<PPMInput, bridge::OCAHandler, bridge::OCBHandler, bridge::OVFHandler>;

template<typename... CC>
struct StaticInitializer {
    StaticInitializer() {
        (CC::init(), ...);
    }
};

using Initializer = StaticInitializer<systemClock, eeprom, bridge, adcController>;

namespace {
    Initializer initializer;
}

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;

    dbg1::dir<AVR::Output>();
    dbg2::dir<AVR::Output>();
    dbg3::dir<AVR::Output>();

    rcUsart::init<115200>();
    metaPA::activatePA(0);
    
    pwm::init<fPWM>();
    
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

    {
        Scoped<EnableInterrupt<>> ei;

        while(true) {
            rcUsart::periodic();
            
            systemClock::periodic([&](){
                fsm::periodic();
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t) {
                        fsm::periodic_slow();
//                        etl::outl<terminal>("v1: "_pgm, v1.toInt());
//                        etl::outl<terminal>("v2: "_pgm, v2.toInt());
//                        etl::outl<terminal>("c: "_pgm, sumdPA::packageCount());
                        appData.expire();
                    }
                });
            });
            if(eeprom::saveIfNeeded()) {
                etl::out<terminal>("."_pgm);
            }    
        }
    }
}

ISR(INT0_vect) {
    isrRegistrar::isr<AVR::ISR::Int<0>>();
}

ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}

ISR(TIMER1_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareB>();
}

ISR(TIMER1_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::Overflow>();
}

//// SumD
//ISR(USART1_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
//}
//ISR(USART1_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
//}
