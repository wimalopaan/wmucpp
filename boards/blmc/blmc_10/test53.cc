#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/event.h>
#include <mcu/internals/timer.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/port.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/adcomparator.h>
#include <mcu/internals/capture.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/ccl.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>

#include <external/units/physical.h>
#include <external/units/percent.h>

#include <external/solutions/series01/sppm_in.h>

#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>

#include <etl/fixedpoint.h>
#include <etl/output.h>

#include <std/chrono>

#include "commuter.h"
#include "sensorless.h"
#include "sensored.h"
#include "sine.h"

using namespace AVR;
using namespace External::Units;
using namespace External::Units::literals;
using namespace std::literals::chrono;

namespace Constants {
    inline static constexpr hertz pwmFrequency = 20000_Hz; 
    inline static constexpr auto fRtc = 512_Hz;
}

struct CommandAdapter {
    enum class Command : uint8_t {Undefined, Off, Start, Info, Reset, 
                                  IncPwm, IncFast, DecPwm, IncDelay, DecDelay,
                                  Commute, CommuteSet, Test,
                                 NextTable, PrevTable, incPeriod, decPeriod, doFloat, incSpeed, decSpeed};
    
    static inline bool process(std::byte v) {
        switch (v) {
//        case std::byte{'s'}:
//            mCommand = Command::Start;
//            break;
        case std::byte{'o'}:
            mCommand = Command::Off;
            break;
        case std::byte{'i'}:
            mCommand = Command::Info;
            break;
        case std::byte{'r'}:
            mCommand = Command::Reset;
            break;
        case std::byte{'p'}:
            mCommand = Command::DecPwm;
            break;
        case std::byte{'P'}:
            mCommand = Command::IncPwm;
            break;
        case std::byte{'F'}:
            mCommand = Command::IncFast;
            break;
        case std::byte{'f'}:
            mCommand = Command::doFloat;
            break;
        case std::byte{'d'}:
            mCommand = Command::DecDelay;
            break;
        case std::byte{'D'}:
            mCommand = Command::IncDelay;
            break;
        case std::byte{'c'}:
            mCommand = Command::Commute;
            break;
        case std::byte{'x'}:
            mCommand = Command::CommuteSet;
            break;
        case std::byte{'y'}:
            mCommand = Command::Test;
            break;
        case std::byte{'T'}:
            mCommand = Command::NextTable;
            break;
        case std::byte{'t'}:
            mCommand = Command::PrevTable;
            break;
        case std::byte{'V'}:
            mCommand = Command::incPeriod;
            break;
        case std::byte{'v'}:
            mCommand = Command::decPeriod;
            break;
        case std::byte{'S'}:
            mCommand = Command::incSpeed;
            break;
        case std::byte{'s'}:
            mCommand = Command::decSpeed;
            break;
        default:
            break;
        }        
        return true;
    }
    
    static inline Command get() {
        auto c = mCommand;
        mCommand = Command::Undefined;
        return c;
    }
    
private:
    inline static volatile Command mCommand = Command::Undefined;
};


using PortA = AVR::Port<AVR::A>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;
using PortF = AVR::Port<AVR::F>;

using lut0 = Ccl::SimpleLut<0, Ccl::Input::Tca0<0>, Ccl::Input::Mask, Ccl::Input::Mask>;
using lut1 = Ccl::SimpleLut<3, Ccl::Input::Mask, Ccl::Input::Tca0<1>, Ccl::Input::Mask>;
using lut2 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Mask, Ccl::Input::Tca0<2>>;

using pinLow0 = AVR::Ccl::LutOutPin<lut0>;
using pinLow1 = AVR::Ccl::LutOutPin<lut1>;
using pinLow2 = AVR::Ccl::LutOutPin<lut2>;

using led =  AVR::Pin<PortF, 2>;

using hall0 =  AVR::Pin<PortD, 0>;
using hall1 =  AVR::Pin<PortD, 1>;
using hall2 =  AVR::Pin<PortD, 2>;
using hall = AVR::PinGroup<Meta::List<hall0, hall1, hall2>>;

using dbg =  AVR::Pin<PortD, 6>;

using ppmIn =  AVR::Pin<PortA, 5>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>;

using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
using hott_t = Hott::hott_t;
using rcUsart = AVR::Usart<usart0Position, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

using terminalDevice = AVR::Usart<usart2Position, CommandAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<terminalDevice>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
using pwm = PWM::DynamicPwm<tcaPosition>;

using ppmTimerPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using ppm = External::Ppm::SinglePpmIn<Component::Tcb<0>>; 

using commuteTimerPosition = Portmux::Position<Component::Tcb<1>, Portmux::Default>;
using commuteTimer = SimpleTimer<Component::Tcb<1>>; 

using rotationTimerPosition = Portmux::Position<Component::Tcb<2>, Portmux::Default>;
using rotationTimer = SimpleTimer<Component::Tcb<2>>; 

//using rotationTimer = SimpleTimer<Component::Rtc<0>>;

using systemTimer = SystemTimer<Component::Pit<0>, Constants::fRtc>;
//using systemTimer = SystemTimer<Component::Rtc<0>, Constants::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

//using sensor = Hott::Experimental::Sensor<usart1Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;

using rtc_channel = Event::Channel<0, Event::Generators::PitDiv<1024>>;
using ppm_channel = Event::Channel<1, Event::Generators::Pin<ppmIn>>; 
using ac_channel = Event::Channel<2, Event::Generators::Ac0<Event::Generators::Kind::Out>>; 
using ppm_user = Event::Route<ppm_channel, Event::Users::Tcb<0>>;
using ac_user = Event::Route<ac_channel, Event::Users::Tcb<2>>;
using evrouter = Event::Router<Event::Channels<rtc_channel, ppm_channel, ac_channel>, Event::Routes<ppm_user, ac_user>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<14, 15>>;

using adcomp = AVR::AdComparator<Component::Ac<0>>; 

using commuter = BLDC::Communter<adcomp, pwm, Meta::List<pinLow0, pinLow1, pinLow2>, dbg>;

using controller = BLDC::Sine4::Controller<rotationTimer, commuteTimer, pwm, commuter, adcomp>;

using lut0 = Ccl::SimpleLut<0, Ccl::Input::Tca0<0>, Ccl::Input::Mask, Ccl::Input::Mask>;
using lut1 = Ccl::SimpleLut<3, Ccl::Input::Mask, Ccl::Input::Tca0<1>, Ccl::Input::Mask>;
using lut2 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Mask, Ccl::Input::Tca0<2>>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, tcaPosition, ppmTimerPosition>>;

using isrRegistrar = IsrRegistrar<controller::ComTimerHandler, controller::AcHandler>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    lut0::init(std::byte{0x01});
    lut1::init(std::byte{0x01});
    lut2::init(std::byte{0x01});
    
    evrouter::init();
    
    systemTimer::init();
    
    hall::template dir<Input>();
    hall::pullup();
    
    terminalDevice::init<AVR::BaudRate<9600>>();

//    sensor::init();
    rcUsart::init<BaudRate<115200>>();
    
//    led::dir<AVR::Output>();
    
//    ppmIn::dir<Input>();
//    ppm::init();

    adcController::init();
    
    etl::uint_ranged<uint16_t, 0, 160> speed{};
    
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        etl::outl<terminal>("Test52"_pgm);
        
        const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
        
        controller::init();
        
        pwm::init();
        pwm::frequency(Constants::pwmFrequency);
        pwm::on<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>();
        
        etl::outl<terminal>("m: "_pgm, pwm::max());            
        
        while(true) {
            terminalDevice::periodic();
            controller::periodic();
            rcUsart::periodic();
            adcController::periodic();
            
//            sensor::periodic();
            
            systemTimer::periodic([&]{
                constexpr adcController::index_type currIndex{0};
                const auto current = adcController::value(currIndex).toInt();
                if (speed >= 10) {
                    if (current < 126) {
                        --controller::mScale; 
                    }
                    if (current > 130) {
                        ++controller::mScale; 
                    }
                }
//                sensor::ratePeriodic();
                alarmTimer::periodic([&](const auto& t){
                    if (t == periodicTimer) {
                        etl::outl<terminal>("p: "_pgm, controller::mActualPeriod.toInt(), " t: "_pgm, controller::mActualSineTable.toInt(), " s: "_pgm, controller::mScale.toInt());            
                        etl::outl<terminal>("a0: "_pgm, current);
                    }
                    
                });
            });
            
            
            if (auto c = CommandAdapter::get(); c != CommandAdapter::Command::Undefined) {
                switch(c) {
                case CommandAdapter::Command::incSpeed:
                    ++speed;
                    controller::speed(speed);
                    etl::outl<terminal>("speed: "_pgm, speed.toInt());
                    break;
                case CommandAdapter::Command::decSpeed:
                    --speed;
                    controller::speed(speed);
                    etl::outl<terminal>("speed: "_pgm, speed.toInt());
                    break;
                case CommandAdapter::Command::doFloat:
                    controller::doFloat(!controller::mDoFloat);
                    etl::outl<terminal>("toggle float"_pgm);
                    break;
                case CommandAdapter::Command::NextTable:
                    controller::nextTable();
                    etl::outl<terminal>("NextTable: "_pgm, controller::mActualSineTable.toInt());
                    break;
                case CommandAdapter::Command::PrevTable:
                    controller::prevTable();
                    etl::outl<terminal>("PrevTable: "_pgm, controller::mActualSineTable.toInt());
                    break;
                case CommandAdapter::Command::incPeriod:
                    controller::incPeriod();
                    etl::outl<terminal>("IP: "_pgm, controller::mActualPeriod.toInt());
                    break;
                case CommandAdapter::Command::decPeriod:
                    controller::decPeriod();
                    etl::outl<terminal>("DP: "_pgm, controller::mActualPeriod.toInt());
                    break;
                case CommandAdapter::Command::Off:
                    etl::outl<terminal>("Off"_pgm);
                    speed = 0;
                    controller::speed(speed);
                    controller::off();
                    break;
                case CommandAdapter::Command::Test:
                    etl::outl<terminal>("test"_pgm);
                    pwm::duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(100);
                    pwm::on<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>();
                    break;
                case CommandAdapter::Command::Commute:
                    etl::outl<terminal>("Com"_pgm);
                    pwm::duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(100);
                    break;
                case CommandAdapter::Command::CommuteSet:
                    pwm::duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(100);
                    break;
                case CommandAdapter::Command::Start:
                    etl::outl<terminal>("Start"_pgm);
                    controller::start();
                    break;
                case CommandAdapter::Command::IncPwm:
                    controller::pwmInc();
                    etl::outl<terminal>("scale: "_pgm, controller::mScale.toInt());
                    break;
                case CommandAdapter::Command::IncFast:
//                    controller::pwmfast();
//                    etl::outl<terminal>("pwm: "_pgm, controller::mActualPwm);
                    break;
                case CommandAdapter::Command::DecPwm:
                    controller::pwmDec();
                    etl::outl<terminal>("scale: "_pgm, controller::mScale.toInt());
                    break;
                case CommandAdapter::Command::Reset:
                    etl::outl<terminal>("Reset"_pgm);
                    break;
                case CommandAdapter::Command::Info:
                    etl::outl<terminal>("Info"_pgm);
//                    etl::outl<terminal>("hall: "_pgm, hall::read());            
//                    etl::outl<terminal>("e: "_pgm, controller::mLoopEstimate);            
//                    etl::outl<terminal>("c: "_pgm, controller::mComPeriod);            
//                    etl::outl<terminal>("rpm: "_pgm, External::Units::timerValueToRPM<rotationTimer::frequency()>(controller::mLoopEstimate * 3 * 14));            
                    break;
                default:
                    break;
                }
                
            }
            led::toggle();
        }
    }
}

ISR(TCB1_INT_vect) {
    isrRegistrar::isr<typename commuteTimer::interrupt_type>();
}

ISR(AC0_AC_vect) {
    isrRegistrar::isr<AVR::ISR::AdComparator<0>::Edge>();
}


#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
#ifndef USE_HOTT
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
    while(true) {
        dbg::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
