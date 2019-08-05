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

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>

#include <external/units/physical.h>
#include <external/units/percent.h>

#include <external/solutions/series01/sppm_in.h>

#include <etl/fixedpoint.h>
#include <etl/output.h>

#include <std/chrono>

#include "commuter.h"
#include "sensorless.h"
#include "sensored.h"

using namespace AVR;
using namespace External::Units;
using namespace External::Units::literals;
using namespace std::literals::chrono;

namespace Constants {
    inline static constexpr hertz pwmFrequency = 20000_Hz; 
    inline static constexpr auto fRtc = 500_Hz;
}

struct CommandAdapter {
    enum class Command : uint8_t {Undefined, Off, Start, Info, Reset, 
                                  IncPwm, IncFast, DecPwm, IncDelay, DecDelay,
                                  Commute};
    
    static inline bool process(std::byte v) {
        switch (v) {
        case std::byte{'s'}:
            mCommand = Command::Start;
            break;
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
        case std::byte{'d'}:
            mCommand = Command::DecDelay;
            break;
        case std::byte{'D'}:
            mCommand = Command::IncDelay;
            break;
        case std::byte{'c'}:
            mCommand = Command::Commute;
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


template<typename... T>
struct Offloader {
    inline static void init() {
        (T::init(),...);
    }
    inline static void enable() {
        (T::enable(),...);
    }
    inline static void disable() {
        (T::disable(),...);
    }
    inline static void run() {
        (T::run(),...);
    }
};


template<typename Adc>
struct ADC_OL {
    enum class Channel : uint8_t {Temp = 3, Current = 6, Voltage = 7};
    
    inline static void init() {
        Adc::template init<AVR::HighSpeed>();    
    }
    inline static void enable() {
        Adc::channel(uint8_t(mChannel)); 
        Adc::enable();
        Adc::startConversion();
    }
    inline static void disable() {
        Adc::disable();
    }
    inline static void run() {
        Adc::whenConversionReady([](auto v) {
            ++mAdcConverions;
            switch(mChannel) {
            case Channel::Temp:
                mTemp = v;
                mChannel = Channel::Current;
                break;
            case Channel::Current:
                mCurrent= v;
                mChannel = Channel::Voltage;
                break;
            case Channel::Voltage:
                mVoltage= v;
                mChannel = Channel::Temp;
                break;
            }
        });
    }
    inline static Channel mChannel = Channel::Temp;
    inline static uint16_t mAdcConverions = 0;
    inline static uint8_t mVoltage = 0;
    inline static uint8_t mCurrent = 0;
    inline static uint8_t mTemp = 0;
};

using PortA = AVR::Port<AVR::A>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;
using PortE = AVR::Port<AVR::E>;
using PortF = AVR::Port<AVR::F>;

using pinLow0 = AVR::Pin<PortC, 1>;
using pinLow1 = AVR::Pin<PortC, 2>;
using pinLow2 = AVR::Pin<PortC, 3>;

using led =  AVR::Pin<PortF, 5>;

using hall0 =  AVR::Pin<PortF, 2>;
using hall1 =  AVR::Pin<PortF, 3>;
using hall2 =  AVR::Pin<PortF, 4>;
using hall = AVR::PinGroup<Meta::List<hall0, hall1, hall2>>;

using ppmIn =  AVR::Pin<PortE, 0>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>;

using terminalDevice = AVR::Usart<usart2Position, CommandAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<terminalDevice>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
using pwm = PWM::DynamicPwm<tcaPosition>;

using ppmTimerPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using ppm = External::Ppm::SinglePpmIn<Component::Tcb<0>>; 

using commuteTimerPosition = Portmux::Position<Component::Tcb<1>, Portmux::Default>;
using commuteTimer = SimpleTimer<Component::Tcb<1>>; 

using captureTimerPosition = Portmux::Position<Component::Tcb<2>, Portmux::Default>;
using captureTimer = Capture<Component::Tcb<2>>; 

using systemTimer = SystemTimer<Component::Rtc<0>, Constants::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using rtc_channel = Event::Channel<0, Event::Generators::PitDiv<1024>>;
using ppm_channel = Event::Channel<4, Event::Generators::Pin<ppmIn>>; 
using ac_channel = Event::Channel<2, Event::Generators::Ac0<Event::Generators::Kind::Out>>; 
using ppm_user = Event::Route<ppm_channel, Event::Users::Tcb<0>>;
using ac_user = Event::Route<ac_channel, Event::Users::Tcb<2>>;
using evrouter = Event::Router<Event::Channels<rtc_channel, ppm_channel, ac_channel>, Event::Routes<ppm_user, ac_user>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0>>;

using adcomp = AVR::AdComparator<Component::Ac<0>>; // umstelllen auf ComponentCount

using commuter = BLDC::Communter<adcomp, captureTimer, pwm, Meta::List<pinLow0, pinLow1, pinLow2>>;

using hallController = BLDC::ControllerWithHall<hall, commuteTimer, pwm, commuter>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, tcaPosition, ppmTimerPosition, captureTimerPosition>>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    evrouter::init();
    
    terminalDevice::init<AVR::BaudRate<9600>>();
    
    hall::dir<Input>();
    hall::pullup();
    
    pinLow0::dir<Output>();
    pinLow1::dir<Output>();
    pinLow2::dir<Output>();
    
    led::dir<AVR::Output>();
    
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        etl::outl<terminal>("Test20"_pgm);
        
        etl::outl<terminal>("hall: "_pgm, hall::read());            
        
        hallController::init();
        
        pwm::init();
        pwm::frequency(Constants::pwmFrequency);
        pwm::on<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>();
        
        //            pwm::duty<PWM::WO<0>>(70);
        //            pwm::duty<PWM::WO<1>>(10);
        //            pwm::duty<PWM::WO<2>>(10);
        
        //            pinLow0::on();            
        //            pinLow1::on();            
        //            pinLow2::on();            
        
        while(true) {
            terminalDevice::periodic();
            hallController::periodic();
            
            if (auto c = CommandAdapter::get(); c != CommandAdapter::Command::Undefined) {
                switch(c) {
                case CommandAdapter::Command::Off:
                    etl::outl<terminal>("Off"_pgm);
                    //                        controller2::off();
                    break;
                case CommandAdapter::Command::Commute:
                    etl::outl<terminal>("Com"_pgm);
                    pwm::duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(70);
                    commuter::next();
                    break;
                case CommandAdapter::Command::Start:
                    etl::outl<terminal>("Start"_pgm);
                    hallController::start();
                    break;
                case CommandAdapter::Command::IncPwm:
                    hallController::pwmInc();
                    etl::outl<terminal>("pwm: "_pgm, hallController::mActualPwm);
                    break;
                case CommandAdapter::Command::IncFast:
                    hallController::pwmfast();
                    etl::outl<terminal>("pwm: "_pgm, hallController::mActualPwm);
                    break;
                case CommandAdapter::Command::DecPwm:
                    hallController::pwmDec();
                    etl::outl<terminal>("pwm: "_pgm, hallController::mActualPwm);
                    break;
                case CommandAdapter::Command::IncDelay:
                    //                        controller2::mDelay += 1;
                    //                        etl::outl<terminal>("d: "_pgm, controller2::mDelay);
                    break;
                case CommandAdapter::Command::DecDelay:
                    //                        controller2::mDelay -= 1;
                    //                        etl::outl<terminal>("d: "_pgm, controller2::mDelay);
                    break;
                case CommandAdapter::Command::Reset:
                    etl::outl<terminal>("Reset"_pgm);
                    break;
                case CommandAdapter::Command::Info:
                    etl::outl<terminal>("Info"_pgm);
                    //                        etl::outl<terminal>("per: "_pgm, controller2::mLoopEstimate);
                    break;
                default:
                    break;
                }
                
            }
            led::toggle();
        }
    }
}
