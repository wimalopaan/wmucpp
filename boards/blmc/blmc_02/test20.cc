#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/port.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/adcomparator.h>
#include <mcu/internals/capture.h>
#include <mcu/internals/pwm.h>

#include <external/units/physical.h>
#include <external/units/percent.h>

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
    static constexpr hertz pwmFrequency = 16000_Hz; 
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

using PortB = AVR::Port<AVR::B>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;
using PortE = AVR::Port<AVR::E>;

using pinLow0 = AVR::Pin<PortB, 7>;
using pinHigh0 = AVR::Pin<PortD, 3>;
using pinLow1 = AVR::Pin<PortD, 5>;
using pinHigh1 = AVR::Pin<PortD, 4>;
using pinLow2 = AVR::Pin<PortD, 7>;
using pinHigh2 = AVR::Pin<PortE, 1>;

using led =  AVR::Pin<PortB, 5>;

using hall0 =  AVR::Pin<PortB, 0>;
using hall1 =  AVR::Pin<PortB, 1>;
using hall2 =  AVR::Pin<PortB, 2>;
using hall = AVR::PinSet<Meta::List<hall0, hall1, hall2>, void>;

using ppmIn =  AVR::Pin<PortE, 0>;

using oc3a =  AVR::Pin<PortD, 0>;
using oc3b =  AVR::Pin<PortD, 2>;

using rcUsart = AVR::Usart<AVR::Component::Usart<1>, CommandAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using terminalDevice = rcUsart;
using terminal = etl::basic_ostream<terminalDevice>;

using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<8>>;

using capture = AVR::Capture<AVR::Component::Timer<1>>; // timer 1

using adcomp = AVR::AdComparator<AVR::Component::Ac<0>>;
using commuter = BLDC::Communter<adcomp, capture, pinHigh0, pinHigh1, pinHigh2, pinLow0, pinLow1, pinLow2>;

using adc_ol = ADC_OL<adc>;

using offloader = Offloader<adc_ol>;

using hardPwm = AVR::PWM::StaticPwm<AVR::Component::Timer<3>, Meta::List<AVR::B>, PWM::Resolution<16>>::Fixed<Constants::pwmFrequency>; // timer 3 Achtung: Umstellen auf OC3B

using controller = BLDC::Controller<capture, hardPwm, commuter, adc, offloader>;
using controller2 = BLDC::ControllerWithHall<hall, capture, hardPwm, commuter>;


int main() {
    hall::dir<Input>();
    hall::allPullup();
    
    rcUsart::init<AVR::BaudRate<9600>>();
    
    led::dir<AVR::Output>();
    
    oc3b::dir<AVR::Output>();
    oc3b::on(); // Achtung: Output Compare Modulator Bug
    
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        etl::outl<terminal>("Test20"_pgm);
        etl::outl<terminal>(hardPwm::frequency());
        
        if (hall::read() == hall::setMask) {
            etl::outl<terminal>("no halls"_pgm);            
            
            controller::init();  
            hardPwm::init();

            while(true) {
                rcUsart::periodic();
                controller::run();
                
                if (auto c = CommandAdapter::get(); c != CommandAdapter::Command::Undefined) {
                    switch(c) {
                    case CommandAdapter::Command::Off:
                        etl::outl<terminal>("Off"_pgm);
                        controller::off();
                        break;
                    case CommandAdapter::Command::Commute:
                        etl::outl<terminal>("Com"_pgm);
                        hardPwm::b(70);
                        commuter::next();
                        break;
                    case CommandAdapter::Command::Start:
                        etl::outl<terminal>("Start"_pgm);
                        controller::start();
                        break;
                    case CommandAdapter::Command::IncPwm:
                        etl::outl<terminal>("P"_pgm);
                        controller::pwmInc();
                        break;
                    case CommandAdapter::Command::DecPwm:
                        etl::outl<terminal>("p"_pgm);
                        controller::pwmDec();
                        break;
                    case CommandAdapter::Command::IncDelay:
                        etl::outl<terminal>("D"_pgm);
                        controller::delayInc();
                        break;
                    case CommandAdapter::Command::DecDelay:
                        etl::outl<terminal>("d"_pgm);
                        controller::delayDec();
                        break;
                    case CommandAdapter::Command::Reset:
                        etl::outl<terminal>("Reset"_pgm);
                        break;
                    case CommandAdapter::Command::Info:
                        etl::outl<terminal>("Info"_pgm);
                        break;
                    default:
                        break;
                    }
                }
                led::toggle();
            }
        }
        else {
            etl::outl<terminal>("hall: "_pgm, hall::read());            

            controller2::init();  
            hardPwm::init();
            
            while(true) {
                rcUsart::periodic();
                controller2::periodic();
                
                if (auto c = CommandAdapter::get(); c != CommandAdapter::Command::Undefined) {
                    switch(c) {
                    case CommandAdapter::Command::Off:
                        etl::outl<terminal>("Off"_pgm);
                        controller2::off();
                        break;
                    case CommandAdapter::Command::Commute:
                        etl::outl<terminal>("Com"_pgm);
                        hardPwm::b(70);
                        commuter::next();
                        break;
                    case CommandAdapter::Command::Start:
                        etl::outl<terminal>("Start"_pgm);
                        controller2::start();
                        break;
                    case CommandAdapter::Command::IncPwm:
                        controller2::pwmInc();
                        etl::outl<terminal>("pwm: "_pgm, controller2::mActualPwm.toInt());
                        break;
                    case CommandAdapter::Command::IncFast:
                        controller2::pwmfast();
                        etl::outl<terminal>("pwm: "_pgm, controller2::mActualPwm.toInt());
                        break;
                    case CommandAdapter::Command::DecPwm:
                        controller2::pwmDec();
                        etl::outl<terminal>("pwm: "_pgm, controller2::mActualPwm.toInt());
                        break;
                    case CommandAdapter::Command::IncDelay:
                        controller2::mDelay += 1;
                        etl::outl<terminal>("d: "_pgm, controller2::mDelay);
                        break;
                    case CommandAdapter::Command::DecDelay:
                        controller2::mDelay -= 1;
                        etl::outl<terminal>("d: "_pgm, controller2::mDelay);
                        break;
                    case CommandAdapter::Command::Reset:
                        etl::outl<terminal>("Reset"_pgm);
                        break;
                    case CommandAdapter::Command::Info:
                        etl::outl<terminal>("Info"_pgm);
                        etl::outl<terminal>("per: "_pgm, controller2::mLoopEstimate);
                        break;
                    default:
                        break;
                    }
                    
                }
                led::toggle();
            }
        }
    }
}
