#define NDEBUG

#include <stdlib.h>
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/mcupwm.h"
#include "mcu/avr/adcomparator.h"
#include "mcu/avr/watchdog.h"

#include "console.h"

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using pinLow0 = AVR::Pin<PortD, 3>;
using pinHigh0 = AVR::Pin<PortB, 4>; // mod
using pinLow1 = AVR::Pin<PortD, 4>;
using pinHigh1 = AVR::Pin<PortB, 2>;
using pinLow2 = AVR::Pin<PortD, 5>;
using pinHigh2 = AVR::Pin<PortD, 2>;

using led =  AVR::Pin<PortD, 7>;
using ppm =  AVR::Pin<PortB, 0>;

//using debugPin0 =  AVR::Pin<PortC, 4>;
//using debugPin1 =  AVR::Pin<PortC, 5>;

struct CommandAdapter {
    enum class Command : uint8_t {Undefined, Off, Step, S0, S1, S2, S3, S4, S5};
    
    static inline bool process(std::byte v) {
        switch (v) {
        case std::byte{'s'}:
            mCommand = Command::Step;
            break;
        case std::byte{'o'}:
            mCommand = Command::Off;
            break;
        case std::byte{'0'}:
            mCommand = Command::S0;
            break;
        case std::byte{'1'}:
            mCommand = Command::S1;
            break;
        case std::byte{'2'}:
            mCommand = Command::S2;
            break;
        case std::byte{'3'}:
            mCommand = Command::S3;
            break;
        case std::byte{'4'}:
            mCommand = Command::S4;
            break;
        case std::byte{'5'}:
            mCommand = Command::S5;
            break;
        default:
            //            return false;
            break;
        }        
        return true;
    }
    
    static inline Command get() {
        Command c = Command::Undefined;
        {
            Scoped<DisbaleInterrupt<>> di;
            c = mCommand;
            mCommand = Command::Undefined;
        }
        return c;
    }
    
private:
    inline static volatile Command mCommand = Command::Undefined;
};

using uart = AVR::Usart<0, CommandAdapter, MCU::UseInterrupts<true>, UseEvents<false>>;
using terminalDevice = uart;
using terminal = std::basic_ostream<terminalDevice>;

template<typename AC, typename... PP>
struct Communter {
    using pin_list = Meta::List<PP...>;
    using h0 = Meta::nth_element<0, pin_list>;
    using h1 = Meta::nth_element<1, pin_list>;
    using h2 = Meta::nth_element<2, pin_list>;
    using l0 = Meta::nth_element<3, pin_list>;
    using l1 = Meta::nth_element<4, pin_list>;
    using l2 = Meta::nth_element<5, pin_list>;
    
    static void init() {
        ((PP::template dir<AVR::Output>(), ...));
        ((PP::off(), ...));
//        AC::init();
    }
    static void off() {
        ((PP::template dir<AVR::Output>(), ...));
        (PP::off(), ...);
    }
    static void startPosition() {
        mState = 0;
    }
    static void on() {
        switch(mState) {
        case 0:
            l1::off();
            l2::on();
            break;
        case 1:
            h0::template dir<AVR::Output>();
            h1::template dir<AVR::Input>();
            break;
        case 2:
            l2::off();
            l0::on();
            break;
        case 3:
            h1::template dir<AVR::Output>();
            h2::template dir<AVR::Input>();
            break;
        case 4:
            l0::off();
            l1::on();
            break;
        case 5:
            h2::template dir<AVR::Output>();
            h0::template dir<AVR::Input>();
            break;
        default:
            break;
        }
    }
    static uint8_t state() {
        return mState;
    }
    static void next() {
        on();
        AC::channel(acNumber);
        --acNumber;
//        if ((state % 2) == 0) {
//            AC::set(AC::Mode::OnRising);
//        }
//        else {
//            AC::set(AC::Mode::OnFalling);
//        }
        ++mState;
    }
    static bool zeroCrossed() {
        return AC::get();
    }
    static bool acv() {
        return AC::getO();
    }
    enum class Error : uint8_t {OK, Internal, NoMotor};
    
    // todo: einzelne Testcases
    static Error test() {
        {
            off();
            AC::channel(0);
            AC::set(AC::Mode::OnFalling);
            bool sb = AC::getO();
            l1::on();
            l2::on();
            h0::on();
            
            Util::delay(50_us);
            
            bool t = AC::get();
            bool sa = AC::getO();
            
            h0::off();
            
            bool r = sb && !sa && t;
            
            if (!r) {
                return Error::Internal;
            }
        }
        l0::on();
        l1::on();
        l2::on();
        {
            off();
            Util::delay(1_ms);
            
            AC::channel(1);
            AC::set(AC::Mode::OnFalling);
            bool sb = AC::getO();
            l0::on();
            l2::on();
            h1::on();
            
            Util::delay(50_us);
            
            bool t = AC::get();
            bool sa = AC::getO();
            
            h1::off();
            
            bool r = sb && !sa && t;
            
            if (!r) {
                return Error::Internal;
            }
        }
        return Error::OK;
    }
private:
    inline static uint_ranged_circular<uint8_t, 0, 5> mState;
    inline static uint_ranged_circular<uint8_t, 0, 2> acNumber{1};
};

using adcomp = AVR::AdComparator<0>;
using commuter = Communter<adcomp, pinHigh0, pinHigh1, pinHigh2, pinLow0, pinLow1, pinLow2>;

//using commutationTimer = AVR::Timer16Bit<1>; // timer 1

namespace Constants {
    static constexpr std::hertz pwmFrequency = 2000_Hz * 256; 
    static constexpr std::hertz fSystem = 100_Hz;
}

using namespace std::literals::quantity;

using isrRegistrar = IsrRegistrar<uart::RxHandler, uart::TxHandler>;

using hardPwm = AVR::PWM<2>; // timer 0

int main() {
//    debugPin0::dir<AVR::Output>();
//    debugPin1::dir<AVR::Output>();
    
    commuter::init();    
    
    isrRegistrar::init();
    
    uart::init<9600>();
    
    hardPwm::init<Constants::pwmFrequency>();
    hardPwm::pwm<hardPwm::A>(98_ppc);
    
    // watchdog: alles aus
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("BL-Ctrl (mod) Test 01"_pgm);
        std::outl<terminal>("f pwm: "_pgm, hardPwm::frequency());
        
        while(true) {
            switch(CommandAdapter::get()) {
            case CommandAdapter::Command::Off:
                commuter::startPosition();
                commuter::off();
                std::outl<terminal>("Off"_pgm);
                break;
            case CommandAdapter::Command::Step:
                std::outl<terminal>("Step: "_pgm, commuter::state());
                commuter::next();
                break;
            case CommandAdapter::Command::S0:
                std::outl<terminal>("S0: "_pgm);
                pinHigh0::template dir<AVR::Input>();
                pinLow2::on();
                break;
            case CommandAdapter::Command::S1:
                std::outl<terminal>("S1: "_pgm);
                pinHigh0::template dir<AVR::Output>();
                pinHigh1::template dir<AVR::Input>();
//                pinHigh1::on();
                break;
            default:
                break;
            }
        }
    }    
}

ISR(USART_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
