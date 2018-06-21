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

using pinLow0 = AVR::Pin<PortC, 0>;
using pinHigh0 = AVR::Pin<PortC, 1>;
using pinLow1 = AVR::Pin<PortC, 2>;
using pinHigh1 = AVR::Pin<PortC, 3>;
using pinLow2 = AVR::Pin<PortC, 4>;
using pinHigh2 = AVR::Pin<PortC, 5>;

using debugPin0 =  AVR::Pin<PortB, 0>;
using debugPin1 =  AVR::Pin<PortB, 1>;

struct CommandAdapter {
    enum class Command : uint8_t {Undefined, Off, Start, Test};
    
    static inline bool process(std::byte v) {
        switch (v) {
        case std::byte{'s'}:
            mCommand = Command::Start;
            break;
        case std::byte{'o'}:
            mCommand = Command::Off;
            break;
        case std::byte{'t'}:
            mCommand = Command::Test;
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
        ((PP::off(), ...));
        ((PP::template dir<AVR::Output>(), ...));
        AC::init();
    }
    static void off() {
        ((PP::template dir<AVR::Output>(), ...));
        (PP::off(), ...);
    }
    static void startPosition() {
        state = 0;
    }
    static void on() {
        switch(state) {
        case 0:
            l1::off();
            l2::on();
            break;
        case 1:
            h0::template dir<AVR::Output>();
            //            h0::off();
            h1::template dir<AVR::Input>();
            //            h1::on();
            break;
        case 2:
            l2::off();
            l0::on();
            break;
        case 3:
            //            h1::off();
            h1::template dir<AVR::Output>();
            //            h2::on();
            h2::template dir<AVR::Input>();
            break;
        case 4:
            l0::off();
            l1::on();
            break;
        case 5:
            //            h2::off();
            h2::template dir<AVR::Output>();
            //            h0::on();
            h0::template dir<AVR::Input>();
            break;
        default:
            break;
        }
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
        ++state;
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
    inline static uint_ranged_circular<uint8_t, 0, 5> state;
    inline static uint_ranged_circular<uint8_t, 0, 2> acNumber{1};
};

using adcomp = AVR::AdComparator<0>;
using commuter = Communter<adcomp, pinHigh0, pinHigh1, pinHigh2, pinLow0, pinLow1, pinLow2>;

template<typename Timer, typename Com>
struct Controller {
    enum class State : uint8_t {Off, BeepReady, BeepFail, Check, AlignStart, AlignWait, RampUp, RunWaitCommutation, RunStart, RunA};
    
    static inline constexpr uint16_t prescaler = 64;
    static inline constexpr auto fTimer = Config::fMcu / prescaler;
    
    static inline constexpr auto rampValues = std::make_array(50_ms * fTimer, 40_ms * fTimer, 30_ms * fTimer, 
                                                              20_ms * fTimer, 15_ms * fTimer, 10_ms * fTimer,
                                                              8_ms * fTimer, 7_ms * fTimer, 6_ms * fTimer,
                                                              5_ms * fTimer, 5_ms * fTimer, 5_ms * fTimer,
                                                              5_ms * fTimer, 5_ms * fTimer, 5_ms * fTimer,
                                                              5_ms * fTimer, 5_ms * fTimer, 5_ms * fTimer,
                                                              5_ms * fTimer, 5_ms * fTimer, 5_ms * fTimer,
                                                              5_ms * fTimer, 5_ms * fTimer, 5_ms * fTimer,
                                                              5_ms * fTimer, 5_ms * fTimer, 5_ms * fTimer,
                                                              5_ms * fTimer, 5_ms * fTimer, 5_ms * fTimer,
                                                              5_ms * fTimer, 5_ms * fTimer, 5_ms * fTimer,
                                                              5_ms * fTimer, 5_ms * fTimer, 5_ms * fTimer,
                                                              5_ms * fTimer, 5_ms * fTimer, 5_ms * fTimer
                                                              );
//    static inline constexpr auto rampValues = std::make_array(20_ms * fTimer, 15_ms * fTimer, 10_ms * fTimer, 
//                                                              8_ms * fTimer, 6_ms * fTimer, 5_ms * fTimer);
//    static inline constexpr auto rampValues = std::make_array(40_ms * fTimer, 30_ms * fTimer, 20_ms * fTimer, 
//                                                              15_ms * fTimer, 10_ms * fTimer, 7_ms * fTimer, 
//                                                              4_ms * fTimer, 3_ms * fTimer, 2_ms * fTimer);
    static inline constexpr uint16_t alignValue = 50_ms * fTimer;
    
    static void init() {
        Com::init();
        Timer::off();
        Timer::mode(AVR::TimerMode::CTCNoInt); // phase corrector mode
    }
    static void run() {
        spin();
        if (mState != State::Off) {
            Timer::template periodic<Timer::flags_type::ocfa>([&](){
                periodic();    
            });            
        }
    }
private:  
    static void off() {
        Com::off();
        Timer::off();
        mState = State::Off;
    }
    static void start() {
        Timer::reset();
        Timer::ocra(alignValue);
        Timer::template prescale<prescaler>();
        mState = State::AlignStart;
    }
    static void spin() {
        static uint8_t zcCount = 0;
        switch(mState) {
        case State::RunWaitCommutation:
//            debugPin0::toggle();
                if (Com::acv()) {
                    debugPin0::on();
                }
                else {
                    debugPin0::off();
                }
            if (Com::zeroCrossed()) {
                ++zcCount;
            }
            break;
        default:
            zcCount = 0;
            break;
        }
    }
    static void periodic() {
        switch(mState) {
        case State::AlignStart:
            Com::startPosition();
            Timer::reset();
            Com::on();
            mState = State::AlignWait;
            break;
        case State::AlignWait:
            Timer::reset();
            rampStep = 0;
            Timer::ocra(rampValues[rampStep]);
            ++rampStep;
            mState = State::RampUp;
            break;
        case State::RampUp:
            Com::next();
            Timer::reset();
            Timer::ocra(rampValues[rampStep]);
            if (rampStep.isTop()) {
                mState = State::RunStart;
                tv = rampValues[rampStep];
            }
            ++rampStep;
            break;
        case State::RunStart:
            Com::next();
            Timer::reset();
            Timer::ocra(tv / 2);
            mState = State::RunA;
            break;
        case State::RunA:
            debugPin1::toggle();
            Timer::reset();
            Timer::ocra(tv * 2);
            mState = State::RunWaitCommutation;
            break;
        case State::RunWaitCommutation:
            debugPin0::off();
            debugPin1::off();
            off();
            break;
        default: 
            assert(false);
            break;
        }
    }
    inline static uint16_t tv = 0;
    inline static State mState = State::Off;
    inline static uint_ranged<uint8_t, 0, rampValues.size - 1> rampStep;
};

using commutationTimer = AVR::Timer16Bit<1>; // timer 1

using controller = Controller<commutationTimer, commuter>;

namespace Constants {
    static constexpr std::hertz pwmFrequency = 2000_Hz * 256; 
    static constexpr std::hertz fSystem = 100_Hz;
}

using namespace std::literals::quantity;

using isrRegistrar = IsrRegistrar<uart::RxHandler, uart::TxHandler>;

using hardPwm = AVR::PWM<0>; // timer 0
//using systemClock = AVR::Timer8Bit<2>; // timer 2

int main() {
    debugPin0::dir<AVR::Output>();
    debugPin1::dir<AVR::Output>();
    
//    systemClock::template setup<Constants::fSystem>(AVR::TimerMode::CTCNoInt);
    
    isrRegistrar::init();
    
    uart::init<9600>();
    
    controller::init();  
    
    hardPwm::init<Constants::pwmFrequency>();
    hardPwm::pwm<hardPwm::A>(80_ppc);
    
    // watchdog: alles aus
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test00 V01"_pgm);
//        std::outl<terminal>("f pwm: "_pgm, hardPwm::frequency());
//        std::outl<terminal>("rv b: "_pgm, controller::rampValues[0]);
//        std::outl<terminal>("rv e: "_pgm, controller::rampValues[controller::rampValues.size - 1]);
        
        while(true) {
            controller::run();
            
            switch(CommandAdapter::get()) {
            case CommandAdapter::Command::Off:
                std::outl<terminal>("Off"_pgm);
                controller::off();
                break;
            case CommandAdapter::Command::Start:
                std::outl<terminal>("Start"_pgm);
                controller::start();
                break;
            case CommandAdapter::Command::Test:
                std::outl<terminal>("Test"_pgm);
                if (commuter::test() == commuter::Error::Internal) {
                    std::outl<terminal>("Err_I"_pgm);
                }
                else if (commuter::test() == commuter::Error::NoMotor) {
                    std::outl<terminal>("Err_NoM"_pgm);
                }
                else {
                    std::outl<terminal>("OK"_pgm);
                }
                break;
            default:
                break;
            }
        }
    }    
}

ISR(USART0_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
