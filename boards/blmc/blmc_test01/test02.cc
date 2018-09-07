#define NDEBUG

#include <stdlib.h>
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/mcupwm.h"
#include "mcu/avr/adcomparator.h"
#include "mcu/avr/watchdog.h"
#include "container/fixedvector.h"

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

using db0 =  AVR::Pin<PortB, 0>;
using db1 =  AVR::Pin<PortB, 6>;
using db2 =  AVR::Pin<PortB, 7>;

namespace Constants {
    static constexpr std::hertz pwmFrequency = 20000_Hz * 256; 
    static constexpr std::hertz fSystem = 100_Hz;
}


struct CommandAdapter {
    enum class Command : uint8_t {Undefined, Off, Start, Test, Info, OpenLoop, ClosedLoop, IncTime, DecTime, Reset};
    
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
        case std::byte{'i'}:
            mCommand = Command::Info;
            break;
        case std::byte{'O'}:
            mCommand = Command::OpenLoop;
            break;
        case std::byte{'C'}:
            mCommand = Command::ClosedLoop;
            break;
        case std::byte{'+'}:
            mCommand = Command::IncTime;
            break;
        case std::byte{'-'}:
            mCommand = Command::DecTime;
            break;
        case std::byte{'r'}:
            mCommand = Command::Reset;
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
    
    using highSides = Meta::List<h0, h1, h2>;
    using lowSides  = Meta::List<l0, l1, l2>;
    
    template<typename Pin>
    requires Meta::contains<highSides, Pin>::value
    static void pwm() {
        Pin::template dir<AVR::Input>();
    }
    template<typename Pin>
    requires Meta::contains<highSides, Pin>::value
    static void off() {
        Pin::template dir<AVR::Output>();
    }
    template<typename Pin>
    requires Meta::contains<lowSides, Pin>::value
    static void floating() {
        Pin::off();
    }
    template<typename Pin>
    requires Meta::contains<lowSides, Pin>::value
    static void on() {
        Pin::on();
    }
    
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
    static void adcClear() {
        AC::reset();
    }
    static void on() {
        switch(state) {
        case 0: // pwm(0) -> 2, ac = 1, rising
            floating<l1>();
            on<l2>();
            AC::channel(1);
            AC::set(AC::Mode::OnRising);
            break;
        case 1: // pwm(1) -> 2, ac = 0
            off<h0>();
            pwm<h1>();
            AC::channel(0);
            AC::set(AC::Mode::OnFalling);
            break;
        case 2: // pwm(1) -> 0, ac = 2
            floating<l2>();
            on<l0>();
            AC::channel(2);
            AC::set(AC::Mode::OnRising);
            break;
        case 3: // pwm(2) -> 0, ac = 1
            off<h1>();
            pwm<h2>();
            AC::channel(1);
            AC::set(AC::Mode::OnFalling);
            break;
        case 4: // pwm(2) -> 1, ac = 0
            floating<l0>();
            on<l1>();
            AC::channel(0);
            AC::set(AC::Mode::OnRising);
            break;
        case 5: // pwm(0) -> 1, ac = 2
            off<h2>();
            pwm<h0>();
            AC::channel(2);
            AC::set(AC::Mode::OnFalling);
            break;
        default:
            break;
        }
    }
    static void next() {
        ++state;
        on();
        last_state = state;
    }
    static bool zeroCrossed() {
        return AC::get();
    }
    static bool acv() {
        return AC::getO();
    }
    
    // todo: pwm
    static bool test_1() {
        off();
        return true;
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
    inline static uint_ranged_circular<uint8_t, 0, 5> state{0};
    inline static uint_ranged_circular<uint8_t, 0, 5> last_state;
};

using adcomp = AVR::AdComparator<0>;
using commuter = Communter<adcomp, pinHigh0, pinHigh1, pinHigh2, pinLow0, pinLow1, pinLow2>;

using namespace std::literals::quantity;

template<typename Timer, typename PWM, typename Com>
struct Controller {
    typedef typename Timer::value_type timer_value_t;
    typedef typename PWM::value_type pwm_value_t;
    
    static inline constexpr uint8_t pwm_max = std::numeric_limits<pwm_value_t>::max();
    
//    timer_value_t::_;
//    pwm_value_t::_;
    
    struct RampValue {
        timer_value_t tv;
        pwm_value_t   pwm;
    };
    
    struct RampPoint {
        std::microseconds time;
        std::percent pvm;
    };
    
    enum class State : uint8_t {Off, BeepReady, BeepFail, Check, AlignStart, AlignWait, RampUp, RunWaitCommutation, RunStart, RunA};
    
    static inline constexpr uint16_t prescaler = 8;
    static inline constexpr auto fTimer = Config::fMcu / prescaler;
    
    static inline RampPoint ramp_start = {20000_us, 4_ppc};
    static inline RampPoint ramp_end   = {3000_us, 8_ppc};
    static inline constexpr size_t ramp_size = 150;
    static inline std::FixedVector<RampValue, ramp_size> rampValues;;
    
    static inline constexpr uint16_t alignValue = 50_ms * fTimer;

    static inline constexpr uint8_t tvFaction = 16;
    
    template<typename C>
    inline static void fill(C& values) {
        for(size_t n = 0; n < values.capacity; ++n) {
            RampValue v;
            v.tv = ramp_start.time * fTimer - ((ramp_start.time * fTimer - ramp_end.time * fTimer) / (values.capacity - 1)) * n;
            v.pwm = std::expand(ramp_start.pvm, uint8_t{0}, pwm_max) + ((std::expand(ramp_end.pvm, uint8_t{0}, pwm_max) - std::expand(ramp_start.pvm, uint8_t{0}, pwm_max)) * n) / (values.capacity - 1);
            values[n] = v;
        }
    }
    
    static void init() {
        fill(rampValues);
        PWM::template init<Constants::pwmFrequency>();
        PWM::template pwm<typename PWM::A>(255);
        Com::init();
        Timer::off();
        Timer::mode(AVR::TimerMode::CTCNoInt); // phase correct mode
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
//        static uint8_t zcCount = 0;
        switch(mState) {
        case State::RunWaitCommutation:
            if (!Com::acv()) {
                db2::on();
            }
            else {
                db2::off();
            }
//            if (Com::zeroCrossed()) {
//                db2::toggle();
//                Com::next();
//                Timer::reset();
//                Timer::ocra(tv / tvFaction);
//                mState = State::RunA;
//                Com::adcClear();
//            }
//            else {
//                db2::off();
//            }
//            if (Com::zeroCrossed()) {
//                ++zcCount;
//            }
            break;
        default:
//            zcCount = 0;
            break;
        }
    }
    static void periodic() {
        switch(mState) {
        case State::AlignStart:
            db1::off();
            db2::off();
            Com::startPosition();
            Timer::reset();
            PWM::template pwm<typename PWM::A>(255 - rampValues[0].pwm);
            Com::on();
            mState = State::AlignWait;
            break;
        case State::AlignWait:
            Timer::reset();
            rampStep = 0;
            Timer::ocra(rampValues[rampStep].tv);
            PWM::template pwm<typename PWM::A>(255 - rampValues[rampStep].pwm);
            ++rampStep;
            mState = State::RampUp;
            break;
        case State::RampUp:
            Com::next();
            Timer::reset();
            Timer::ocra(rampValues[rampStep].tv);
            PWM::template pwm<typename PWM::A>(255 - rampValues[rampStep].pwm);
            if (rampStep.isTop()) {
                if (!openMode) {
                    mState = State::RunStart;
                }
                tv = rampValues[rampStep].tv;
            }
            ++rampStep;
            break;
        case State::RunStart:
            Com::next();
            Timer::reset();
            Timer::ocra(tv / tvFaction);
            mState = State::RunA;
            Com::adcClear();
            break;
        case State::RunA:
            db1::on();
            Timer::reset();
            Timer::ocra((tv * (tvFaction - 1)) / tvFaction);
            mState = State::RunWaitCommutation;
            break;
        case State::RunWaitCommutation:
            db1::off();
            off();
            break;
        default: 
            assert(false);
            break;
        }
    }
    inline static void targetModeOpen(bool v = true) {
        openMode = v;
    }
    inline static void tvInc(std::microseconds v) {
        ramp_end.time += v;
        fill(rampValues);
    }
    inline static void tvDec(std::microseconds v) {
        ramp_end.time -= v;
        fill(rampValues);
    }
    inline static void reset() {
        fill(rampValues);
    }
    inline static bool openMode = false;
    inline static uint16_t tv = 0;
    inline static State mState = State::Off;
    inline static uint_ranged<uint8_t, 0, rampValues.capacity - 1> rampStep;
};

using commutationTimer = AVR::Timer16Bit<1>; // timer 1

using hardPwm = AVR::PWM<2>; // timer 0

using controller = Controller<commutationTimer, hardPwm, commuter>;

using isrRegistrar = IsrRegistrar<uart::RxHandler, uart::TxHandler>;

//using systemClock = AVR::Timer8Bit<2>; // timer 2

int main() {
    OSCCAL = 0xff; // 16 MHz
    
    db0::dir<AVR::Output>();
    db1::dir<AVR::Output>();
    db2::dir<AVR::Output>();
    
    //    systemClock::template setup<Constants::fSystem>(AVR::TimerMode::CTCNoInt);
    
    isrRegistrar::init();
    
    uart::init<9600>();
    
    controller::init();  
    
    // watchdog: alles aus
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("BL-Ctrl (mod) V01"_pgm);
        std::outl<terminal>("f pwm: "_pgm, hardPwm::frequency());
        std::outl<terminal>("rv b: "_pgm, controller::rampValues[0].tv);
        std::outl<terminal>("rv e: "_pgm, controller::rampValues[controller::rampValues.capacity - 1].tv);
        std::outl<terminal>("pv b: "_pgm, controller::rampValues[0].pwm);
        std::outl<terminal>("pv e: "_pgm, controller::rampValues[controller::rampValues.capacity - 1].pwm);
        
        while(true) {
            db0::toggle();
            controller::run();
            
            if (auto c = CommandAdapter::get(); c != CommandAdapter::Command::Undefined) {
                switch(c) {
                case CommandAdapter::Command::Off:
                    std::outl<terminal>("Off"_pgm);
                    controller::off();
                    break;
                case CommandAdapter::Command::Start:
                    std::outl<terminal>("Start"_pgm);
                    controller::start();
                    break;
                case CommandAdapter::Command::OpenLoop:
                    std::outl<terminal>("OpenLoop"_pgm);
                    controller::targetModeOpen(true);
                    break;
                case CommandAdapter::Command::ClosedLoop:
                    std::outl<terminal>("ClosedLoop"_pgm);
                    controller::targetModeOpen(false);
                    break;
                case CommandAdapter::Command::IncTime:
                    std::outl<terminal>("+"_pgm);
                    controller::tvInc(100_us);
                    break;
                case CommandAdapter::Command::DecTime:
                    std::outl<terminal>("-"_pgm);
                    controller::tvDec(100_us);
                    break;
                case CommandAdapter::Command::Reset:
                    std::outl<terminal>("Reset"_pgm);
                    controller::reset();
                    break;
                case CommandAdapter::Command::Info:
                    std::outl<terminal>("last: "_pgm, commuter::last_state.toInt());
                    std::outl<terminal>("rt end: "_pgm, controller::ramp_end.time);
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
}

ISR(USART_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
