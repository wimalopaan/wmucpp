#define NDEBUG

#include <stdlib.h>
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/mcupwm.h"
#include "mcu/avr/adcomparator.h"
#include "mcu/avr/watchdog.h"
#include "mcu/avr/twislave.h"
#include "mcu/avr/adcomparator.h"
#include "mcu/avr/adc.h"
#include "external/hott/hott.h"

#include "console.h"

namespace Constants {
    static constexpr std::hertz pwmFrequency = 2000_Hz * 256; 
    static constexpr std::hertz fSystem = 100_Hz;
}


struct CommandAdapter {
    enum class Command : uint8_t {Undefined, Off, Start, Info, Reset, 
                                  IncPwm, DecPwm, IncDelay, DecDelay,
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

template<typename AC, typename Timer, typename... PP>
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
    inline static void pwm() {
        Pin::template dir<AVR::Input>();
    }
    template<typename Pin>
    requires Meta::contains<highSides, Pin>::value
    inline static void off() {
        Pin::template dir<AVR::Output>();
    }
    template<typename Pin>
    requires Meta::contains<lowSides, Pin>::value
    inline static void floating() {
        Pin::off();
    }
    template<typename Pin>
    requires Meta::contains<lowSides, Pin>::value
    inline static void on() {
        Pin::on();
    }
    
    inline static void init() {
        ((PP::off(), ...));
        ((PP::template dir<AVR::Output>(), ...));
        AC::init();
        AC::enableCapture();
    }
    inline static void enable() {
        AC::init();
    }
    inline static void disable() {
        AC::disable();
    }
    inline static void off() {
        ((PP::template dir<AVR::Output>(), ...));
        (PP::off(), ...);
    }
    inline static void startPosition() {
        state = 0;
    }
    inline static void on() {
        switch(state) {
        case 0: // pwm(0) -> 2, ac = 1, rising
            floating<l1>();
            on<l2>();
            AC::channel(1);
            Timer::captureRising(true);
            break;
        case 1: // pwm(1) -> 2, ac = 0
            off<h0>();
            pwm<h1>();
            AC::channel(0);
            Timer::captureRising(false);
            break;
        case 2: // pwm(1) -> 0, ac = 2
            floating<l2>();
            on<l0>();
            AC::channel(2);
            Timer::captureRising(true);
            break;
        case 3: // pwm(2) -> 0, ac = 1
            off<h1>();
            pwm<h2>();
            AC::channel(1);
            Timer::captureRising(false);
            break;
        case 4: // pwm(2) -> 1, ac = 0
            floating<l0>();
            on<l1>();
            AC::channel(0);
            Timer::captureRising(true);
            break;
        case 5: // pwm(0) -> 1, ac = 2
            off<h2>();
            pwm<h0>();
            AC::channel(2);
            Timer::captureRising(false);
            break;
        default:
            break;
        }
    }
    
    inline static void next() {
        ++state;
        on();
    }
    
private:
    inline static uint_ranged_circular<uint8_t, 0, 5> state{0};
};

using namespace std::literals::quantity;

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

template<typename Timer, typename PWM, typename Com, typename xAdc, typename OL>
struct Controller {
    typedef typename Timer::value_type timer_value_t;
    typedef typename PWM::value_type pwm_value_t;
    typedef typename PWM::B pwmCH_t;
    
    static inline constexpr uint8_t pwm_max = std::numeric_limits<pwm_value_t>::max();
    
    struct RampValue {
        timer_value_t tv;
        FixedPoint<uint16_t, 8> pwm;
        
        inline RampValue& operator+=(const RampValue& rhs) {
            tv -= rhs.tv;
            pwm += rhs.pwm;
            return *this;
        }
    };
    
    struct Ramp {
        RampValue start;
        RampValue end;
        RampValue delta;
        uint8_t   steps;
    };
    
    struct RampPoint {
        std::microseconds time;
        std::percent pvm;
    };
    
    enum class State : uint8_t {Off, Align, Align2, RampUp, ClosedLoop, ClosedLoopComDelay, ClosedLoopCommute, ClosedLoopError};
    
    static inline constexpr uint16_t prescaler = 8;
    static inline constexpr auto fTimer = Config::fMcu / prescaler;

    // 190KV / 64mm    
    static inline RampPoint ramp_start = {20000_us, 5_ppc};
    static inline RampPoint ramp_end   = {2000_us, 9_ppc};
    static inline uint8_t ramp_steps = 50;
    static inline auto pwmStart = FixedPoint<uint16_t, 8>{(double)std::expand(5_ppc, uint8_t{0}, pwm_max)};

    static inline constexpr uint32_t alignValue2 = 200_ms * fTimer;
    static inline constexpr timer_value_t alignValueCount = alignValue2 / std::numeric_limits<timer_value_t>::module();
    static inline constexpr timer_value_t alignValueLast  = alignValue2 % std::numeric_limits<timer_value_t>::module();
    
    // 1000KV/37mm
//    static inline RampPoint ramp_start = {20000_us, 3_ppc};
//    static inline RampPoint ramp_end   = {2000_us, 6_ppc};
//    static inline uint8_t ramp_steps = 50;
//    static inline auto pwmStart = FixedPoint<uint16_t, 8>{(double)std::expand(5_ppc, uint8_t{0}, pwm_max)};
    
    inline static RampValue convert(const RampPoint& p) {
        RampValue v;
        v.tv = p.time * fTimer;
        v.pwm = FixedPoint<uint16_t, 8>{(double)std::expand(p.pvm, uint8_t{0}, pwm_max)};
        return v;
    } 
    
    inline static Ramp make_ramp(const RampPoint& start, const RampPoint& end, uint8_t steps) {
        Ramp ramp;
        ramp.start = convert(start);
        ramp.end = convert(end);
        ramp.steps = steps;
        ramp.delta.tv = (ramp.start.tv - ramp.end.tv) / steps;
        ramp.delta.pwm = (ramp.end.pwm - ramp.start.pwm) / steps;
        return ramp;
    }
    
    inline static Ramp ramp = make_ramp(ramp_start, ramp_end, ramp_steps);
    
    inline static RampValue actualRV = ramp.start;
    
    inline static void init() {
        OL::init();
        PWM::template init<Constants::pwmFrequency>();
        PWM::template pwm<pwmCH_t>(1);
        Com::init();
        Timer::off();
        Timer::noiseCancel();
        Timer::mode(AVR::TimerMode::IcpNoInt); 
    }
    inline static void run() {
        spin();
        if ((mState != State::Off)) {
            Timer::template periodic<Timer::flags_type::ocfa>([&](){
                periodic();    
            });            
        }
    }
private:  
    
    inline static void off() {
        Com::off();
        Timer::off();
        mState = State::Off;
    }
    inline static void start() {
        actualRV = ramp.start;
        Timer::reset();
        if (mAlignCounter < alignValueCount) {
            Timer::ocra(std::numeric_limits<timer_value_t>::max());
            ++mAlignCounter;
        }
        else {
            Timer::ocra(alignValueLast);
        }
//        Timer::ocra(alignValue);
        Timer::template prescale<prescaler>();
        mDelayFactor = 4;
        mState = State::Align;
        mAlignCounter = 0;
        mStep = 0;
    }
    
    inline static uint16_t period = 0;

    inline static void enableOffloader() {
        Com::disable();
        OL::enable();
    }
    
    inline static void disableOffloder() {
        OL::disable();
        Com::enable();
    }
    
    inline static void spin() {
        switch(mState) {
        case State::ClosedLoop:
            Timer::template periodic<Timer::flags_type::icf>([&](){ // zero crossing
//                db1::on();
                auto icr = Timer::icr();
                if (icr >= period / 4) {
                    enableOffloader();                
                    period = icr;
                    uint16_t delay = std::min(period / mDelayFactor, uint16_t{2500}); // todo: Konstante
                    if (delay < 50) { // 500ns * 50 = 25us
                        mState = State::ClosedLoopCommute;
                    }
                    else {
                        Timer::ocra(delay);
                        Timer::template clearFlag<Timer::flags_type::ocfa>();
                        mState = State::ClosedLoopComDelay;
                    }
                    Timer::reset();
                } 
                else {
                    Timer::template clearFlag<Timer::flags_type::icf>();
                }
//                db1::off();
            });
            break;
        case State::ClosedLoopCommute:
//            db2::on();
            Com::next();
            
            Timer::template clearFlag<Timer::flags_type::icf>();
            mState = State::ClosedLoop;
            
//            db2::off();
            
            OL::run();
            
            disableOffloder();

//            Timer::ocra(4 * period);
//            Timer::template clearFlag<Timer::flags_type::ocfa>();
            
            break;
        default:
            break;
        }
    }
    inline static void periodic() {
        switch(mState) {
        case State::Align:
            Com::startPosition();
            Timer::reset();
            if (mAlignCounter < alignValueCount) {
                Timer::ocra(std::numeric_limits<timer_value_t>::max());
                ++mAlignCounter;
            }
            else {
                Timer::ocra(alignValueLast);
                mState = State::RampUp;
            }
//            Timer::ocra(alignValue);
            PWM::template pwm<pwmCH_t>(actualRV.pwm.integer());
            Com::on();
            break;
        case State::RampUp:
            Com::next();
            Timer::reset();
            PWM::template pwm<pwmCH_t>(actualRV.pwm.integer());
            if (mStep < ramp_steps) {
                Timer::ocra(actualRV.tv);
                ++mStep;
                actualRV += ramp.delta;
            }
            else {
                Timer::ocra(4 * actualRV.tv); // timeout
                mState = State::ClosedLoop;
                actualRV.pwm = pwmStart;
                period = actualRV.tv;
                Timer::template clearFlag<Timer::flags_type::icf>();
            }
            break;
        case State::ClosedLoop:
//            mState = State::ClosedLoopError;
            break;
        case State::ClosedLoopComDelay:
            PWM::template pwm<pwmCH_t>(actualRV.pwm.integer());
            mState = State::ClosedLoopCommute;
            break;
        case State::ClosedLoopError:
            off();
            break;
        default: 
            assert(false);
            break;
        }
    }
    inline static void delayInc() {
        mDelayFactor += 1;
    }
    inline static void delayDec() {
        mDelayFactor = std::max(4, mDelayFactor - 1);
    }
    inline static void tvInc() {
        ramp.end.tv += 100;
    }
    inline static void tvDec() {
        ramp.end.tv -= 100;
    }
    inline static void pwmInc() {
        if (actualRV.pwm.integer() < 255) {
            actualRV.pwm += FixedPoint<uint16_t, 8>(1);
        }
    }
    inline static void pwmDec() {
        if (actualRV.pwm.integer() > 0) {
            actualRV.pwm -= FixedPoint<uint16_t, 8>(1);
        }
    }
    inline static void reset() {
        ramp = make_ramp(ramp_start, ramp_end, ramp_steps);
    }
    inline static State mState = State::Off;
    inline static  uint8_t mStep = 0;
    inline static uint8_t mDelayFactor = 4;
    inline static uint8_t mAlignCounter = 0;
};

template<typename I2c>
struct I2C_OL {
    inline static void init() {
        I2c::init();
    }
    inline static void enable() {
    }
    inline static void disable() {
    }
    inline static void run() {
        I2c::whenReady([]{});
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

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

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

using ppmIn =  AVR::Pin<PortE, 0>;

using oc3a =  AVR::Pin<PortD, 0>;
using oc3b =  AVR::Pin<PortD, 2>;

//using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0, UseEvents<false>, AsciiHandler, BinaryHandler, BCastHandler>, MCU::UseInterrupts<true>, UseEvents<false>> ;

//using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>, MCU::UseInterrupts<true>, UseEvents<false>>;
using rcUsart = AVR::Usart<1, CommandAdapter, MCU::UseInterrupts<true>, UseEvents<false>>;

using terminalDevice = rcUsart;
using terminal = std::basic_ostream<terminalDevice>;

constexpr TWI::Address address{50_B};
using i2c = TWI::Slave<0, address, 2, MCU::UseInterrupts<false>>; 

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler>;

using adc = AVR::Adc<0, AVR::Resolution<8>>;

using commutationTimer = AVR::Timer16Bit<1>; // timer 1

using adcomp = AVR::AdComparator<0>;
using commuter = Communter<adcomp, commutationTimer, pinHigh0, pinHigh1, pinHigh2, pinLow0, pinLow1, pinLow2>;

using i2c_ol = I2C_OL<i2c>;
using adc_ol = ADC_OL<adc>;

using offloader = Offloader<i2c_ol, adc_ol>;

using hardPwm = AVR::PWM<3, AVR::NonInverting>; // timer 3 Achtung: Umstellen auf OC3B

using controller = Controller<commutationTimer, hardPwm, commuter, adc, offloader>;

int main() {
    isrRegistrar::init();
    rcUsart::init<9600>();
    
    led::dir<AVR::Output>();
    
    oc3b::dir<AVR::Output>();
    oc3b::on(); // Achtung: Output Compare Modulator Bug
    
    controller::init();  
    
    i2c::init();
    
    {
        Scoped<EnableInterrupt<>> ei;
        std::outl<terminal>("Test06"_pgm);
        std::outl<terminal>(hardPwm::frequency());
        
        while(true) {
            i2c::whenReady([]{
                i2c::changed(false);
            });
            
            controller::run();
            
            if (auto c = CommandAdapter::get(); c != CommandAdapter::Command::Undefined) {
                switch(c) {
                case CommandAdapter::Command::Off:
                    std::outl<terminal>("Off"_pgm);
                    controller::off();
                    break;
                case CommandAdapter::Command::Commute:
                    std::outl<terminal>("Com"_pgm);
                    hardPwm::pwm<hardPwm::B>(3);
                    commuter::next();
                    break;
                case CommandAdapter::Command::Start:
                    std::outl<terminal>("Start"_pgm);
                    controller::start();
                    break;
                case CommandAdapter::Command::IncPwm:
                    std::outl<terminal>("P"_pgm);
                    controller::pwmInc();
                    break;
                case CommandAdapter::Command::DecPwm:
                    std::outl<terminal>("p"_pgm);
                    controller::pwmDec();
                    break;
                case CommandAdapter::Command::IncDelay:
                    std::outl<terminal>("D"_pgm);
                    controller::delayInc();
                    break;
                case CommandAdapter::Command::DecDelay:
                    std::outl<terminal>("d"_pgm);
                    controller::delayDec();
                    break;
                case CommandAdapter::Command::Reset:
                    std::outl<terminal>("Reset"_pgm);
                    controller::reset();
                    break;
                case CommandAdapter::Command::Info:
                    std::outl<terminal>("I2C 0: "_pgm, i2c::registers()[0]);
                    std::outl<terminal>("I2C 1: "_pgm, i2c::registers()[1]);
                    break;
                default:
                    break;
                }
                
            }
            led::toggle();
        }
    }
}

ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}
