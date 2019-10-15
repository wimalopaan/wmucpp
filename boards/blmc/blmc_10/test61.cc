#define NDEBUG

//#define QT_ROBO

#include <etl/trace.h>

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
#include <mcu/internals/eeprom.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>

#include <external/units/physical.h>
#include <external/units/percent.h>

#include <external/solutions/series01/sppm_in.h>
#include <external/bluetooth/qtrobo.h>

#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/hott.h>
#include <external/hott/menu.h>

#include <etl/fixedpoint.h>
#include <etl/output.h>

#include <std/chrono>

#include "commuter2.h"
#include "sensorless.h"
#include "sensored.h"
#include "sine10.h"

#ifndef NDEBUG
# define TRACE_FILENAME "ma"
#endif

using namespace AVR;
using namespace External::Units;
using namespace External::Units::literals;
using namespace std::literals::chrono;

namespace Constants {
    inline static constexpr hertz pwmFrequency = 20000_Hz; 
    inline static constexpr auto fRtc = 512_Hz;
    constexpr inline static uint8_t menuLines = 8;
}

namespace Storage {
    enum class AVKey : uint8_t {TimeOut = 0, SoftStart, SensorType, Magic0, Magic1, _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        using value_type = etl::uint_NaN<uint8_t>;
        value_type& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<value_type, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
}

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

template<typename Adc, uint8_t Channel = 0>
struct CurrentSensor {
    inline static constexpr typename Adc::index_type currIndex{Channel};
    
    inline static void init() {
    }
    
    inline static uint16_t value() {
        return Adc::value(currIndex).toInt();
    }
    
};

struct CommandAdapter {
    enum class Command : uint8_t {Undefined, Off, Info, Reset, 
                                  Test,
                                  incSpeed, decSpeed,
                                  incSpeedFast, decSpeedFast,
                                  
                                 };
    
    static inline bool process(std::byte v) {
        switch (v) {
        case std::byte{'o'}:
            mCommand = Command::Off;
            break;
        case std::byte{'i'}:
            mCommand = Command::Info;
            break;
        case std::byte{'r'}:
            mCommand = Command::Reset;
            break;
        case std::byte{'y'}:
            mCommand = Command::Test;
            break;
        case std::byte{'S'}:
            mCommand = Command::incSpeed;
            break;
        case std::byte{'s'}:
            mCommand = Command::decSpeed;
            break;
        case std::byte{'A'}:
            mCommand = Command::incSpeedFast;
            break;
        case std::byte{'a'}:
            mCommand = Command::decSpeedFast;
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

using dbg  =  AVR::Pin<PortD, 6>;
using dbg2 =  AVR::Pin<PortC, 2>;

using ppmIn =  AVR::Pin<PortA, 6>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>;

using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
using hott_t = Hott::hott_t;
using rcUsart = AVR::Usart<usart0Position, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifdef QT_ROBO
using qtrobo = External::QtRobo::ProtocollAdapter<0>;
using qtDevice = AVR::Usart<usart2Position, qtrobo, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using qt = etl::basic_ostream<qtDevice>;
using log0 = External::QtRobo::Logging<qt, 0>;
#else
using terminalDevice = AVR::Usart<usart2Position, CommandAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<terminalDevice>;
#endif

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
using pwm = PWM::DynamicPwm<tcaPosition>;

using ppmTimerPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using ppm = External::Ppm::SinglePpmIn<Component::Tcb<0>>; 

using commuteTimerPosition = Portmux::Position<Component::Tcb<1>, Portmux::Default>;
using commuteTimer = SimpleTimer<Component::Tcb<1>>; 

//using rotationTimerPosition = Portmux::Position<Component::Tcb<2>, Portmux::Default>;
//using rotationTimer = ExtendedTimer<Component::Tcb<2>>; 

using rotationTimer = SimpleTimer<Component::Rtc<0>>;

using systemTimer = SystemTimer<Component::Pit<0>, Constants::fRtc>;
//using systemTimer = SystemTimer<Component::Rtc<0>, Constants::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using sensor = Hott::Experimental::Sensor<usart1Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;

using rtc_channel = Event::Channel<0, Event::Generators::PitDiv<1024>>;
using ppm_channel = Event::Channel<1, Event::Generators::Pin<ppmIn>>; 
//using ac_channel = Event::Channel<2, Event::Generators::Ac0<Event::Generators::Kind::Out>>; 
//using userstrobe_channel = Event::Channel<3, void>; 
using ppm_user = Event::Route<ppm_channel, Event::Users::Tcb<0>>;
//using ac_user = Event::Route<ac_channel, Event::Users::Tcb<2>>;
//using userstrobe_user = Event::Route<userstrobe_channel, Event::Users::Tcb<2>>;
using evrouter = Event::Router<Event::Channels<rtc_channel, ppm_channel>, Event::Routes<ppm_user>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<14, 15>>;
using currentSensor = CurrentSensor<adcController, 0>;

using adcomp = AVR::AdComparator<Component::Ac<0>>; 

using commuter = BLDC2::Communter<adcomp, pwm, Meta::List<pinLow0, pinLow1, pinLow2>, void>;

using controller = BLDC::Sine10::Controller<rotationTimer, commuteTimer, pwm, commuter, adcomp, currentSensor, void>;

using lut0 = Ccl::SimpleLut<0, Ccl::Input::Tca0<0>, Ccl::Input::Mask, Ccl::Input::Mask>;
using lut1 = Ccl::SimpleLut<3, Ccl::Input::Mask, Ccl::Input::Tca0<1>, Ccl::Input::Mask>;
using lut2 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Mask, Ccl::Input::Tca0<2>>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, tcaPosition, ppmTimerPosition>>;

//using isrRegistrar = IsrRegistrar<controller::ComTimerHandler, controller::AcHandler, rotationTimer::OverflowHandler>;
using isrRegistrar = IsrRegistrar<controller::ComTimerHandler, controller::AcHandler>;


class RCMenu final : public Hott::Menu<Constants::menuLines> {
public:
    constexpr RCMenu() : Menu(this, "WM-ESC 1.0"_pgm, &mSoft, &mTimeout, &mType) {}
private:
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mSoft{"Start"_pgm, appData, Storage::AVKey::SoftStart, 2};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mTimeout{"TimeOut"_pgm, appData, Storage::AVKey::TimeOut, 10};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mType{"Type"_pgm, appData, Storage::AVKey::SensorType, 2};
};


template<typename PA, typename TopMenu>
class HottMenu final {
    HottMenu() = delete;
public:
    inline static void init() {
        clear();
    }
    inline static void periodic() {
        PA::processKey([&](Hott::key_t k){
            mMenu = mMenu->processKey(k);
            clear();
        });
        mMenu->textTo(PA::text());
    }
private:
    inline static void clear() {
        mMenu->reset();
        for(auto& line : PA::text()) {
            line.clear();
        }
    }
    constinit inline static TopMenu mTopMenu;
    constinit inline static Hott::Menu<PA::menuLines>* mMenu = &mTopMenu;
};

using menu = HottMenu<sensor, RCMenu>;


int main() {
    TRACE;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    eeprom::init();
    
    lut0::init(std::byte{0x01});
    lut1::init(std::byte{0x01});
    lut2::init(std::byte{0x01});
    
    evrouter::init();
    
    systemTimer::init();
    
    hall::template dir<Input>();
    hall::pullup();
#ifdef QT_ROBO
    qtDevice::init<AVR::BaudRate<9600>>();
#else
    terminalDevice::init<AVR::BaudRate<9600>>();
#endif
    sensor::init();
    rcUsart::init<BaudRate<115200>, HalfDuplex>();
    
    dbg::dir<AVR::Output>();
    dbg2::dir<AVR::Output>();
    
    led::dir<AVR::Output>();
    
    ppmIn::dir<Input>();
    ppm::init();
    
    adcController::init();
    
    menu::init();
    
    controller::speed_t speed{};
    
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
#ifndef QT_ROBO
        etl::outl<terminal>("Test61"_pgm);
#endif
        const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
        
        controller::init();
        
        pwm::init();
        pwm::frequency(Constants::pwmFrequency);
        pwm::on<Meta::List<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>>();
        
#ifndef QT_ROBO
        etl::outl<terminal>("m: "_pgm, pwm::max());            
        
        for(auto s: controller::sineSpeeds) {
            etl::outl<terminal>("speed: "_pgm, s.toInt());            
        }
#endif        
        while(true) {
#ifdef QT_ROBO
            qtDevice::periodic();
#else
            terminalDevice::periodic();
#endif
            controller::periodic();
            rcUsart::periodic();
            adcController::periodic();
            sensor::periodic();
            
            systemTimer::periodic([&]{
                TRACE;
                controller::ratePeriodic();
                sensor::ratePeriodic();
                appData.expire();
                menu::periodic();
                
#ifdef QT_ROBO
                qtrobo::whenChanged([&]{
                    speed = qtrobo::propValues[0];
                    controller::speed(speed);
                });
#endif
                
                alarmTimer::periodic([&](const auto& t){
                    if (t == periodicTimer) {
#ifndef QT_ROBO
                        etl::outl<terminal>("S: "_pgm, uint8_t(controller::mState), " cPID: "_pgm, controller::currentPid, " pPID: "_pgm, controller::periodPid);
                        //                        etl::outl<terminal>(" apwm: "_pgm, controller::mActualPwm, " s: "_pgm, controller::mScale
                        //                                            , " ape: "_pgm, controller::mActualPeriodEstimate, " dp: "_pgm, controller::desiredRtcPeriod()
                        //                                            , " npw: "_pgm, controller::npwm
                        //                                            , " ap: "_pgm, controller::speed_value.period().toInt(), " at: "_pgm, controller::speed_value.sineTable().toInt()
                        //                                            );
                        if (auto p = sumd::value(2); p) {
                            etl::outl<terminal>("sumd3: "_pgm, p.toInt());
                        }
                        if (auto p = ppm::value(); p) {
                            etl::outl<terminal>("ppm: "_pgm, p.toInt());
                            int16_t s = (ppm::value().toInt() - 15000) / 8; 
                            if (s > 0) {
                                //                                                speed = s;
                                //                        //                        controller::speed(s);
                            }
                        }
#else
                        log0::outl(speed.toInt());
#endif                            
                    }
                });
            });
            
            
#ifndef QT_ROBO
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
                case CommandAdapter::Command::incSpeedFast:
                    speed += 20;;
                    controller::speed(speed);
                    etl::outl<terminal>("speed: "_pgm, speed.toInt());
                    break;
                case CommandAdapter::Command::decSpeedFast:
                    speed -= 20;
                    controller::speed(speed);
                    etl::outl<terminal>("speed: "_pgm, speed.toInt());
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
                    pwm::on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();
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
#endif
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
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    while(!terminalDevice::isEmpty()) {
        terminalDevice::periodic();
    }
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(!terminalDevice::isEmpty()) {
        terminalDevice::periodic();
    }
    etl::outl<terminal>("-> "_pgm, etl::Char{Trace::file[0]}, etl::Char{Trace::file[1]}, etl::Char{','}, Trace::line);
    while(!terminalDevice::isEmpty()) {
        terminalDevice::periodic();
    }
}

template<typename String1, typename String2>
/*[[noreturn]] */inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
