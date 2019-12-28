#define NDEBUG

#define USE_TERMINAL
#define USE_HOTT

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/sleep.h>
#include <mcu/internals/pwm.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/hott.h>
#include <external/hott/menu.h>

#include <std/chrono>

#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

struct Storage final {
    Storage() = delete;
    enum class AVKey : uint8_t {OnDelay = 0, SoftStart,
                                _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        etl::uint_NaN<uint8_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<etl::uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
    
    inline static uint8_t configValue = 0;
    
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

using PortA = Port<A>;
using PortB = Port<B>;
using PortD = Port<D>;
using PortF = Port<F>;
using pf2 = Pin<PortF, 2>; 
using led = Pin<PortF, 5>; 
using fet = Pin<PortB, 1>; 
using pa0 = Pin<PortA, 0>; 
using button = Pin<PortD, 1>; // auch AIN1 

//using pe3 = Pin<PortE, 3>; // ppm (s.u.)
//using rpmPin = Pin<PortE, 0>; 

using mosfetPin = AVR::ActiveHigh<fet, AVR::Output>;
using buttonPin = AVR::ActiveHigh<button, AVR::Input>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

#ifdef USE_TERMINAL
using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;
#endif
using usart3Position = Portmux::Position<Component::Usart<3>, Portmux::Default>;

struct Parameter {
    inline static constexpr uint8_t menuLines = 4;
    inline static constexpr auto fRtc = 500_Hz;
    inline static constexpr auto fPwm = 1000_Hz;
    inline static constexpr auto dt = 2000_us;
    inline static constexpr auto intervall = 100_ms;
    inline static constexpr uint8_t ticksPerSecond = 1000_ms / intervall;
};

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 2>;

#ifdef USE_HOTT
//using sensor = Hott::Experimental::Sensor<usart3Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg_1, Hott::VarTextMsg<Parameter::menuLines>, systemTimer>;
using sensor = Hott::Experimental::Sensor<usart3Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;
using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<1, 2>>;
#endif


using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
using pwm = PWM::DynamicPwm<tcaPosition>;

#ifdef USE_TERMINAL
using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart3Position, tcaPosition>>;
#else
using portmux = Portmux::StaticMapper<Meta::List<usart3Position, tcaPosition>>;
#endif
//using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart3Position>>;

#ifdef USE_HOTT
class RCMenu final : public Hott::Menu<8> {
//class RCMenu final : public Hott::Menu<Parameter::menuLines> {
public:
    RCMenu() : Menu(this, "Switch 1.0"_pgm, &mTimeOut, &mSoftStart) {}
private:
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mTimeOut{"Timeout"_pgm, appData, Storage::AVKey::OnDelay, 3};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mSoftStart{"Softstart"_pgm, appData, Storage::AVKey::SoftStart, 1};
};

template<typename PA, typename TopMenu>
class HottMenu final {
    HottMenu() = delete;
public:
    inline static void init() {
        clear();
    }
    inline static void periodic() {
        if (auto k = PA::key(); k != Hott::key_t::nokey) {
            processKey(k);
        }
        mMenu->textTo(PA::text());
    }
    inline static void processKey(Hott::key_t key) {
        assert(mMenu);
        if (auto m = mMenu->processKey(key); m != mMenu) {
            mMenu = m;
            clear();
        }
    }
private:
    inline static void clear() {
        for(auto& line : PA::text()) {
            line.clear();
        }
    }
    inline static TopMenu mTopMenu;
    inline static Hott::Menu<PA::menuLines>* mMenu = &mTopMenu;
};
#endif
#ifdef USE_HOTT
using menu = HottMenu<sensor, RCMenu>;
#endif

using sleep = Sleep<>;

template<typename PwmDev, typename P>
struct Beeper {
    enum class Mode : uint8_t {Off, Periodic, OneShot};
    enum class Tone : uint8_t {Tone, ScaleUp, ScaleDown, Stakkato};
    
    inline static constexpr auto fTone     = 4200_Hz;
    inline static constexpr auto fToneHigh = 4300_Hz;
    inline static constexpr auto fToneLow  = 4100_Hz;
    
    inline static constexpr auto formFactor = 2;
    
    inline static constexpr auto beepDuration  = 200_ms;
    inline static constexpr auto beepTicks  = beepDuration / P::intervall;
    
    inline static constexpr auto stakkatoDuration  = 100_ms;
    inline static constexpr auto stakkatoTicks  = stakkatoDuration / P::intervall;
    
    using value_type = typename etl::typeForValue_t<beepTicks>;
    
    inline static constexpr auto beepIntervall = 1000_ms;
    inline static constexpr auto beepIntervallTicks  = beepIntervall/ P::intervall;
    static_assert(beepIntervallTicks < std::numeric_limits<value_type>::max());
    
    inline static constexpr auto numberOfPhases = beepIntervallTicks / beepTicks;
    
    //    static_print<beepIntervallTicks> x;
    
 
    inline static void init() {
        PwmDev::init();
        PwmDev::frequency(Parameter::fPwm);
        PwmDev::template on<Meta::List<PWM::WO<0>>>();
        PwmDev::template duty<PWM::WO<0>>(1000);
        normal();
    }
    
    inline static void tick() {
        ++mTicks;
        uint8_t phase = mTicks.toInt() / beepTicks;
        if (mMode != Mode::Off) {
            if (mTone == Tone::Tone) {
                if (phase == 0) {
                    PwmDev::template on<Meta::List<AVR::PWM::WO<0>>>();
//                    buzzerPins::dir<AVR::Output>();
                }
                else {
                    PwmDev::template off<Meta::List<AVR::PWM::WO<0>>>();
//                    buzzerPins::dir<AVR::Input>();
                    if (mMode == Mode::OneShot) {
                        mMode = Mode::Off;
                    }
                }
            }
            else if (mTone == Tone::ScaleUp) {
                if (phase == 0) {
                    low();
                    PwmDev::template on<Meta::List<AVR::PWM::WO<0>>>();
//                    buzzerPins::dir<AVR::Output>();
                }
                else if (phase == 1) {
                    normal();
                }
                else if (phase == 2) {
                    high();
                }
                else {
                    PwmDev::template off<Meta::List<AVR::PWM::WO<0>>>();
//                    buzzerPins::dir<AVR::Input>();
                }
            }
            else if (mTone == Tone::ScaleDown) {
                if (phase == 0) {
                    high();
//                    PwmDev::template on<AVR::PWM::WO<0>>();
//                    buzzerPins::dir<AVR::Output>();
                }
                else if (phase == 1) {
                    normal();
                }
                else if (phase == 2) {
                    low();
                }
                else {
//                    PwmDev::template off<AVR::PWM::WO<0>>();
//                    buzzerPins::dir<AVR::Input>();
                }
            }
            else if (mTone == Tone::Stakkato) {
                uint8_t sphase = mTicks.toInt() / stakkatoTicks;
                if ((sphase < (2 * mStakkatos)) && ((sphase % 2) == 0)) {
//                    PwmDev::template on<AVR::PWM::WO<0>>();
//                    buzzerPins::dir<AVR::Output>();
                }
                else {
//                    PwmDev::template off<AVR::PWM::WO<0>>();
//                    buzzerPins::dir<AVR::Input>();
                }
            }
        }
        else {
//            PwmDev::template off<AVR::PWM::WO<0>>();
//            buzzerPins::dir<AVR::Input>();
        }
    }
    inline static void low() {
        PwmDev::frequency(fToneLow);
    }            
    inline static void high() {
        PwmDev::frequency(fToneHigh);
    }            
    inline static void normal() {
        PwmDev::frequency(fTone);
    }            
    inline static void on() {
        mTicks = 0;
        mMode = Mode::Periodic;
    }
    inline static void oneShot() {
        mTicks = 0;
        mMode = Mode::OneShot;
    }
    inline static void off() {
        mTicks = 0;
        mMode = Mode::Off;
    }
    inline static void mode(Mode m) {
        mTicks = 0;
        mMode = m;
    }
    inline static void tone(Tone t) {
        mTone = t;
    }
    inline static void stakkatos(uint8_t n) {
        mTicks = 0;
        mStakkatos = n; 
    }
private:
    inline static Mode mMode = Mode::Off;
    inline static Tone mTone = Tone::Tone;
    inline static etl::uint_ranged_circular<value_type, 0, beepIntervallTicks> mTicks;
    inline static uint8_t mStakkatos = 1;
};

using beeper = Beeper<pwm, Parameter>;


template<typename P, typename Beeper>
struct FSM {
    inline static constexpr auto startPressDuration = 1000_ms;
    inline static constexpr auto  startPressTicks = startPressDuration / P::intervall;
    
    inline static constexpr auto sleepStartDuration = 3000_ms;
    inline static constexpr auto sleepStartTicks = sleepStartDuration / P::intervall;
    
    using value_type = typename etl::typeForValue_t<sleepStartTicks>;
    
    enum class State : uint8_t {PowerOn, 
                                PreConfig, Config, 
                                OnDelay,
                                Warn, WarnOff, Off, WaitForOn, PreOn, On, WaitForOff, PreOff};
    
    inline static void tick() {
        State newState = state;
        switch(state) {
        case State::PowerOn:
            if (buttonPin::isActive()) {
                newState = State::PreConfig;
            }
            else {
                pressTicks = 0;
                if (++stateTicks >= beeper::beepIntervallTicks) {
                    newState = State::Warn;
                }
            }
            break;
        case State::PreConfig:
            if (buttonPin::isActive()) {
                if (++pressTicks > startPressTicks) {
                    newState = State::Config;
                }
            }
            else {
                newState = State::Warn;
            }
            break;
        case State::Config:
            if (buttonPin::isActive()) {
                stateTicks = 0;
                if (++pressTicks >= (3 * startPressTicks)) {
                    pressTicks = 0;

                    Storage::configValue += 1;
                    
                    if (Storage::configValue > 5) {
                        Storage::configValue = 1;
                    }
                    if (Storage::configValue < 1) {
                        Storage::configValue = 1;
                    } 
                    
                    if (Storage::configValue <= 3) {
                        beeper::low();
                        beeper::stakkatos(Storage::configValue);
                    }
                    else {
                        beeper::high();
                        beeper::stakkatos(Storage::configValue - 3);
                    }
                }
            }
            else {
                pressTicks = 0;
                if (stateTicks < std::numeric_limits<value_type>::max()) {
                    if (++stateTicks > (5 * startPressTicks)) {
                        if (Storage::configValue <= 3) {
                            appData[Storage::AVKey::OnDelay] = Storage::configValue;
                            appData.change();
                        }
                        else {
                            appData[Storage::AVKey::SoftStart] = Storage::configValue - 3;
                            appData.change();
                        }
                        beeper::tone(beeper::Tone::ScaleDown);
                    }
                    if (stateTicks > (7 * startPressTicks)) {
                        beeper::off();
                        stateTicks = std::numeric_limits<value_type>::max();
                    }
                }
            }
            break;
        case State::Warn:
            if (++stateTicks >= beeper::beepIntervallTicks) {
                newState = State::WarnOff;
            }
            break;
        case State::WarnOff:
            if (++stateTicks >= beeper::beepIntervallTicks) {
                newState = State::Off;
            }
            break;
        case State::Off:
            if (buttonPin::isActive()) {
                if (!buttonStillPressed) {
                    if (++pressTicks > startPressTicks) {
                        newState = State::WaitForOn;
                    }
                }
            }
            else {
                buttonStillPressed = false;
                pressTicks = 0;
                ++sleepTicks;
                if (sleepTicks > sleepStartTicks) {
//                    sleep::down();
                    sleepTicks = 0;
                }
            }
            break;
        case State::WaitForOn:
            if (buttonPin::isActive()) {
                if (++pressTicks > (3 * startPressTicks)) {
                    newState = State::PreOn;
                }
            }
            else {
                newState = State::Off;
            }
            break;
        case State::PreOn:
            if (buttonPin::isActive()) {
                if (++pressTicks > startPressTicks) {
                    newState = State::OnDelay;
                    buttonStillPressed = true;
                }
            }
            else {
                buttonStillPressed = false;
                newState = State::Off;
            }
            break;
        case State::OnDelay:
        {
            auto n = appData[Storage::AVKey::OnDelay];
            if (n) {
                if (++stateTicks > (n.toInt() * Parameter::ticksPerSecond)) {
                    newState = State::On;
                } 
            }
            else {
                assert(false);
            }
        }
            break;
        case State::On:
            if (buttonPin::isActive()) {
                if (!buttonStillPressed) {
                    if (++pressTicks > startPressTicks) {
                        newState = State::WaitForOff;
                    }
                }
            }
            else {
                buttonStillPressed = false;
                pressTicks = 0;
            }
            break;
        case State::WaitForOff:
            if (buttonPin::isActive()) {
                if (++pressTicks > (3 * startPressTicks)) {
                    newState = State::PreOff;
                }
            }
            else {
                newState = State::On;
            }
            break;
        case State::PreOff:
            if (buttonPin::isActive()) {
                if (++pressTicks > startPressTicks) {
                    newState = State::Off;
                    buttonStillPressed = true;
                }
            }
            else {
                buttonStillPressed = false;
                newState = State::On;
            }
            break;
        default:
            assert(false);
            break;
        }
        if (newState != state) {
            pressTicks = 0;
            stateTicks = 0;            
            switch(state = newState) {
            case State::PowerOn:
                mosfetPin::inactivate();
                break;
            case State::PreConfig:
                break;
            case State::Config:
                Storage::configValue = 1;
                beeper::low();
                beeper::tone(beeper::Tone::Stakkato);
                beeper::stakkatos(Storage::configValue);
                beeper::on();
                break;
            case State::Warn:
                beeper::tone(beeper::Tone::ScaleUp);
                beeper::on();
                break;
            case State::WarnOff:
                beeper::off();
                beeper::tone(beeper::Tone::Tone);
                break;
            case State::Off:
                Beeper::low();
                Beeper::oneShot();
                mosfetPin::inactivate();
                break;
            case State::WaitForOn:
            case State::WaitForOff:
                Beeper::normal();
                Beeper::on();
                break;
            case State::PreOn:
                Beeper::high();
                break;
            case State::OnDelay:
                Beeper::high();
                Beeper::oneShot();
                break;
            case State::On:
                if (mosfetPin::activated()) {
                    beeper::high();
                    Beeper::oneShot();
                }
                softStart();
                break;
            case State::PreOff:
                Beeper::low();
                break;
            default:
                assert(false);
                break;
            }
        }
    }
    inline static void softStart() {
        if (!mosfetPin::activated()) {
            if (appData[Storage::AVKey::SoftStart].toInt() > 1) {
                for(uint8_t counter = 0; counter < 100; ++counter) {
                    mosfetPin::activate();        
                    Util::delay(5_us); // 20us on
                    mosfetPin::inactivate();        
                    Util::delay(200_us); // 200us off -> 1/11 PWM
                }
            }
            mosfetPin::activate();        
        }
    }
    inline static bool buttonStillPressed = false;
    inline static State state = State::PowerOn;
    inline static value_type pressTicks = 0;
    inline static value_type sleepTicks = 0;
    inline static value_type stateTicks = 0;
};

using fsm = FSM<Parameter, beeper>;

struct Button : public IsrBaseHandler<AVR::ISR::Port<D>> {
    inline static void isr() {
        button::resetInt();
    }
};

using isrRegistrar = IsrRegistrar<Button>;

int main() {
    portmux::init();
    
    sleep::init<sleep::PowerDown>();

    led::template dir<Output>();
    led::off();
    
    fet::template dir<Output>();
    fet::off();
    
    button::template dir<Input>();
    button::template attributes<Meta::List<Attributes::Interrupt<Attributes::BothEdges>>>(); // sonst keine Aufwecken.
    
    pf2::template dir<Output>();
    
    eeprom::init();

    ccp::unlock([]{
        clock::prescale<1>();
    });

    systemTimer::init();
#ifdef USE_TERMINAL
    terminalDevice::init<BaudRate<9600>>();
#endif
#ifdef USE_HOTT
    sensor::init();
    menu::init();
    adcController::init();
#endif
    
    beeper::init();
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);
    const auto dbgTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    bool eepSave = false;
    
    uint8_t counter = 0;
    
#ifdef USE_HOTT
    sensor::data().current = 100;
    sensor::data().current_max = 100;
#endif
    mosfetPin::activate();
    
    sei();
    
    while(true) {
#ifdef USE_HOTT
        sensor::periodic();
        menu::periodic();
        adcController::periodic();
#endif
#ifdef USE_TERMINAL
        terminalDevice::periodic();
#endif
        
        eepSave |= eeprom::saveIfNeeded();
        
        
        systemTimer::periodic([&]{
#ifdef USE_HOTT
            sensor::ratePeriodic();
#endif
            pf2::toggle();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    beeper::tick();
                    fsm::tick();
                    led::toggle();
                }
                if (dbgTimer == t) {
#ifdef USE_TERMINAL
//                    etl::outl<terminal>("test90: "_pgm, ++counter, " ain1: "_pgm, adcController::value(0).toInt(), " ain2: "_pgm, adcController::value(1).toInt());
#endif 
                    if (counter > 100) {
                        led::on();
                        mosfetPin::inactivate();
                        sleep::down();
                        mosfetPin::activate();
                        counter = 0;
                    }
                }
            });
            appData.expire();
        });
    }
}

ISR(PORTD_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<D>>();
}
