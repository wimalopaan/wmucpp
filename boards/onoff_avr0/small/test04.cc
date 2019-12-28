// todo:

// ibus protokoll
// calibrierung über eeprom (offset / steigung) für spannung / strom
// einschaltverzögerung (nach on) (eeprom)
// hot-t-sensor-type (eeprom)

//#define NDEBUG

#define USE_HOTT

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/sleep.h>
#include <mcu/internals/sigrow.h>
#include <mcu/pgm/pgmarray.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>
#include <external/units/music.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/tick.h>

#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace Parameter {
    constexpr uint8_t menuLines = 8;
    constexpr auto fRtc = 500_Hz;
}

struct Storage final {
    Storage() = delete;
    enum class AVKey : uint8_t {TimeOut = 0, SoftStart, SensorType,
                                _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        etl::uint_NaN<uint8_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<etl::uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

// PA1: Voltage = AIN1
// PA2: Debug Pin
// PA3: Led
// PA4:
// PA5: current sense = AIN5
// PA6:
// PA7: Taster
// PB3:
// PB2: TXRX (Sensor)
// PB1: VN7003 = fet
// PB0: Buzzer = tca:wo0

using PortA = Port<A>;
using PortB = Port<B>;

#ifndef NDEBUG
using dbg1      = Pin<PortA, 2>; 
#endif
using ledPin    = ActiveHigh<Pin<PortA, 3>, Output>;
using led       = External::Blinker<ledPin, systemTimer::intervall, 30_ms, 1000_ms>;
using buttonPin = Pin<PortA, 7>;
using button    = External::Button<ActiveLow<buttonPin, Input>, systemTimer, External::Tick<systemTimer>{100_ms}, External::Tick<systemTimer>{3000_ms}>;
using fet       = ActiveHigh<Pin<PortB, 1>, Output>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using sleep = Sleep<>;

using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;
#ifndef USE_HOTT
using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;
#endif

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<1, 5, 0x1e>>; // 1e = temp
using battVoltageConverter = Hott::Units::Converter<adc, Hott::Units::battery_voltage_t, std::ratio<121,21>>; // todo: richtiger scale faktor
using currentConverter = Hott::Units::Converter<adc, Hott::Units::current_t, std::ratio<11117,1000>>; // todo: richtiger scale faktor

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

class RCMenu final : public Hott::Menu<Parameter::menuLines> {
public:
    RCMenu() : Menu(this, "OnOff 1.0"_pgm, &mSoft, &mTimeout, &mType) {}
private:
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mSoft{"Start"_pgm, appData, Storage::AVKey::SoftStart, 2};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mTimeout{"TimeOut"_pgm, appData, Storage::AVKey::TimeOut, 2};
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
        for(auto& line : PA::text()) {
            line.clear();
        }
    }
    inline static TopMenu mTopMenu;
    inline static Hott::Menu<PA::menuLines>* mMenu = &mTopMenu;
};

#ifdef USE_HOTT
using menu = HottMenu<sensor, RCMenu>;
#endif

template<typename PWM, typename Output>
struct PwmWrapper {
    inline static constexpr auto f_timer = PWM::f_timer;
    inline static void init() {
        PWM::init();
    }
    inline static void off() {
        PWM::template off<Meta::List<Output>>();
    }
    inline static void on() {
        PWM::template on<Meta::List<Output>>();
    }
    inline static void frequency(uint16_t f) {
        PWM::frequency(f);
    }
    inline static void duty(uint16_t d) {
        PWM::template duty<Meta::List<Output>>(d);
    }
};

using tonePwm = PWM::DynamicPwm<tcaPosition>;
using toneWrapper = PwmWrapper<tonePwm, AVR::PWM::WO<0>>;
using toneGenerator = External::Music::Generator<toneWrapper>;

namespace {
    using namespace External::Music;
    
    using note = Note<toneWrapper, std::integral_constant<uint16_t, 40>>;
    
    constexpr note c_ii_1 {Pitch{Letter::c}, Length{Base::whole}};
    constexpr note c_s_ii_1 {Pitch{Letter::c, Octave::ii, Accidential::sharp}, Length{Base::whole}};
    constexpr note d_ii_1 {Pitch{Letter::d}, Length{Base::whole}};
    constexpr note d_s_ii_1 {Pitch{Letter::d, Octave::ii, Accidential::sharp}, Length{Base::whole}};
    constexpr note e_ii_1 {Pitch{Letter::e}, Length{Base::whole}};
    constexpr note f_ii_1 {Pitch{Letter::f}, Length{Base::whole}};
    constexpr note f_s_ii_1 {Pitch{Letter::f, Octave::ii, Accidential::sharp}, Length{Base::whole}};
    constexpr note g_ii_1 {Pitch{Letter::g}, Length{Base::whole}};
    constexpr note g_s_ii_1 {Pitch{Letter::g, Octave::ii, Accidential::sharp}, Length{Base::whole}};
    constexpr note a_ii_1 {Pitch{Letter::a}, Length{Base::whole}};
    constexpr note h_ii_1 {Pitch{Letter::h}, Length{Base::whole}};
    
    constexpr note c_ii_2 {Pitch{Letter::c}, Length{Base::half}};
    constexpr note e_ii_2 {Pitch{Letter::e}, Length{Base::half}};
    constexpr note g_ii_2 {Pitch{Letter::g}, Length{Base::half}};
    
    constexpr note c_ii_4 {Pitch{Letter::c}, Length{Base::quarter}};
    constexpr note c_s_ii_4 {Pitch{Letter::c, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note d_ii_4 {Pitch{Letter::d}, Length{Base::quarter}};
    constexpr note d_s_ii_4 {Pitch{Letter::d, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note e_ii_4 {Pitch{Letter::e}, Length{Base::quarter}};
    constexpr note f_ii_4 {Pitch{Letter::f}, Length{Base::quarter}};
    constexpr note f_s_ii_4 {Pitch{Letter::f, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note g_ii_4 {Pitch{Letter::g}, Length{Base::quarter}};
    constexpr note g_s_ii_4 {Pitch{Letter::g, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note a_ii_4 {Pitch{Letter::a}, Length{Base::quarter}};
    constexpr note h_ii_4 {Pitch{Letter::h}, Length{Base::quarter}};
    
    constexpr note c_iii_1 {Pitch{Letter::c, Octave::iii}, Length{Base::whole}};
    
    constexpr note c_iii_4 {Pitch{Letter::c, Octave::iii}, Length{Base::quarter}};
    constexpr note c_s_iii_4 {Pitch{Letter::c, Octave::iii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note d_iii_4 {Pitch{Letter::d}, Length{Base::quarter}};
    constexpr note d_s_iii_4 {Pitch{Letter::d, Octave::iii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note e_iii_4 {Pitch{Letter::e}, Length{Base::quarter}};
    constexpr note f_iii_4 {Pitch{Letter::f}, Length{Base::quarter}};
    constexpr note f_s_iii_4 {Pitch{Letter::f, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note g_iii_4 {Pitch{Letter::g}, Length{Base::quarter}};
    constexpr note g_s_iii_4 {Pitch{Letter::g, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note a_iii_4 {Pitch{Letter::a}, Length{Base::quarter}};
    constexpr note h_iii_4 {Pitch{Letter::h}, Length{Base::quarter}};
    
    constexpr note pause_4 {Pitch{Letter::pause}, Length{Base::quarter}};
    
    constexpr auto startMelody = AVR::Pgm::Array<note, c_ii_1, d_ii_1, e_ii_1, f_ii_1, g_ii_1, a_ii_1, h_ii_1, c_iii_1>{}; // C-Dur up
    constexpr auto sleepMelody = AVR::Pgm::Array<note, c_iii_1, g_ii_1, e_ii_1, c_ii_1>{}; // C-Dur 3-tone down
    
    constexpr auto waitOnMelody = AVR::Pgm::Array<note, c_ii_1, f_s_ii_1>{}; // Tritonus
    constexpr auto waitOffMelody = AVR::Pgm::Array<note, c_iii_1, h_ii_1, c_iii_1, a_ii_1, c_iii_1, g_ii_1, c_iii_1, f_ii_1, c_iii_1, e_ii_1, c_iii_1, d_ii_1, c_iii_1, c_ii_1>{};
    
    constexpr auto onMelody = AVR::Pgm::Array<note, c_ii_4, pause_4, c_ii_4, pause_4, c_ii_2, e_ii_2, g_ii_2, c_iii_1>{};
    
}

template<typename Timer>
struct FSM {
    enum class State : uint8_t {Init, Startup, Idle, WaitSleep, Sleep, WaitOn, On, FetOn, WaitOff};
    
    static constexpr auto intervall = Timer::intervall;
    
    static constexpr External::Tick<Timer> idleTimeBeforeSleepTicks{10000_ms};
    //    std::integral_constant<uint16_t, idleTimeBeforeSleepTicks.value>::_;
    
    inline static void init() {}    
    
    inline static void periodic() {
        const State lastState = mState;
        ++stateTicks;
        switch (mState) {
        case State::Init:
            fet::inactivate();
            toneGenerator::play(startMelody);
            mState = State::Startup;
            break;
        case State::Startup:
            if (!toneGenerator::busy()) {
                mState = State::Idle;
            }
            break;
        case State::Idle:
            if (stateTicks > idleTimeBeforeSleepTicks) {
                mState = State::WaitSleep;
            }
            else {
                if (button::event() == button::Press::Short) {
                    mState = State::WaitOn;
                }
            }
            break;
        case State::WaitSleep:
            if (!toneGenerator::busy()) {
                mState = State::Sleep;
            }
            break;
        case State::Sleep:
            if (button::event() == button::Press::Short) {
                mState = State::WaitOn;
            }
            break;
        case State::WaitOn:
            if (auto event = button::event(); event == button::Press::Long) {
                mState = State::On;
            }
            else if (event == button::Press::Release) {
                mState = State::Idle;
            }
            break;
        case State::On:
            if (button::event() == button::Press::Short) {
                mState = State::WaitOff;
            }
            break;
        case State::FetOn:
            break;
        case State::WaitOff:
            if (auto event = button::event(); event == button::Press::Long) {
                mState = State::Idle;
            }
            else if (event == button::Press::Release) {
                mState = State::On;
            }
            break;
            //        default:
            //            break;
        }
        if (lastState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Init:
                break;
            case State::Startup:
                fet::inactivate();
                break;
            case State::WaitSleep:
                led::off();
                toneGenerator::play(sleepMelody);
                break;
            case State::Sleep:
                fet::inactivate();
                led::off();
                sleep::down();
                mState = State::Idle;
                break;
            case State::WaitOn:
                led::blink(led::count_type{2});
                toneGenerator::play(waitOnMelody, true);
                break;
            case State::On:
                toneGenerator::off();
                led::blink(led::count_type{5});
                toneGenerator::play(onMelody, true);
                fet::activate();
                break;
            case State::FetOn:
                break;
            case State::WaitOff:
                led::blink(led::count_type{3});
                toneGenerator::play(waitOffMelody, true);
                break;
            case State::Idle:
                toneGenerator::off();
                led::blink(led::count_type{1});
                fet::inactivate();
                break;
            }
        }
    }
private:
    inline static External::Tick<Timer> stateTicks;
    inline static State mState{State::Init};    
};

using fsm = FSM<systemTimer>;

struct Measurements {
    static inline void periodic() {
        etl::circular_call(part0, part1, part2, part3);
    }
private:
    inline static auto sensorData = Hott::Experimental::Adapter<sensor::binaryMessage_type>(sensor::data());

    inline static Hott::Units::battery_voltage_t voltage_min;
    inline static Hott::Units::current_t current_max;
    inline static External::Units::celsius<uint16_t, std::ratio<1, 1>> temp_max;

    static inline void part0() {
        auto v = battVoltageConverter::convert(adcController::value(adcController::index_type{0}));
        sensorData.voltage(v);
        voltage_min = std::min(voltage_min, v);
        sensorData.voltageMin(voltage_min);
        
    }
    static inline void part1() {
        auto c = currentConverter::convert(adcController::value(adcController::index_type{1}));
        sensorData.current(c);
        current_max = std::max(current_max, c);
        sensorData.currentMax(current_max);
        
    }
    static inline void part2() {
        auto t = sigrow::adcValueToTemperature(adcController::value(adcController::index_type{2}));
        sensorData.temp(t);
        temp_max = std::max(temp_max, t);
        sensorData.tempMax(temp_max);
    }
    static inline void part3() {
        
    }
};

using measurements = Measurements;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    fet::init();
    
    buttonPin::attributes<Meta::List<Attributes::Interrupt<Attributes::BothEdges>>>();
    button::init();
    
    led::init();
    portmux::init();
    eeprom::init();
    systemTimer::init();
    adcController::init();
    
    toneGenerator::init();    
    fsm::init();
    
#ifndef NDEBUG
    dbg1::template dir<Output>();
#endif
    
#ifdef USE_HOTT
    sensor::init();
    menu::init();
#else
    terminalDevice::init<AVR::BaudRate<9600>>();
#endif
    
    sleep::template init<sleep::PowerDown>();
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    bool eepSave = false;
    
    {
        buttonPin::resetInt();
        etl::Scoped<etl::EnableInterrupt<>> ei;
        
        while(true) {
    #ifdef USE_HOTT
            sensor::periodic();
            menu::periodic();
    #else
            terminalDevice::periodic();
    #endif
            eepSave |= eeprom::saveIfNeeded();
            adcController::periodic();
            
            systemTimer::periodic([&]{
    #ifdef USE_HOTT
                sensor::ratePeriodic();
                measurements::periodic();
    #endif
                toneGenerator::periodic();
                fsm::periodic();
                button::periodic();
                led::periodic();
                
#ifndef NDEBUG
                dbg1::toggle();
#endif
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
#ifndef USE_HOTT
                        
                        etl::outl<terminal>("test04 v: "_pgm, adcController::value(adcController::index_type{0}).toInt(), 
                                            " c: "_pgm, adcController::value(adcController::index_type{1}).toInt(),
                                            " t: "_pgm, adcController::value(adcController::index_type{2}).toInt(),
                                            " k: "_pgm, sigrow::adcValueToTemperature(adcController::value(adcController::index_type{2})).value
                                            );
                        auto v = battVoltageConverter::convert(adcController::value(adcController::index_type{0}));
                        auto c = currentConverter::convert(adcController::value(adcController::index_type{1}));
    //                    decltype(v)::_;
                        etl::outl<terminal>("v * 10: "_pgm, v.value, " c: "_pgm, c.value);
#endif
                    }
                });
                appData.expire();
            });
        }
    }
}

ISR(PORTA_PORT_vect) {
    buttonPin::resetInt();
}

#ifndef NDEBUG

[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
//    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}

#endif
