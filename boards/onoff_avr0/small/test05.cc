// todo:

// ibus protokoll
// calibrierung über eeprom (offset / steigung) für spannung / strom
// hot-t-sensor-type (eeprom)

#define NDEBUG

#define USE_HOTT
//#define USE_IBUS

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
#include <external/ibus/ibus.h>
#include <external/units/music.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>

#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace Parameter {
    constexpr uint8_t menuLines = 8;
#ifdef USE_IBUS
    constexpr auto fRtc = 2000_Hz;
#endif
#ifdef USE_HOTT
    constexpr auto fRtc = 500_Hz;
#endif
}

namespace Storage {
    enum class AVKey : uint8_t {TimeOut = 0, SoftStart, SensorType, Magic0, Magic1, _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        using value_type = etl::uint_NaN<uint8_t>;
        inline value_type& operator[](const AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
        inline void select(const AVKey) {}
    private:
        std::array<value_type, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
}

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

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<1, 5, 0x1e>>; // 1e = temp

#ifdef USE_IBUS
template<typename ADC, uint8_t Channel>
struct VoltageProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
//    index_t::_;
    inline static constexpr auto ibus_type = IBus::Type::type::EXTERNAL_VOLTAGE;
    inline static constexpr void init() {}
    
    using battVoltageConverter = Hott::Units::Converter<adc, IBus::battery_voltage_t, std::ratio<121,21>>; 
    
    inline static constexpr uint16_t value() {
        return battVoltageConverter::convert(ADC::value(channel)).value;
    }
};

using voltageP = VoltageProvider<adcController, 0>;

template<typename ADC, uint8_t Channel>
struct CurrentProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::BAT_CURR;
    using currentConverter = Hott::Units::Converter<adc, IBus::current_t, std::ratio<11117,1000>>; // todo: richtiger scale faktor
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return currentConverter::convert(ADC::value(channel)).value;
    }
};

using currentP = CurrentProvider<adcController, 1>;

template<typename ADC, uint8_t Channel>
struct TempProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        return sigrow::adcValueToTemperature<std::ratio<1,10>, 40>(ADC::value(channel)).value;
    }
};
using tempP = TempProvider<adcController, 2>;

using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, Meta::List<voltageP, currentP, tempP>, systemTimer>;
#endif

#ifdef USE_HOTT
using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;
#endif

#if !(defined(USE_IBUS) || defined(USE_HOTT))
using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;
#endif

using battVoltageConverter = Hott::Units::Converter<adc, Hott::Units::battery_voltage_t, std::ratio<121,21>>; // todo: richtiger scale faktor
using currentConverter = Hott::Units::Converter<adc, Hott::Units::current_t, std::ratio<11117,1000>>; // todo: richtiger scale faktor

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

#ifdef USE_HOTT
class RCMenu final : public Hott::Menu<Parameter::menuLines, true, 4> {
public:
    RCMenu() : Menu(this, "OnOff 1.0"_pgm, &mSoft, &mTimeout, &mType) {}
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
        for(auto& line : PA::text()) {
            line.clear();
        }
    }
    inline static TopMenu mTopMenu;
    inline static Hott::IMenu<PA::menuLines>* mMenu = &mTopMenu;
};

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
    
    constexpr note c_ii_0 {Pitch{Letter::c}, Length{Base::doublenote}};
    
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
    constexpr note d_iii_4 {Pitch{Letter::d, Octave::iii}, Length{Base::quarter}};
    constexpr note d_s_iii_4 {Pitch{Letter::d, Octave::iii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note e_iii_4 {Pitch{Letter::e, Octave::iii}, Length{Base::quarter}};
    constexpr note f_iii_4 {Pitch{Letter::f, Octave::iii}, Length{Base::quarter}};
    constexpr note f_s_iii_4 {Pitch{Letter::f, Octave::iii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note g_iii_4 {Pitch{Letter::g, Octave::iii}, Length{Base::quarter}};
    constexpr note g_s_iii_4 {Pitch{Letter::g, Octave::iii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note a_iii_4 {Pitch{Letter::a, Octave::iii}, Length{Base::quarter}};
    constexpr note a_iii_1 {Pitch{Letter::a, Octave::iii}, Length{Base::whole}};
    constexpr note h_iii_4 {Pitch{Letter::h, Octave::iii}, Length{Base::quarter}};
    
    constexpr note pause_4 {Pitch{Letter::pause}, Length{Base::quarter}};
    constexpr note pause_1 {Pitch{Letter::pause}, Length{Base::whole}};
    
    constexpr auto startMelody = AVR::Pgm::Array<note, c_ii_1, d_ii_1, e_ii_1, f_ii_1, g_ii_1, a_ii_1, h_ii_1, c_iii_1>{}; // C-Dur up
    constexpr auto sleepMelody = AVR::Pgm::Array<note, c_iii_1, g_ii_1, e_ii_1, c_ii_1>{}; // C-Dur 3-tone down
    
    constexpr auto waitOnMelody = AVR::Pgm::Array<note, c_ii_1, f_s_ii_1>{}; // Tritonus
    constexpr auto waitOffMelody = AVR::Pgm::Array<note, c_iii_1, h_ii_1, c_iii_1, a_ii_1, c_iii_1, g_ii_1, c_iii_1, f_ii_1, c_iii_1, e_ii_1, c_iii_1, d_ii_1, c_iii_1, c_ii_1>{};
    
    constexpr auto onMelody = AVR::Pgm::Array<note, pause_4, c_ii_4, pause_4, c_ii_4, pause_4, c_ii_2, e_ii_2, g_ii_2, c_iii_1>{};
    constexpr auto offMelody = AVR::Pgm::Array<note, c_ii_0, pause_1, c_ii_0, pause_1, c_ii_0>{};
    
}

template<typename Timer>
struct FSM {
    enum class State : uint8_t {Init, Startup, Idle, WaitSleep, Sleep, WaitOn, WaitOnInterrupted, On, FetOn, WaitOff, WaitOffInterrupted};
    
    static constexpr auto intervall = Timer::intervall;
    
    static constexpr External::Tick<Timer> idleTimeBeforeSleepTicks{10000_ms};
    
    //    std::integral_constant<uint16_t, idleTimeBeforeSleepTicks.value>::_;

    static constexpr External::Tick<Timer> softStartTicks{100_ms};
    
    inline static External::Tick<Timer> onDelay;
    
    inline static void init() {
        load();
    }    

    inline static void load() {
        if (appData[Storage::AVKey::TimeOut]) {
            onDelay = 1000_ms * appData[Storage::AVKey::TimeOut].toInt();
        }
    }    
    
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
                mState = State::WaitOnInterrupted;
            }
            break;
        case State::WaitOnInterrupted:
            mState = State::Idle;
            break;
        case State::On:
            if (stateTicks > onDelay) {
                mState = State::FetOn;
            }
            if (button::event() == button::Press::Short) {
                mState = State::WaitOff;
            }
            break;
        case State::FetOn:
            if (button::event() == button::Press::Short) {
                mState = State::WaitOff;
            }
            break;
        case State::WaitOff:
            if (auto event = button::event(); event == button::Press::Long) {
                mState = State::Idle;
            }
            else if (event == button::Press::Release) {
                mState = State::WaitOffInterrupted;
            }
            break;
        case State::WaitOffInterrupted:
            mState = State::On;
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
                toneGenerator::play(waitOnMelody, true);
                led::blink(led::count_type{2});
                break;
            case State::WaitOnInterrupted:
                toneGenerator::play(h_iii_4);
                break;                
            case State::WaitOffInterrupted:
                toneGenerator::play(h_iii_4);
                break;                
            case State::On:
                if (lastState != State::WaitOffInterrupted) {
                    toneGenerator::play(onMelody);
                }
                led::blink(led::count_type{4});
                break;
            case State::FetOn:
                if (!toneGenerator::busy()) {
                    toneGenerator::play(a_iii_1);
                }
                led::blink(led::count_type{5});
                fet::activate();
                break;
            case State::WaitOff:
                led::blink(led::count_type{3});
                toneGenerator::play(waitOffMelody, true);
                break;
            case State::Idle:
                if (lastState != State::WaitOnInterrupted) {
                    toneGenerator::play(offMelody);
                }
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
    static inline void tick() {
        c += ((uint32_t)last_current.value * 2); // 200ms
    }
private:
#ifdef USE_HOTT
    inline static auto sensorData = Hott::Experimental::Adapter<sensor::binaryMessage_type>(sensor::data());
#endif
    inline static Hott::Units::battery_voltage_t voltage_min{std::numeric_limits<Hott::Units::battery_voltage_t::value_type>::max()};
    inline static Hott::Units::battery_voltage_t last_voltage;
    inline static Hott::Units::current_t current_max;
    inline static Hott::Units::current_t last_current;
    inline static External::Units::celsius<uint16_t, std::ratio<1, 1>> temp_max;

    inline static uint32_t c = 0;
    
    static inline void part0() {
        auto v = battVoltageConverter::convert(adcController::value(adcController::index_type{0}));
        last_voltage = v;
#ifdef USE_HOTT
        sensorData.voltage(v);
        if (v > Hott::Units::battery_voltage_t{20}) {
            voltage_min = std::min(voltage_min, v);
            sensorData.voltageMin(voltage_min);
        }
#endif
    }
    static inline void part1() {
        auto c = currentConverter::convert(adcController::value(adcController::index_type{1}));
        last_current = c;
#ifdef USE_HOTT
        sensorData.current(c);
        current_max = std::max(current_max, c);
        sensorData.currentMax(current_max);
#endif        
    }
    static inline void part2() {
        auto t = sigrow::adcValueToTemperature(adcController::value(adcController::index_type{2}));
#ifdef USE_HOTT
        sensorData.temp(t);
        temp_max = std::max(temp_max, t);
        sensorData.tempMax(temp_max);
#endif
    }
    static inline void part3() {
#ifdef USE_HOTT
        sensorData.capRaw(c / 3600);
#endif
    }
};

using measurements = Measurements;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    fet::init();
    eeprom::init();
    
    [[maybe_unused]] bool changed = false;
    
    {
        constexpr auto m0 = Storage::ApplData::value_type{43};
        constexpr auto m1 = Storage::ApplData::value_type{44};
        if (!((appData[Storage::AVKey::Magic0] == m0) && (appData[Storage::AVKey::Magic1] == m1))) {
            appData[Storage::AVKey::Magic0] = m0;
            appData[Storage::AVKey::Magic1] = m1;
            appData.change();
            changed  = true;
        }
    }
    
    buttonPin::attributes<Meta::List<Attributes::Interrupt<Attributes::BothEdges>>>();
    button::init();
    
    led::init();
    portmux::init();
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
#elif defined(USE_IBUS)
    ibus::periodic();
#else
    terminalDevice::init<AVR::BaudRate<9600>>();
#endif
    
    sleep::template init<sleep::PowerDown>();
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    const auto capTimer = alarmTimer::create(200_ms, External::Hal::AlarmFlags::Periodic);
    
    {
        buttonPin::resetInt();
        etl::Scoped<etl::EnableInterrupt<>> ei;
#if !(defined(USE_IBUS) || defined(USE_HOTT))
        etl::outl<terminal>("*"_pgm);
#endif
        while(true) {
#ifdef USE_HOTT
            sensor::periodic();
            menu::periodic();
#endif
#ifdef USE_IBUS
            ibus::periodic();
#endif
#if !(defined(USE_IBUS) || defined(USE_HOTT))
            terminalDevice::periodic();
#endif
            bool eepSave{};
            eeprom::saveIfNeeded([&]{
                fsm::load();                
                eepSave = true;
            });
            
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
#if !(defined(USE_IBUS) || defined(USE_HOTT))
                        if (eepSave) {
                            etl::outl<terminal>("es"_pgm);
                            eepSave = false;
                        }
                        if (changed) {
                            etl::outl<terminal>("ni"_pgm);
                        }
                        etl::outl<terminal>("test05 v: "_pgm, adcController::value(adcController::index_type{0}).toInt(), 
                                            " c: "_pgm, adcController::value(adcController::index_type{1}).toInt(),
                                            " t: "_pgm, adcController::value(adcController::index_type{2}).toInt(),
                                            " k: "_pgm, sigrow::adcValueToTemperature(adcController::value(adcController::index_type{2})).value
                                            );
#endif
                    }
                    if (capTimer == t) {
                        measurements::tick();
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
#if !(defined(USE_IBUS) || defined(USE_HOTT))
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
    while(true) {
        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
