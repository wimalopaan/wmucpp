#define NDEBUG

//#define USE_HOTT
#define USE_IBUS

#define SBUS_IBUS_NO_WARN

#include <mcu/avr.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/sleep.h>

#include <external/solutions/cells.h>
#include <external/ibus/ibus.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/tick.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using activate = Pin<Port<A>, 1>; // and led
using conn12 = Pin<Port<A>, 7>;
using daisy = Pin<Port<A>, 2>;

//using txrx = Pin<Port<A>, 6>; // used for wake-up

// AIN3 = Voltage

namespace Parameter {
#ifdef USE_IBUS
    constexpr auto fRtc = 2000_Hz;
#endif
#ifdef USE_HOTT
    constexpr auto fRtc = 1000_Hz;
#endif
    constexpr uint16_t R1vd = 100;
    constexpr uint16_t R2vd = 220;
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;
using sleep = Sleep<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V1_5>;
using adcController = External::Hal::AdcController<adc, Meta::NList<3, 0x1e>>; // 1e = temp
using channel_t = adcController::index_type;

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

template<typename ADC, uint8_t Channel>
struct CellsCollector {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};

    inline static constexpr std::byte startByte{0xa5};
    inline static constexpr uint8_t offset{1};

    template<auto N>
    static inline void copy(const etl::FixedVector<uint16_t, N>& data) {
        static_assert(N == mSize);
        if (mValues.size() < (data.size() + offset)) {
            mValues.reserve(data.size() + offset);
        }
        for(uint8_t i = 0; i < data.size(); ++i) {
            mValues[i + offset] = data[i];
        }
    }    
    static inline void set(const uint16_t v) {
        if (mValues.size() == 0) {
            mValues.reserve(offset);
        }
        mValues[0] = v;
    }
    template<auto N>
    struct Provider {
        inline static constexpr auto ibus_type = IBus::Type::type::CELL;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            return mValues[N];
        }
    };
    
    template<typename R>
    struct MinProvider {
        inline static constexpr auto ibus_type = IBus::Type::type::CELL;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            R::reset();
            auto min = mValues[0];
            for(uint8_t i = 1; i < mValues.size(); ++i) {
                if (mValues[i] > 0) {
                    if (mValues[i] < min) {
                        min = mValues[i];
                    } 
                }
            }
            return min;
        }
        inline static constexpr std::pair<uint8_t, uint16_t> indexedValue() {
            auto min = mValues[0];
            uint8_t index = 0;
            for(uint8_t i = 1; i < mValues.size(); ++i) {
                if (mValues[i] > 0) {
                    if (mValues[i] < min) {
                        min = mValues[i];
                        index = i;
                    } 
                }
            }
            return {index, min};
        }
    };
    struct TotalProvider {
        inline static constexpr auto ibus_type = IBus::Type::type::EXTERNAL_VOLTAGE;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            uint16_t t = 0;
            for(uint8_t i = 0; i < mValues.size(); ++i) {
                t += mValues[i];
            }
            return t;
        }
    };
private:
    inline static constexpr uint8_t mSize = 16;
    inline static volatile etl::FixedVector<uint16_t, mSize> mValues{};
};

using cellsColl = CellsCollector<adcController, 0>;

using cell0P = cellsColl::Provider<0>;
using cell1P = cellsColl::Provider<1>;
using cell2P = cellsColl::Provider<2>;
using cell3P = cellsColl::Provider<3>;

using cellPA = External::CellPA<0, cellsColl, IBus::CheckSum>;

template<typename ADC, uint8_t Channel>
struct TempProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        return sigrow::adcValueToTemperature<std::ratio<1,10>, 40 - 15>(ADC::value(channel)).value;
    }
};
using tempP = TempProvider<adcController, 1>;

struct IBusThrough {
    inline static void init() {
        daisy::template dir<Output>();
    }
    inline static void on() {
        daisy::on();
    }
    inline static void off() {
        daisy::off();
    }
};
using ibt = IBusThrough;

#ifdef USE_IBUS
struct Resetter;
using telemetry = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<cell0P, cell1P, cell2P, cell3P, 
                                    cellsColl::MinProvider<Resetter>, cellsColl::TotalProvider, tempP>, systemTimer, ibt
//                          ,etl::NamedFlag<true>
//                          ,etl::NamedFlag<true>
                          >;
#endif

#ifdef USE_HOTT
using telemetry = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::VarTextMsg<2>, systemTimer>;
//using telemetry = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, systemTimer>;
using battVoltageConverter = Hott::Units::Converter<adc, Hott::Units::battery_voltage_t, 
                                                    std::ratio<Parameter::R1vd + Parameter::R2vd, Parameter::R2vd>>;
using cellVoltageConverter = Hott::Units::Converter<adc, Hott::Units::cell_voltage_t, 
                                                    std::ratio<Parameter::R1vd + Parameter::R2vd, Parameter::R2vd>>;
//inline static auto sensorData = Hott::Experimental::Adapter<telemetry::binaryMessage_type>(telemetry::data());
inline static auto& sensorData = telemetry::data();

#endif

using upperCell = External::SoftSerial::Usart<Meta::List<conn12, void>, Component::Tcd<0>, cellPA,
                                            AVR::BaudRate<9600>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<16>, 
                                            etl::NamedFlag<false>, etl::NamedFlag<false>>;
template<typename Timer, typename TELE>
struct FSM {
    enum class State : uint8_t {Init, Run, Sleep};
    
    static constexpr auto intervall = Timer::intervall;
    static constexpr External::Tick<Timer> idleTimeBeforeSleepTicks{5000_ms};
    
    inline static void reset() {
        stateTicks.reset();
    }

    inline static void periodic() {
    }
    
    inline static void ratePeriodic() {
        auto lastState = mState;
        ++stateTicks;
        switch(mState) {
        case State::Init:
            mState = State::Run;
            break;
        case State::Run:
            if (stateTicks > idleTimeBeforeSleepTicks) {
                mState = State::Sleep;
            }
            break; 
        case State::Sleep:
            break;        
        }
        if (lastState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Init:
                break;
            case State::Run:
                break; 
            case State::Sleep:
                sleep::down();
                TELE::clear();
                mState = State::Run;
                break;        
            }
        }
    }
private:    
    inline static State mState = State::Init;
    inline static External::Tick<Timer> stateTicks;
};

using fsm = FSM<systemTimer, telemetry>;

struct Resetter {
    static inline void reset() {
        fsm::reset();
    }
};

using isrRegistrar = IsrRegistrar<typename upperCell::StartBitHandler, typename upperCell::BitHandler>;

using vdiv = External::AnalogSensor<adcController, 0, std::ratio<0,1>, 
                                    std::ratio<Parameter::R1vd, Parameter::R1vd + Parameter::R2vd>, 
                                    std::ratio<100,1>>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>(); 
    });
   
    systemTimer::init();
    adcController::init();
    
    adc::nsamples(6);
    
    activate::template dir<Output>();
    
    telemetry::init();
    upperCell::init<AVR::HalfDuplex>();

#ifdef USE_HOTT
    etl::copy(telemetry::text()[0], "WM 4S Sensor"_pgm);
    etl::copy(telemetry::text()[1], "Version 0.1"_pgm);
#endif
    
    const auto activateTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    const auto measureTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        telemetry::periodic();
        adcController::periodic();
        systemTimer::periodic([&]{
            telemetry::ratePeriodic();
            fsm::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (activateTimer == t) {
                    activate::toggle();
                }
                if (measureTimer == t) {
#ifdef USE_IBUS
                    auto v = vdiv::value();
                    cellsColl::set(v);
#endif
#ifdef USE_HOTT 
                    sensorData.cell[0] = cellVoltageConverter::convert(cell0P::value()).value;
                    sensorData.cell[1] = cellVoltageConverter::convert(cell1P::value()).value;
                    sensorData.cell[2] = cellVoltageConverter::convert(cell2P::value()).value;
                    sensorData.cell[3] = cellVoltageConverter::convert(cell3P::value()).value;
                    
                    auto [i,v] = cellsColl::MinProvider<Resetter>::indexedValue();
                    sensorData.min_cell_volt = cellVoltageConverter::convert(v).value;
                    sensorData.min_cell_volt_num = i;
                    
                    sensorData.Battery1 = battVoltageConverter::convert(cellsColl::TotalProvider::value()).value;
#endif
                }
            });
        });
    }
}

ISR(PORTA_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<conn12::name_type>>();
}

ISR(TCD0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Tcd<0>::Ovf>();
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
//#if !(defined(USE_IBUS) || defined(USE_HOTT))
//    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
//#endif
    while(true) {
//        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
