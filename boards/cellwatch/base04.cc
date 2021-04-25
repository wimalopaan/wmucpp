#define NDEBUG

#define USE_HOTT
//#define USE_IBUS
//#define USE_SPORT

#define SBUS_IBUS_NO_WARN

#ifndef GITMAJOR
# define VERSION_NUMBER 2300
#endif

#ifdef USE_HOTT
# define HOTT_VERSION "Ver: " GITTAG2_PGM
#endif

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
#include <external/sbus/sport.h>
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
#ifndef USE_SPORT
using conn12 = Pin<Port<A>, 7>; // usart rx
#endif
using daisy = Pin<Port<A>, 2>;
#ifdef USE_SPORT
using txrx = Pin<Port<A>, 6>; // soft-usart SPORT
#endif

// AIN3 = Voltage

namespace Parameter {
#ifdef USE_IBUS
    constexpr auto fRtc = 2000_Hz;
#endif
#ifdef USE_HOTT
    constexpr auto fRtc = 500_Hz;
#endif
#ifdef USE_SPORT
    constexpr auto fRtc = 1000_Hz;
#endif
    constexpr uint16_t R1vd = 100; // 100K
    constexpr uint16_t R2vd = 220; // 220K
    constexpr uint16_t countForMinCell = 10;
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V1_5>;
using adcController = External::Hal::AdcController<adc, Meta::NList<3, 0x1e>>; // 1e = temp
using channel_t = adcController::index_type;

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 3>;

template<typename ADC, uint8_t Channel>
struct CellsCollector {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};

    inline static constexpr std::byte startByte{0xa5};
    inline static constexpr uint8_t offset{1};

    template<auto N>
    static inline void copy(const etl::FixedVector<uint16_t, N>& data) {
        etl::Scoped<etl::DisbaleInterrupt<>> ei;
        ++mUpdateCounter;
        for(uint8_t i = offset; i < mValues.size(); ++i) {
            const uint8_t k = i - offset;
            if (k < data.size()) {
                mValues[i] = data[k];
            }
            else {
                mValues[i] = 0;
            }
        }
    }    
    static inline void set(const uint16_t& v) {
        etl::Scoped<etl::DisbaleInterrupt<>> ei;
        mValues[0] = v;
    }
    static inline uint8_t size() {
        return mValues.size();
    }
    static inline void clear() {
        etl::Scoped<etl::DisbaleInterrupt<>> ei;
        for(uint8_t i = 1; i < mValues.size(); ++i) {
            mValues[i] = 0;
        }
    }
    template<uint8_t N>
    struct SPortCell {
        inline static constexpr auto valueId = External::SPort::ValueId::Cells;
        inline static constexpr void init() {}
        inline static constexpr uint32_t value() {
            return (((uint32_t)(mValues[N + 1]*5) & 0x0fff) << 20) | (((uint32_t)(mValues[N]*5) & 0x0fff) << 8) | (mCellTotal << 4) | N;
        }
//        inline static void setTotal(const uint8_t max) {
//            mCellTotal = max;
//        }
    private:
        static inline uint8_t mCellTotal{4};
    };
    struct VersionProvider {
        inline static constexpr auto valueId = External::SPort::ValueId::DIY;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
#if defined(GITMAJOR) && defined(GITMINOR)
        static_assert(GITMINOR < 10);
//            return mUpdateCounter;
        return GITMAJOR * 100 + GITMINOR;
#else
        return VERSION_NUMBER;
#endif
            return 42;
        }
    };
    
    template<auto N>
    struct Provider {
        inline static constexpr auto valueId = External::SPort::ValueId::Voltage;
        inline static constexpr auto ibus_type = IBus::Type::type::CELL;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            return mValues[N];
        }
    };
    
    struct MinProvider {
        inline static constexpr auto valueId = External::SPort::ValueId::Voltage;
        inline static constexpr auto ibus_type = IBus::Type::type::CELL;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            auto min = mValues[0];
            for(uint8_t i = 1; i < mValues.size(); ++i) {
                if (mValues[i] > Parameter::countForMinCell) {
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
                if (mValues[i] > Parameter::countForMinCell) {
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
        inline static constexpr auto valueId = External::SPort::ValueId::Voltage;
        inline static constexpr auto ibus_type = IBus::Type::type::EXTERNAL_VOLTAGE;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            uint16_t t = mValues[0];
            for(uint8_t i = 1; i < mValues.size(); ++i) {
                t += mValues[i];
            }
            return t;
        }
    };
    inline static etl::uint_ranged<uint8_t, 0, 10> mUpdateCounter{10};
private:
    inline static constexpr uint8_t mSize = 4;
    inline static volatile etl::array<uint16_t, mSize> mValues{};
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

#ifdef USE_IBUS
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

using telemetry = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<cell0P, cell1P, cell2P, cell3P, 
                                    cellsColl::MinProvider, cellsColl::TotalProvider, tempP>, systemTimer, ibt
//                          ,etl::NamedFlag<true>
//                          ,etl::NamedFlag<true>
                          >;
#endif

#ifdef USE_HOTT
using telemetry = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::VarTextMsg<2>, systemTimer>;
inline static auto& sensorData = telemetry::data();
#endif

#if defined(USE_IBUS) || defined(USE_HOTT)
using upperCell = External::SoftSerial::Usart<Meta::List<conn12, void>, Component::Tcd<0>, cellPA,
                                            AVR::BaudRate<9600>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<2>, 
                                            etl::NamedFlag<false>, etl::NamedFlag<false>>;
using isrRegistrar = IsrRegistrar<typename upperCell::StartBitHandler, typename upperCell::BitHandler>;
#endif

using vdiv = External::AnalogSensor<adcController, 0, std::ratio<0,1>, 
                                    std::ratio<Parameter::R1vd, Parameter::R1vd + Parameter::R2vd>, 
                                    std::ratio<100,1>>; // 10mV

#ifdef USE_SPORT
template<typename ADC, uint8_t Channel, typename SigRow>
struct InternalTempProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto valueId = External::SPort::ValueId::Temp2;
    inline static constexpr void init() {}
    inline static constexpr uint32_t value() {
//        return SigRow::template adcValueToTemperature<std::ratio<1,10>, 40 - 15>(ADC::value(channel)).value;
        return SigRow::template adcValueToTemperature<std::ratio<1,1>, 0>(ADC::value(channel)).value;
    }
};

using tempiP = InternalTempProvider<adcController, 1, sigrow>;

using upperCell = Usart<usart0Position, cellPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;

// inverted soft-uart
template<typename PA>
using sensorUsart = External::SoftSerial::Usart<Meta::List<txrx, txrx>, Component::Tcd<0>, PA, AVR::BaudRate<57600>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>, etl::NamedFlag<true>>;

using cells01 = cellsColl::SPortCell<0>;
using cells23 = cellsColl::SPortCell<2>;
using telemetry = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer
                                       ,Meta::List<cellsColl::VersionProvider, cells01, cells23
                                       ,cell0P, cell1P, cell2P, cell3P, tempiP
, cellsColl::MinProvider
, cellsColl::TotalProvider
>>;

using isrRegistrar = IsrRegistrar<typename telemetry::uart::StartBitHandler, typename telemetry::uart::BitHandler>;
#endif

int main() {
    portmux::init();
    
    ccp::unlock([]{
        static_assert(F_OSC == 20000000);
        clock::prescale<1>(); 
//        clock::prescale<2>(); 
    });
    
    systemTimer::init();
    adcController::init();
    
    adc::nsamples(4);
    
    activate::template dir<Output>();
    
    telemetry::init();
    
#if defined(USE_IBUS) || defined(USE_HOTT)
    upperCell::init<AVR::HalfDuplex>();
#endif
#ifdef USE_SPORT
    upperCell::init<AVR::BaudRate<9600>, FullDuplex, false>();
    upperCell::txEnable<false>();
#endif
    
#ifdef USE_HOTT
    daisy::dir<Output>();
    daisy::on();
    
    etl::copy(telemetry::text()[0], "WM 4S Sensor"_pgm);
    etl::copy(telemetry::text()[1], HOTT_VERSION);
#endif
    
    const auto activateTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    const auto measureTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);
    const auto updateTimer = alarmTimer::create(300_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        telemetry::periodic();
#ifdef USE_SPORT
        upperCell::periodic();
#endif
        adcController::periodic();
        systemTimer::periodic([&]{
            telemetry::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (activateTimer == t) {
                    activate::toggle();
                }
                if (updateTimer == t) {
                    --cellsColl::mUpdateCounter;
                    if (cellsColl::mUpdateCounter.isBottom()) {
                        cellsColl::clear();
                    }
                }
                if (measureTimer == t) {
                    const auto v0 = vdiv::value();  
                    cellsColl::set(v0);
#ifdef USE_HOTT 
                    sensorData.cell[0] = cell0P::value() / 2; // 20mV bei hott
                    sensorData.cell[1] = cell1P::value() / 2;
                    sensorData.cell[2] = cell2P::value() / 2;
                    sensorData.cell[3] = cell3P::value() / 2;
                    
                    const auto [i,v] = cellsColl::MinProvider::indexedValue();
                    sensorData.min_cell_volt = v / 2;
                    sensorData.min_cell_volt_num = i;
                    
                    sensorData.Battery1 = cellsColl::TotalProvider::value() / 10; // 100mV
                            
                    using adci_t = adcController::index_type;
                    sensorData.temperature1 = 20 + sigrow::adcValueToTemperature<std::ratio<1,1>, 0>(adcController::value(adci_t{1})).value;
                    sensorData.temperature2 = sensorData.temperature1;
#endif
#ifdef USE_SPORT
//                    cells01::setTotal(cellsColl::size());
//                    cells23::setTotal(cellsColl::size());
#endif
                }
            });
        });
    }
}

ISR(PORTA_PORT_vect) {
#ifdef USE_SPORT
    isrRegistrar::isr<AVR::ISR::Port<txrx::name_type>>();
#else
    isrRegistrar::isr<AVR::ISR::Port<conn12::name_type>>();
#endif
}

ISR(TCD0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Tcd<0>::Ovf>();
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    while(true) {
        activate::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
