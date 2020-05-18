#define NDEBUG

#include <mcu/avr.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>

#include <external/ibus/ibus.h>
#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/analogsensor.h>
#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using activate = Pin<Port<A>, 1>;
using conn12 = Pin<Port<A>, 7>;
using daisy = Pin<Port<A>, 2>;

// AIN3 = Voltage

namespace  {
    constexpr auto fRtc = 2000_Hz;
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V1_5>;
using adcController = External::Hal::AdcController<adc, Meta::NList<3, 0x1e>>; // 1e = temp
using channel_t = adcController::index_type;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

template<typename ADC, uint8_t Channel>
struct CellsCollector {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};

    inline static constexpr std::byte startByte{0xa5};
    inline static constexpr uint8_t offset{1};

    inline static volatile uint8_t c = 0;
    
    template<auto N>
    static inline void copy(const etl::FixedVector<uint16_t, N>& data) {
        ++c;
        if (mValues.size() < (data.size() + 1)) {
            mValues.reserve(data.size() + 1);
        }
        for(uint8_t i = 0; i < data.size(); ++i) {
            mValues[i + offset] = data[i];
        }
    }    
    static inline void set(const uint16_t v) {
        if (mValues.size() == 0) {
            mValues.reserve(1);
        }
        mValues[0] = v;
    }
    template<auto N>
    struct Provider {
        inline static constexpr auto ibus_type = IBus::Type::type::CELL;
        inline static constexpr void init() {
        }
        inline static constexpr uint16_t value() {
            return mValues[N];
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

template<auto N, typename CallBack = void, typename MCU = DefaultMcuType>
struct CellPA {
    inline static constexpr std::byte startByte{0xa5};

    inline static volatile uint8_t c = 0;
    
    enum class State : uint8_t {Init, AwaitLength, AwaitDataL, AwaitDataH, AwaitCSLow, AwaitCSHigh};
    inline static bool process(const std::byte b) {
        static uint16_t v{};
        static IBus::CheckSum cs;
        switch(mState) {
        case State::Init:
            cs.reset();
            if (b == startByte) {
                cs += b;
                mState = State::AwaitLength;
                mLength.setToBottom();
            }
            break;
        case State::AwaitLength:
            cs += b;
            mLength.set(static_cast<uint8_t>(b));
            if ((mLength != 0) && (mLength <= mValues.capacity)) {
                mState = State::AwaitDataL;
                mValues.clear();
                mValues.reserve(mLength);
                mIndex.setToBottom();
            }
            else {
                mState = State::Init;
            }
            break;
        case State::AwaitDataL:
            cs += b;
            --mLength;
            v = static_cast<uint16_t>(b);
            mState = State::AwaitDataH;
            break;
        case State::AwaitDataH:
            cs += b;
            v |= (static_cast<uint16_t>(b) << 8);
            mValues[mIndex] = v;
            ++mIndex;
            if (mLength.isBottom()) {
                mState = State::AwaitCSLow;
            }
            else {
                mState = State::AwaitDataL;
            }
            break;
        case State::AwaitCSLow:
            cs.lowByte(b);
            mState = State::AwaitCSHigh;
            break;
        case State::AwaitCSHigh:
            cs.highByte(b);
            if (cs) {
                c++;
                if constexpr(!std::is_same_v<CallBack, void>) {
                    CallBack::copy(mValues);                
                }
            }
            mState = State::Init;
            break;
        }
        return true;
    }
private:
    inline static constexpr uint8_t mSize = 16;
    inline static etl::FixedVector<uint16_t, mSize> mValues{};
    inline static etl::uint_ranged<uint8_t, 0, mSize - 1> mLength{};
    inline static etl::uint_ranged<uint8_t, 0, mSize - 1> mIndex{};
    inline static State mState{State::Init};
};

using cellPA = CellPA<0, cellsColl>;

struct TestProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::FLIGHT_MODE;
    inline static constexpr void init() {
    }
    
    inline static volatile uint8_t i = 0;
    
    inline static uint16_t value() {
//        return i;
        return cellsColl::c;
//        return cellPA::c;
    }
};


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

using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<cell0P, cell1P, cell2P, cell3P, TestProvider>, systemTimer, ibt
//                          ,etl::NamedFlag<true>
//                          ,etl::NamedFlag<true>
                          >;

using upperCell = External::SoftSerial::Usart<Meta::List<conn12, void>, Component::Tcd<0>, cellPA,
                                            AVR::BaudRate<9600>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<16>, 
                                            etl::NamedFlag<false>, etl::NamedFlag<false>>;


struct FSM {
    enum class State : uint8_t {Init, Run, Sleep};
};

using isrRegistrar = IsrRegistrar<typename upperCell::StartBitHandler, typename upperCell::BitHandler>;

using vdiv = External::AnalogSensor<adcController, 0, std::ratio<0,1>, 
                                    std::ratio<100, 320>, 
                                    std::ratio<100,1>>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>(); // 2MHz
    });
   
    systemTimer::init();
    adcController::init();
    
    activate::template dir<Output>();
    
    ibus::init();
    upperCell::init<AVR::HalfDuplex>();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        ibus::periodic();
        adcController::periodic();
        systemTimer::periodic([&]{
            ibus::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                                auto v = vdiv::value();
//                auto v = adcController::value(channel_t{0});
                cellsColl::set(v);
                if (periodicTimer == t) {
//                    activate::toggle();
                }
            });
        });
    }
}

ISR(PORTA_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<conn12::name_type>>();
}

ISR(TCD0_OVF_vect) {
//    TestProvider::i++;
    isrRegistrar::isr<AVR::ISR::Tcd<0>::Ovf>();
}
