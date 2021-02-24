#define NDEBUG

#define SBUS_IBUS_NO_WARN

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

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

//using rxPin = Pin<Port<A>, 6>; // default
//using txPin = rxPin;
using activate = Pin<Port<A>, 3>;
//using conn12 = Pin<Port<A>, 7>;
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

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;


template<auto N, typename CallBack, typename MCU = DefaultMcuType>
struct CellPA {
    inline static constexpr std::byte startByte{0xa5};
    
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
            if ((mLength != 0) && (mLength <= mValues.size())) {
                mState = State::AwaitDataL;
                mValues.clear();
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
            mValues.push_back(v);
            if (mLength.isBottom()) {
                mState = State::AwaitCSLow;
                cs.reset();
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
                CallBack::copy(mValues);                
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
    inline static State mState{State::Init};
};

struct Sender;
using sendUp = Sender;

using cell = CellPA<0, sendUp>;

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

struct Debug {
    inline static void init() {
    }
    template<auto n>
    inline static void set() {
//        activate::toggle();
//        if constexpr(n == 0) {
//            activate::low();
//        }
//        if constexpr(n == 1) {
//            activate::high();
//        }
    }
    inline static void set(const std::byte) {
        
    }
};

using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<tempP>, systemTimer, ibt
                          ,etl::NamedFlag<true>
                        ,etl::NamedFlag<true>
//                       ,Debug
                          >;


struct Sender {
    inline static constexpr std::byte startByte{0xa5};
    inline static constexpr uint8_t offset{1};
    template<auto N>
    static inline void copy(const etl::FixedVector<uint16_t, N>& data) {
        for(uint8_t i = 0; i < data.size(); ++i) {
            mValues[i + offset] = data[i];
        }
    }    
    static inline void set(const uint16_t v) {
        mValues[0] = v;
    }
    template<typename Dev>
    static inline void send() {
        IBus::CheckSum cs;
        cs += startByte;
        Dev::put(startByte);
        for(uint8_t i = 0; i < mValues.size(); ++i) {
            const auto l = etl::nth_byte<0>(mValues[i]);
            Dev::put(l);
            const auto h = etl::nth_byte<1>(mValues[i]);
            Dev::put(h);
        }
        Dev::put(cs.lowByte());
        Dev::put(cs.highByte());
    }
private:
    inline static constexpr uint8_t mSize = 16;
    inline static etl::FixedVector<uint16_t, mSize> mValues{};
};

struct FSM {
    enum class State : uint8_t {Init, Run, Sleep};
};

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>(); // 2MHz
    });
   
    systemTimer::init();
    adcController::init();
    
    activate::template dir<Output>();
    
    ibus::init();
    
//    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        ibus::periodic();
        adcController::periodic();
        systemTimer::periodic([&]{
            ibus::ratePeriodic();
//            alarmTimer::periodic([&](const auto& t){
//                if (periodicTimer == t) {
////                    activate::toggle();
//                }
//            });
        });
    }
}
