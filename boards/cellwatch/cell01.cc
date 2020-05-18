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
#include <external/solutions/analogsensor.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using activate = Pin<Port<A>, 7>;

// AIN3 = Voltage

namespace  {
    constexpr auto fRtc = 2000_Hz;
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V1_5>;
using adcController = External::Hal::AdcController<adc, Meta::NList<3, 0x1e>>; // 1e = temp
using channel_t = adcController::index_type;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

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

struct Sender;
using sendDown = Sender;

using cell = CellPA<0, sendDown>;

struct Sender {
    inline static constexpr std::byte startByte{0xa5};
    inline static constexpr uint8_t offset{1};
    
    template<auto N>
    static inline void copy(const etl::FixedVector<uint16_t, N>& data) {
        if (mValues.size() < (data.size() + offset)) {
            mValues.reserve(data.size() + offset);
        }
        for(uint8_t i = 0; i < data.size(); ++i) {
            mValues[i + offset] = 42;
//            mValues[i + offset] = data[i];
        }
    }    
    static inline void set(const uint16_t v) {
//        if (mValues.size() == 0) {
//            mValues.reserve(1);
//        }
//        mValues[0] = v;
        
        mValues.reserve(2);
        mValues[0] = 42;
        mValues[0] = 43;
        
    }
    template<typename Dev>
    static inline void send() {
        IBus::CheckSum cs;
        cs += startByte;
        Dev::put(startByte);
        
        auto s = mValues.size();
        cs += std::byte{s};
        Dev::put(std::byte{s});
        
        for(uint8_t i = 0; i < mValues.size(); ++i) {
            const auto l = etl::nth_byte<0>(mValues[i]);
            cs += l;
            Dev::put(l);
            
            const auto h = etl::nth_byte<1>(mValues[i]);
            cs += h;
            Dev::put(h);
        }
        Dev::put(cs.lowByte());
        Dev::put(cs.highByte());
    }
private:
    inline static constexpr uint8_t mSize = 16;
    inline static etl::FixedVector<uint16_t, mSize> mValues{};
};

using upDown = AVR::Usart<usart0Position, cell, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<16>>;

struct FSM {
    enum class State : uint8_t {Init, Run, Sleep};
};

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

    upDown::init<AVR::BaudRate<9600>>();
    
    activate::template dir<Input>();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        upDown::periodic();
        adcController::periodic();
        systemTimer::periodic([&]{
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    auto v = vdiv::value();
                    sendDown::set(v);
                    sendDown::send<upDown>();
                }
            });
        });
    }
}
