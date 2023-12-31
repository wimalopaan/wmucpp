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
#include <mcu/internals/sleep.h>

#include <external/solutions/cells.h>
#include <external/ibus/ibus.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/tick.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using activate = Pin<Port<A>, 7>;

// AIN3 = Voltage

namespace Parameter {
    constexpr auto fRtc = 1000_Hz;
    constexpr uint16_t R1vd = 100; // 100K
    constexpr uint16_t R2vd = 220; // 220K
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;
using sleep = Sleep<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V1_5>;
using adcController = External::Hal::AdcController<adc, Meta::NList<3, 0x1e>>; // 1e = temp
using channel_t = adcController::index_type;

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

struct Sender {
    inline static constexpr std::byte startByte{0xa5};
    inline static constexpr uint8_t offset{1};
    
    template<auto N>
    static inline void copy(const etl::FixedVector<uint16_t, N>& data) {
        static_assert(mSize == N);
        const uint8_t newSize = std::min((uint8_t)(data.size() + offset), mSize);
        if (mValues.size() < newSize) {
            mValues.reserve(newSize);
        }
        for(uint8_t i = offset; i < mValues.size(); ++i) {
            mValues[i] = data[i - offset];
        }
    }    
    static inline void set(const uint16_t v) {
        if (mValues.size() == 0) {
            mValues.reserve(offset);
        }
        mValues[0] = v;
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
    static inline constexpr uint8_t size() {
        return mSize * sizeof(uint16_t) + 2 + 2;
    }
private:
    inline static constexpr uint8_t mSize = 16;
    inline static etl::FixedVector<uint16_t, mSize> mValues{};
};

using sendDown = Sender;
using cell = External::CellPA<0, sendDown, IBus::CheckSum>;

using upDown = AVR::Usart<usart0Position, cell, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<sendDown::size()>>;

template<typename Timer>
struct FSM {
    enum class State : uint8_t {Init, Run, Sleep};

    static constexpr auto intervall = Timer::intervall;
    static constexpr External::Tick<Timer> idleTimeBeforeSleepTicks{5000_ms};
    
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
                upDown::txEnable<false>();
                upDown::txPinDisable();
                sleep::down();
                upDown::txPinEnable();
                upDown::txEnable<true>();
                mState = State::Run;
                break;        
            }
        }
    }
    
    inline static State mState = State::Init;
    inline static External::Tick<Timer> stateTicks;
};

using fsm = FSM<systemTimer>;

using vdiv = External::AnalogSensor<adcController, 0, std::ratio<0,1>, 
                                    std::ratio<Parameter::R1vd, Parameter::R1vd + Parameter::R2vd>, // 100k + 220K
                                    std::ratio<100,1>>; // 10mV
namespace  {
    alarmTimer::index_type sleepTimer;
}

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>(); 
    });
    
    sleep::template init<sleep::PowerDown>();
   
    systemTimer::init();
    adcController::init();

    adc::nsamples(4);
    
    upDown::init<AVR::BaudRate<9600>, AVR::FullDuplex, false>();
    
    activate::template dir<Input>();
    activate::attributes<Meta::List<Attributes::Interrupt<Attributes::BothEdges>>>();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    sleepTimer = alarmTimer::create(5000_ms, External::Hal::AlarmFlags::Periodic);

    {
        activate::resetInt();
        etl::Scoped<etl::EnableInterrupt<>> ei;
        
        while(true) {
            upDown::periodic();
            adcController::periodic();
            systemTimer::periodic([&]{
                fsm::ratePeriodic();
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                        auto v = vdiv::value();
                        sendDown::set(v);
                        sendDown::send<upDown>();
                    }
                    else if (sleepTimer == t) {
                        sleep::down();
                        alarmTimer::restart(sleepTimer);
                    }
                });
            });
        }
    }
}

ISR(PORTA_PORT_vect) {
    activate::resetInt();
    alarmTimer::restart(sleepTimer);
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    while(true) {
//        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
