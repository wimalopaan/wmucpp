#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/spi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>
#include <mcu/internals/syscfg.h>
#include <mcu/internals/twi.h>
 
#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/apa102.h>
#include <external/solutions/series01/sppm_in.h>
#include <external/solutions/rc/busscan.h>
#include <external/solutions/series01/cppm_out.h>
#include <external/solutions/rc/rf.h>
#include <external/solutions/si5351.h>

#include <mcu/pgm/pgmarray.h>

#include <std/chrono>
#include <std/bit>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

#ifndef NDEBUG
namespace xassert {
    etl::StringBuffer<160> ab;
    etl::StringBuffer<10> aline;
    bool on{false};
}
#endif

namespace  {
    constexpr auto fRtc = 2048_Hz;
//    constexpr auto fRtc = 2 * 2048_Hz;
    constexpr Twi::Address mcp23008{39};
    constexpr Twi::Address si5351{0x60};
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

// RTC hat große Abweichung
//using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

using systemTimer = SystemTimer<Component::Timer<0, A>, fRtc>;

using dbgPin = Pin<Port<A>, 6>;

//using a0Pin = Pin<Port<A>, 0>; // usart0
//using a1Pin = Pin<Port<A>, 1>;
//using a2Pin = Pin<Port<A>, 2>;

using led1 = ActiveHigh<Pin<Port<F>, 4>, Output>; 
using led2 = ActiveHigh<Pin<Port<F>, 5>, Output>; 

//using a5Pin = Pin<Port<A>, 5>; 

using fSwitchPin = Pin<Port<C>, 1>;
using fSwitch = ActiveHigh<fSwitchPin, Output>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>; 
using tdev = Usart<usart0Position, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<16>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<tdev>;

using tca0Position = AVR::Portmux::Position<AVR::Component::Tca<0>, Portmux::Default>;
//using cppm = External::Ppm::Cppm<tca0Position, std::integral_constant<uint8_t, 16>, AVR::UseInterrupts<true>>;
//using lut0 = Ccl::SimpleLut<0, Ccl::Input::Mask, Ccl::Input::Tca0<1>, Ccl::Input::Mask>;    

using twi0Position = Portmux::Position<Component::Twi<0>, Portmux::Alt2>;
using twi = AVR::Twi::Master<twi0Position, etl::NamedConstant<128>>;

using portmux = Portmux::StaticMapper<Meta::List<tca0Position, twi0Position, usart0Position>>;

using si = External::SI5351::Clock<twi, si5351>;

struct SetBaseFreq {
    inline static void once() {
        fSwitch::activate();
    }
};
struct SetUpperFreq {
    inline static void once() {
        fSwitch::inactivate();
    }
};


template<typename MCU = DefaultMcuType>
struct Fsm {
    enum class State : uint8_t {Undefined, Init, Set, Set2, Run, Sync, Bit};
    
    static inline constexpr External::Tick<systemTimer> mInitTicks{500_ms};
    static inline constexpr External::Tick<systemTimer> mChangeTicks{1_ms};
    static inline constexpr External::Tick<systemTimer> mDebugTicks{500_ms};
    
    static inline void init() {
        led1::init();
        led2::init();
        
//        dbgPin::dir<Output>();
//        a5Pin::dir<Output>();
        
        fSwitch::init();
        fSwitch::activate();
        
        twi::init();
        
        tdev::init<BaudRate<9600>>();
        
        etl::outl<terminal>("fsk00"_pgm);
    }
    static inline void periodic() {
        dbgPin::toggle();
        tdev::periodic();
        twi::periodic();
        si::periodic();
    }
    static inline void ratePeriodic() {
//        fSwitch::toggle();
//        return;
        const auto oldState = mState;
        ++mStateTick;
        ++mChangeTick;
        (++mDebugTick).on(mDebugTicks, []{
        });
        switch(mState) {
        case State::Undefined:
            mStateTick.on(mInitTicks, []{
               mState = State::Init; 
            });
            break;
        case State::Init:
            mStateTick.on(mInitTicks, []{
                si::setOutput(0);
                mState = State::Set; 
            });
            break;
        case State::Set:
            if (si::setFrequency(7'000'000_Hz, &div)) {
                si::setOutput(2);
                mState = State::Set2;
            }
            break;
        case State::Set2:
            if (si::setFrequency(7'005'000_Hz)) {
                mState = State::Run;
            }
            break;
        case State::Run:
            mState = State::Sync;
            mBitCounter = 0;
            SetUpperFreq::once();
            break;
        case State::Sync:
            ++mBitCounter;
            if (mBitCounter == 10) {
                SetBaseFreq::once();
                mState = State::Bit;
                mBitCounter = 0;
                mByteCounter = 0;
                mData[1] += 1;
                mActual = mData[0];
                uint16_t cs = 0;
                for(uint8_t i = 0; i < 8; ++i) {
                    etl::crc16(cs, mData[i]);
                }
                mData[8] = (cs >> 0) & 0xff;
                mData[9] = (cs >> 8) & 0xff;
            }
            break;
        case State::Bit:
            if (mByteCounter == mData.size()) {
                mState = State::Sync;
                mBitCounter = 0;
                SetUpperFreq::once();
            }
            else {
                if (mBitCounter < 8) {
                    if (mActual & 0x01) {
                        SetUpperFreq::once();
                    }
                    else {
                        SetBaseFreq::once();
                    }
                    mActual >>= 1;
                    ++mBitCounter;
                }
                else {
                    SetBaseFreq::once();
                    mBitCounter = 0;
                    ++mByteCounter;
                    if (mByteCounter < mData.size()) {
                        mActual = mData[mByteCounter];
                    }
                }
            }
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                etl::outl<terminal>("S: Undef"_pgm);
                break;
            case State::Init:
                etl::outl<terminal>("S: Init"_pgm);
                led1::activate();
                break;
            case State::Set:
                etl::outl<terminal>("S: Set"_pgm);
                led2::activate();
                break;
            case State::Set2:
                etl::outl<terminal>("S: Set2"_pgm);
                break;
            case State::Run:
                etl::outl<terminal>("S: Run"_pgm);
                break;
            }
        } 
    }
private:
    static inline uint8_t mActual;
    static inline std::array<uint8_t, 10> mData{40, 0x00, 42, 43, 44, 45, 46, 47, 0, 0};
    
    static inline uint32_t div{0};
    static inline State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline External::Tick<systemTimer> mChangeTick;
    
    static inline uint8_t mBitCounter{};
    static inline uint8_t mByteCounter{};
};

using fsm = Fsm<>;

int main() {
    portmux::init();
    ccp::unlock([]{
         clock::template init<Project::Config::fMcuMhz>();
    });
    systemTimer::init();
    
    fsm::init();
    {
        while(true) {
            fsm::periodic();                        
            systemTimer::periodic([&]{
                fsm::ratePeriodic();                        
            });
        }
    }    
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
//        terminalDevice::periodic();
//        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
