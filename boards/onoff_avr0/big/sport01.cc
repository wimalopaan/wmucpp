#define NDEBUG

//#define USE_HOTT
#define USE_SPORT
//#define USE_IBUS
//#define FS_I6S
//#define USE_DAISY
//#define USE_EEPROM

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
#include <mcu/internals/reset.h>
#include <mcu/internals/watchdog.h>
#include <mcu/common/uninitialized.h>

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
#include <external/solutions/series01/swuart.h>

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
#ifdef USE_SPORT
    constexpr auto fRtc = 1000_Hz;
#endif
    
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using rxPin = Pin<Port<B>, 1>;
using txPin = Pin<Port<B>, 0>;

template<typename PA>
using sensorUsart = External::SoftSerial::Usart<Meta::List<rxPin, txPin>, Component::Tcd<0>, 
                                            PA, AVR::BaudRate<57600>,
                                            AVR::ReceiveQueueLength<0>
>;

namespace External {
    namespace SPort {
        template<std::byte ID, template<typename> typename Uart, typename Timer>
        struct Sensor {
            inline static constexpr External::Tick<Timer> mResonseDelay{2_ms};

            enum class State : uint8_t {Init, Request, ReplyWait, Reply, WaitReplyComplete};
            
            struct ProtocollAdapter {
                static inline bool process(const std::byte b) {
                    switch(mState) {
                    case State::Init: 
                        if (b == 0x7e_B) {
                            mState = State::Request;
                        }
                        break;
                    case State::Request: 
                        if (b == ID) {
                            mState = State::ReplyWait;
                        }
                        break;
                    default:
                        break;
                    }
                    return true;
                }
                
            };
            using uart = Uart<ProtocollAdapter>;   
            
            static_assert(uart::sendQLength >= 16);
            
            static inline void init() {
                uart::template init<AVR::HalfDuplex>();
            }

            static inline void periodic() {
//                uart::periodic();
            }
            static inline void ratePeriodic() {
                const auto lastState = mState;
                ++mStateTicks;
                switch(mState) {
                case State::ReplyWait:
                    mStateTicks.on(mResonseDelay, []{
                        mState = State::Reply;    
                    });
                    break;
                case State::Reply:
                    stuffResponse();
                    mState = State::WaitReplyComplete;
                    break;
                case State::WaitReplyComplete:
                    if (uart::isIdle()) {
                        mState = State::Init;
                    }
                    break;
                default:
                    break;
                }
                if (lastState != mState) {
                    mStateTicks.reset();
                }
            }
        private:
            inline static void stuffResponse() {
                uart::put(0x55_B);
            }
            
            inline static External::Tick<Timer> mStateTicks;
            inline static State mState{State::Init};
        };
    }
}


using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

using sensor = External::SPort::Sensor<0xA1_B, sensorUsart, systemTimer>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

using isrRegistrar = IsrRegistrar<sensor::uart::StartBitHandler, sensor::uart::BitHandler>;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });

    portmux::init();
    systemTimer::init();
    sensor::init();
    
//    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);
//    const auto resetTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
    while(true) {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        sensor::periodic();        
        systemTimer::periodic([&]{
            sensor::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
            });
        });
    }
}

ISR(PORTA_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<rxPin::name_type>>();
}

ISR(TCD0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Tcd<0>::Ovf>();
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
