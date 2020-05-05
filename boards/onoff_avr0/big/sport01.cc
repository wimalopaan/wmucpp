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
//using txPin = void;
using txPin = rxPin;
//using txPin = Pin<Port<B>, 0>;
using dbg1 = Pin<Port<A>, 1>;

template<typename PA>
using sensorUsart = External::SoftSerial::Usart<Meta::List<rxPin, txPin>, Component::Tcd<0>, 
                                            PA, AVR::BaudRate<57600>,
                                            AVR::ReceiveQueueLength<0>,
                                            AVR::SendQueueLength<64>,
                                            etl::NamedFlag<true>
//                                            ,dbg1
>;

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

namespace External {
    namespace SPort {
        template<std::byte ID, template<typename> typename Uart, typename Timer, typename ProviderList = Meta::List<>>
        struct Sensor;
        
        template<std::byte ID, template<typename> typename Uart, typename Timer, typename... Providers>
        struct Sensor<ID, Uart, Timer, Meta::List<Providers...>> {
            inline static constexpr External::Tick<Timer> mResponseDelay{2_ms};
//            std::integral_constant<uint16_t, mResponseDelay.value>::_;

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
                            mRequests = mRequests + 1;
                            mStateTicks.reset();
                            mState = State::ReplyWait;
                        }
                        else {
                            mState = State::Init;
                        }
                        break;
                    default:
                        break;
                    }
                    return true;
                }
                inline static uint8_t requests() {
                    return mRequests;
                }
            private:
                static inline volatile uint8_t mRequests{};
            };
            
            using uart = Uart<ProtocollAdapter>;   
            static_assert(uart::sendQLength >= 16);
            
            inline static constexpr bool useInterrupts = uart::useInterrupts;
            using tick_type = std::conditional_t<useInterrupts, volatile External::Tick<Timer>, External::Tick<Timer>>;
            using state_type = std::conditional_t<useInterrupts, volatile State, State>;
            
            static inline void init() {
                uart::template init<AVR::HalfDuplex>();
            }
            static inline void periodic() {
            }
            static inline void ratePeriodic() {
                const auto lastState = mState;
                ++mStateTicks;
                switch(mState) {
                case State::ReplyWait:
                    mStateTicks.on(mResponseDelay, []{
                        mState = State::Reply;    
                        uart::template rxEnable<false>();
                    });
                    break;
                case State::Reply:
                    reply();
                    mState = State::WaitReplyComplete;
                    break;
                case State::WaitReplyComplete:
                    if (uart::isIdle()) {
                        uart::template rxEnable<true>();
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
            struct CheckSum {
                void operator+=(const std::byte b) {
                    mValue += uint8_t(b);
                    mValue += mValue >> 8;
                    mValue &= 0x00ff;
                }
                std::byte value() const {
                    return etl::nth_byte<0>(0xff - mValue);
                }
            private:
                    uint16_t mValue{};
            };
            inline static void reply() {
                CheckSum cs;
                stuffResponse(0x10_B, cs);
//                stuff(uint16_t{0x0200}, cs); // current 0,1A
                stuff(uint16_t{0x0900}, cs); // voltage 0,01V
                stuff(uint32_t{2}, cs);
                stuff(cs);
            }
            inline static void stuff(const CheckSum& cs) {
                uart::put(cs.value());
            }
            inline static void stuff(const uint32_t b, CheckSum& cs) {
                stuffResponse(etl::nth_byte<0>(b), cs);
                stuffResponse(etl::nth_byte<1>(b), cs);
                stuffResponse(etl::nth_byte<2>(b), cs);
                stuffResponse(etl::nth_byte<3>(b), cs);
            }
            inline static void stuff(const uint16_t b, CheckSum& cs) {
                stuffResponse(etl::nth_byte<0>(b), cs);
                stuffResponse(etl::nth_byte<1>(b), cs);
            }
            inline static void stuffResponse(const std::byte b, CheckSum& cs) {
                if (b == 0x7e_B) {
                    cs += 0x7d_B;
                    uart::put(0x7d_B);
                    cs += 0x5d_B;
                    uart::put(0x5e_B);
                }
                else {
                    cs += b;
                    uart::put(b);
                }
            }
        private:            
            inline static tick_type mStateTicks;
            inline static state_type mState{State::Init};
        };
    }
}

struct VoltageProvider {
    
};
using vProv = VoltageProvider;

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

using sensor = External::SPort::Sensor<0x22_B, sensorUsart, systemTimer, Meta::List<vProv>>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

using isrRegistrar = IsrRegistrar<sensor::uart::StartBitHandler, sensor::uart::BitHandler>;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });

    portmux::init();
    systemTimer::init();
    sensor::init();
    terminalDevice::init<AVR::BaudRate<9600>>();
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    uint16_t counter{};
    
    while(true) {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        terminalDevice::periodic();
        sensor::periodic();        
        systemTimer::periodic([&]{
            sensor::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (t == periodicTimer) {
                    etl::outl<terminal>("c: "_pgm, ++counter, 
                                        " r: "_pgm, sensor::ProtocollAdapter::requests()
                                        );
                }
            });
        });
    }
}

ISR(PORTB_PORT_vect) {
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
