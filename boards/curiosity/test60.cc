#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>

#include <external/hal/alarmtimer.h>
#include <external/ibus/ibus.h>

#include <external/hal/adccontroller.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>
#include <etl/converter.h>
#include <etl/fixedpoint.h>

template<typename Dir, typename Step, typename Timer>
struct Stepper {
    enum class State : uint8_t {Init, Off, Forward, ForwardStart, Backward, BackwardStart, };
    
    inline static uint16_t steps = 200 * 16;
    
    inline static constexpr void init() {
        Dir::low();
        Step::low();
        Dir::template dir<AVR::Output>();
        Step::template dir<AVR::Output>();
    }
    inline static constexpr void position(const uint16_t p) {
        mTargetPosition = (uint32_t(p - 988) * steps) / 1024 + 1600;
    }
    inline static constexpr bool active() {
        return mState != State::Off;
    }
    inline static constexpr void ratePeriodic() {
        const auto lastState = mState;
        switch(mState) {
        case State::Init:
            mState = State::Off;
            break;
        case State::Off:
            if (mTargetPosition < mActualPosition) {
                if (((mActualPosition - mTargetPosition) > 1600) && (mActualPosition > 3200))  {
                    mActualPosition -= 3200;
                    mState = State::BackwardStart;
                }
                else {
                    mState = State::ForwardStart;
                }
            }
            if (mTargetPosition > mActualPosition) {
                if (((mTargetPosition - mActualPosition) > 1600) && (mActualPosition < 3200)) {
                    mActualPosition += 3200;
                    mState = State::ForwardStart;
                }
                mState = State::BackwardStart;
            }
            break;
        case State::ForwardStart:
            Dir::high();
            mState = State::Forward;
            break;
        case State::Forward:
            Step::high();
            --mActualPosition;
            Step::low();
            if (mTargetPosition >= mActualPosition) {
                mState = State::Off;
            }
            break;
        case State::BackwardStart:
            Dir::low();
            mState = State::Backward;
            break;
        case State::Backward:
            Step::high();
            ++mActualPosition;
            Step::low();
            if (mTargetPosition <= mActualPosition) {
                mState = State::Off;
            }
            break;
        }
        if (lastState != mState) {
            switch(mState) {
            case State::Init:
                break;
            case State::Off:
                break;
            case State::ForwardStart:
                break;
            case State::Forward:
                break;
            case State::BackwardStart:
                break;
            case State::Backward:
                break;
            }
        }
    }
private:
    static inline State mState{State::Init};
    static inline uint16_t mActualPosition{3200};
    static inline uint16_t mTargetPosition{3200};
};


using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;


using PortA = Port<A>;
using PortF = Port<F>;

using led = Pin<PortF, 5>; 

using dirPin = Pin<Port<E>, 2>;
using stepPin = Pin<Port<E>, 3>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Alt1>;

//using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltB>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position>>;

using terminalDevice = Usart<usart2Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0x1e>>; // 1e = temp

namespace  {
    constexpr auto fRtc = 500_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using stepper = Stepper<dirPin, stepPin, systemTimer>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    terminalDevice::init<BaudRate<9600>>();
    servo::init<BaudRate<115200>>();
    
    systemTimer::init();
    
    led::template dir<Output>();     
    
    stepper::init();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
    while(true) {
        terminalDevice::periodic();
        servo::periodic();
        systemTimer::periodic([&]{
            stepper::ratePeriodic();
            stepper::position(servo_pa::value(3).toInt());
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
                    etl::outl<terminal>("test60: "_pgm, servo_pa::value(3).toInt());
                }
            });
        });
    }
}

