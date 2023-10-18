#define USE_MCU_STM_V2
#define NDEBUG

#include "devices.h"

#include <chrono>
#include <cassert>

using namespace std::literals::chrono_literals;

struct Config {
    Config() = delete;
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using term = devs::serial2;
    using systemTimer = devs::systemTimer;
    
    using led = devs::led;
    using driver = devs::driver;
    using hall = devs::hall;
    using pos = devs::pos_estimator;
    
    using adc = devs::adc1;
//    using tp2 = devs::tp2;
//    using tp3 = devs::tp3;
    
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    
    enum class State : uint8_t {Undefined, Init, Run, Info};
    
    static inline void init() {
        devs::init();
        devs::led::set();
    }   
    static inline void periodic() {
        term::periodic();
        if (!adc::busy()) {
            mLastAdc = adc::value();
        }
    }
    static inline void ratePeriodic() {
        static int i = 0;
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(debugTicks, []{
                mState = State::Info;
            });
        break;
        case State::Info:
            mState = State::Init;
            if (term::isIdle()) {
//                IO::out<term>((uint8_t)(pos::sPhaseU[i++] * 100), ",");
//                if (i == pos::rotLength) {
//                    mState = State::Init;
//                    i = 0;
//                    IO::outl<term>();
//                }
            }
        break;
        case State::Init:
            mStateTick.on(debugTicks, []{
                mState = State::Run;
                driver::all_on();
                adc::start();
                
            });
        break;
        case State::Run:
            driver::duty((1.0 * mLastAdc) / 4095);
            (++mDebugTick).on(debugTicks, []{
                led::toggle();
                IO::outl<term>("v: ", (uint16_t)hall::vMean.value(), " p: ", (uint16_t) pos::mPos, " s: ", (uint16_t) (100 * driver::mScale)); 
                IO::outl<term>("h: ", hall::hallSensors, " s: ", hall::state);
            });
        break;
        }
        if (oldState != mState) {
            mStateTick.reset();
        }
    }
private:
    static inline uint16_t mLastAdc{};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState{State::Undefined};
};

void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}

using devs = Devices<ESC_HALL_02, Config, Mcu::Stm::Stm32G431>;

int main() {
    using gfsm = GFSM<devs>;
    gfsm::init();

    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_EnableIRQ(TIM7_IRQn);
    __enable_irq();
    
    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

extern "C" {

void TIM7_IRQHandler() __attribute__ ((isr));
void TIM7_IRQHandler()  {   
    devs::tp2::toggle();
    devs::pos_estimator::isr();
}

void TIM4_IRQHandler() __attribute__ ((isr));
void TIM4_IRQHandler()  {   
    devs::hall::isr();    
}
}
