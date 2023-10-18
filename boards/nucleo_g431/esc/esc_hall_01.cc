#define USE_MCU_STM_V2
#define NDEBUG
#define OLD

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
//    using exti = devs::exti;
    
    using adc = devs::adc1;
    using tp2 = devs::tp2;
//    using tp3 = devs::tp3;
    
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    
    enum class State : uint8_t {Undefined, Init, Run};
    
    static inline void init() {
        devs::init();
        devs::led::set();
    }   
    static inline void periodic() {
        term::periodic();
        if (!adc::busy()) {
//            tp2::toggle();
            mLastAdc = adc::value();
        }
    }
    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(debugTicks, []{
                mState = State::Init;
            });
        break;
        case State::Init:
            mStateTick.on(debugTicks, []{
                mState = State::Run;
                adc::start();
//                driver::all_on();
                hall::isr();
            });
        break;
        case State::Run:
            driver::duty((0.3 * mLastAdc) / 4095);
            driver::duty(1.0, 1.0, 1.0);
//            driver::forceStep();
            (++mDebugTick).on(debugTicks, []{
                led::toggle();
                IO::outl<term>("\no: ", hall::overTrigger, " h: ", hall::lastmean(), " ic: ", hall::isrCount); 
                IO::outl<term>("a: ", mLastAdc, " s: ", hall::state);
//                for(const auto v: hall::last) {
//                    IO::out<term>(v, ' ');
//                }
            });
        break;
        }
        if (oldState != mState) {
            mStateTick.reset();
        }
    }
private:
    static inline uint16_t mLastAdc{};
//    static inline float mScale = 0.7;
//    static inline uint32_t mCounter{};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState{State::Undefined};
};

void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}

using devs = Devices<ESC_HALL_01, Config, Mcu::Stm::Stm32G431>;

int main() {
    using gfsm = GFSM<devs>;
    gfsm::init();

    NVIC_EnableIRQ(TIM4_IRQn);
//    NVIC_EnableIRQ(EXTI9_5_IRQn);
    __enable_irq();
    
    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

extern "C" {

//void EXTI9_5_IRQHandler() __attribute__ ((isr));
//void EXTI9_5_IRQHandler()  { 
//    EXTI->PR1 |= EXTI_PR1_PIF6;
//    devs::exti::isr();    
//}


void TIM4_IRQHandler() __attribute__ ((isr));
void TIM4_IRQHandler()  {   
    devs::tp2::toggle();
    devs::hall::isr();    
}
}
