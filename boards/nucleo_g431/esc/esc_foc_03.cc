#define USE_MCU_STM_V2
#define NDEBUG

#include "devices_foc.h"

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
    using meas = devs::measurement;
    
    using adc = devs::adc1;
//    using tp2 = devs::tp2;
//    using tp3 = devs::tp3;

    struct Printer {
        enum class State {StartLine, Elements};
        
        static inline void start() {
            sIndex = 0;            
            mState = State::StartLine;
        }   
        static inline bool next() {
            if (!term::isIdle()) {
                return false;
            }
            switch(mState) {
            case State::StartLine:
                IO::out<term>("\nh: ");
                mState = State::Elements;
            break;
            case State::Elements:
                IO::out<term>(" [", meas::mHallSectionLengths[sIndex].lastLength, ",", meas::mHallSectionLengths[sIndex].sineIndex, "]"); 
                ++sIndex;
                if (sIndex == (6 * 7)) {
//                    if (sIndex == meas::mHallSectionLengths.size()) {
                    sIndex = 0;
                    return true;
                }
            break;
            }
            return false;
        }
        static inline State mState{State::StartLine};
        static inline uint8_t sIndex{};
    };
    
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> incTicks{10ms};
    static inline constexpr External::Tick<systemTimer> measTicks{10'000ms};
    
    enum class State : uint8_t {Undefined, Init, Run, Info, Print,
                               MeasureStart, MeasureRamp, Measure, MeasureStop,
                               Set, IncrementMeasurementScale, DecrementMeasurementScale,
                               };
    
    static inline void init() {
        devs::init();
        devs::led::set();
    }   
    static inline void periodic() {
        term::periodic();
    }
    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(debugTicks, []{
                mState = State::Info;
            });
        break;
        case State::MeasureStart:
            mStateTick.on(debugTicks, []{
                mState = State::MeasureRamp;
            });
        break;
        case State::MeasureRamp:
            mStateTick.on(debugTicks, []{
                mState = State::Measure;
            });
            (++mMeasTick).on(incTicks, []{
                devs::measurement::period(mMeasPeriod);
                driver::scale(mMeasScale);
            });
        break;
        case State::Measure:
            mStateTick.on(measTicks, []{
                mState = State::MeasureStop;
            });
        break;
        case State::MeasureStop:
            mStateTick.on(debugTicks, []{
                mState = State::Run;
            });
        break;
        case State::Info:
            mState = State::Init;
        break;
        case State::Init:
            mStateTick.on(debugTicks, []{
                mState = State::MeasureStart;
                adc::start();
            });
        break;
        case State::Run:
            (++mDebugTick).on(debugTicks, []{
                led::toggle();
                IO::outl<term>("s[0]: ", Storage::adcValues[0], " s[1]: ", Storage::adcValues[1], " s[2]: ", Storage::adcValues[2]);
                IO::outl<term>("m: ", meas::mRotation, " s: ", meas::mMechSection);
                IO::outl<term>("ds: ", (uint16_t)(1000 * driver::mScale));
            });
            if (const auto input = term::get()) {
                switch(*input) {
                case 'p':
                    Printer::start();
                    mState = State::Print;  
                break;
                case 's':
                    mState = State::Set;  
                break;
                case 'm':
                    mState = State::MeasureStart;  
                break;
                case 'i':
                    mState = State::IncrementMeasurementScale;  
                break;
                case 'd':
                    mState = State::DecrementMeasurementScale;  
                break;
                case 'o':
                    driver::scale(0.0);
                break;
                default:
                break;
                }
            }
        break;
        case State::Print:
            if (Printer::next()) {
                mState = State::Run;
            }
        break;
        case State::Set:
            mState = State::Run;
        break;
        case State::IncrementMeasurementScale:
            mState = State::Run;
        break;
        case State::DecrementMeasurementScale:
            mState = State::Run;
        break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                IO::outl<term>("> Undefined");
            break;
            case State::Init:
                IO::outl<term>("> Init");
            break;
            case State::Info:
                IO::outl<term>("> Info");
            break;
            case State::MeasureStart:
                IO::outl<term>("> MeasureStart");
                driver::scale(0.0);
                driver::all_on();
                pos::stop();
                devs::measurement::start();
            break;
            case State::MeasureRamp:
                IO::outl<term>("> MeasureRamp");
                driver::scale(mMeasScale);
                IO::outl<term>("ds: ", (uint16_t)(1000 * driver::mScale));
                devs::measurement::period(mMeasPeriod);
            break;
            case State::Measure:
                IO::outl<term>("> Measure");
            break;
            case State::MeasureStop:
                IO::outl<term>("> MeasureStop");
                devs::measurement::stop();
                driver::scale(0.0);
                driver::all_off();
            break;
            case State::Print:
                IO::outl<term>("> Print");
            break;
            case State::Run:
                IO::outl<term>("> Run");
                driver::all_on();
                pos::start();
            break;
            case State::Set:
                IO::outl<term>("> Set");
                driver::scale(mMeasScale);
//                driver::scale((1.0 * Storage::adcValues[0]) / 4095);
            break;
            case State::IncrementMeasurementScale:
                IO::outl<term>("> Inc");
                mMeasScale *= 1.5;
            break;
            case State::DecrementMeasurementScale:
                IO::outl<term>("> Dec");
                mMeasScale /= 1.5;
            break;
            }
        }
    }
//private:
    static inline float mMeasScale{0.1};
    static inline uint16_t mMeasPeriod{100}; // 10KHz
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline External::Tick<systemTimer> mMeasTick;
    static inline State mState{State::Undefined};
};

void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}

using devs = Devices<ESC_FOC_02, Config, Mcu::Stm::Stm32G431>;
using gfsm = GFSM<devs>;

int main() {
    gfsm::init();

    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    NVIC_EnableIRQ(TIM7_IRQn);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    __enable_irq();
    
    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

extern "C" {
#ifndef STM32G431xx
void TIM4_IRQHandler() __attribute__ ((isr));
void TIM6_DAC_IRQHandler() __attribute__ ((isr));
void TIM7_IRQHandler() __attribute__ ((isr));
void DMA1_Channel1_IRQHandler() __attribute__ ((isr));
#endif

void DMA1_Channel1_IRQHandler() {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}
void TIM4_IRQHandler()  {   
    devs::tp2::set();
    devs::hall::Interrupt::isr();    
    devs::tp2::reset();
}
void TIM6_DAC_IRQHandler()  {   
    devs::measurement::Interrupt::isr();
}
void TIM7_IRQHandler()  {   
//    devs::tp3::set();
    devs::pos_estimator::Interrupt::isr();
//    devs::tp3::reset();
}
}
