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
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;
    
    using pwm1 = devs::pwm1;
    using crsf = devs::crsf;
    using crsf_pa = devs::crsf_pa;
    using crsf_out = devs::crsf_out;
    using telemetry = devs::telemetry;
    
    using led = devs::led;
    
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    
    enum class State : uint8_t {Undefined, Init, Run};
    
    static inline void init() {
        devs::init();
        devs::led::set();
    }   
    static inline void periodic() {
        trace::periodic();
        crsf::periodic();
    }
    static inline void ratePeriodic() {
        crsf_out::ratePeriodic();
        telemetry::ratePeriodic();
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
            });
        break;
        case State::Run:
            (++mDebugTick).on(debugTicks, []{
                led::toggle();
                IO::outl<trace>(
                            "b: ", crsf_pa::mBytesCounter, 
                            " p: ", crsf_pa::mPackagesCounter, 
                            " l: ", crsf_pa::mLinkPackagesCounter, 
                            " ch: ", crsf_pa::mChannelsPackagesCounter, 
                            " pg: ", crsf_pa::mPingPackagesCounter, 
                            " pe: ", crsf_pa::mParameterEntryPackagesCounter, 
                            " pr: ", crsf_pa::mParameterReadPackagesCounter, 
                            " pw: ", crsf_pa::mParameterWritePackagesCounter, 
                            " c: ", crsf_pa::mCommandPackagesCounter, 
                            " d: ", crsf_pa::mDataPackagesCounter); 
                IO::outl<trace>(
                            "ch0: ", crsf_pa::mChannels[0],
                            " ch1: ", crsf_pa::mChannels[1],
                            " ch2: ", crsf_pa::mChannels[2],
                            " ch3: ", crsf_pa::mChannels[3]
                        );
                
            });
        break;
        }
        if (oldState != mState) {
            mStateTick.reset();
        }
    }
private:
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState{State::Undefined};
};

void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}

int main() {
    using devs = Devices<CC01, Config, Mcu::Stm::Stm32G431>;
    using gfsm = GFSM<devs>;
    gfsm::init();
    
    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}
