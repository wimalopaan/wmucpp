#define USE_MCU_STM_V3
#define USE_CRSF_V2

#define NDEBUG

#include <cstdint>
#include <array>

#include "devices.h"

using namespace std::literals::chrono_literals;


template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using crsf = devs::crsf;
    using debug = devs::debug;

    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};

    static inline void init() {
        devs::init();
    }
    static inline void periodic() {
        crsf::periodic();
        debug::periodic();
    }
    static inline void ratePeriodic() {
        (++mDebugTick).on(debugTicks, []{
            IO::outl<debug>("bc: ", (uint8_t)crsf::mCounter);
            IO::outl<debug>("isr: ", (uint8_t)crsf::mcuUart->ISR & 0x0f);
            IO::outl<debug>("cr1: ", (uint8_t)crsf::mcuUart->CR1 & 0x0f);
        });
    }
    private:
    static inline External::Tick<systemTimer> mDebugTick;
};

template<typename, typename> struct Dummy{};

using devs = Devices2<SW99, Dummy>;
using gfsm = GFSM<devs>;

int main() {
    gfsm::init();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

