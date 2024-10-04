#define USE_MCU_STM_V3
#define USE_CRSF_V2
#define USE_USART_DMA

#define NDEBUG

#include <cstdint>
#include <chrono>

#include "devices.h"

using namespace std::literals::chrono_literals;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;

    static inline void init() {
        devs::init();
    }
    static inline void periodic() {

    }
    static inline void ratePeriodic() {

    }
};

struct DevsConfig {

};

using devs = Devices<SW01, DevsConfig, Mcu::Stm::Stm32G0B1>;
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
