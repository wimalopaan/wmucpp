#pragma once

#include <memory>
#include "port_iface.h"

template<typename Devs>
struct Busses {
    using debug = Devs::debug;
    using bus_crsf = Devs::bus_crsf;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Bus ", a);
        switch(a) {
        case 0: // crsf
        {
            mBus = nullptr;
            mBus = std::make_unique<Bus<bus_crsf>>();
            bus_crsf::baud(400'000);
        }
            break;
        default:
            mBus = nullptr;
            break;

        }
    }
    static inline void periodic() {
        if (mBus) {
            mBus->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mBus) {
            mBus->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<IBus> mBus{};
};
