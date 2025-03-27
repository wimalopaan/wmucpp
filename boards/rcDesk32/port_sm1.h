#pragma once

#include <memory>
#include "port_iface.h"

template<typename Devs>
struct Smes1 {
    using debug = Devs::debug;
    using sm1 = Devs::sm1;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Smes1 ", a);
        switch(a) {
        case 0: // SM
        {
            mSm = nullptr;
            mSm = std::make_unique<Sm<sm1>>();
        }
            break;
        default:
            mSm = nullptr;
            break;

        }
    }
    static inline void periodic() {
        if (mSm) {
            mSm->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mSm) {
            mSm->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<ISm> mSm{};

};
