#pragma once

#include <memory>
#include "port_iface.h"

template<typename Devs>
struct Smes2 {
    using debug = Devs::debug;
    using sm2 = Devs::sm2;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Smes2 ", a);
        switch(a) {
        case 0: // SM
        {
            mSm = nullptr;
            mSm = std::make_unique<Sm<sm2>>();
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
