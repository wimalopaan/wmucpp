#pragma once

#include <memory>
#include "port_iface.h"

template<typename Devs>
struct Enc1s {
    using debug = Devs::debug;
    using enc1 = Devs::enc1;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Enc1 ", a);
        switch(a) {
        case 0: // Enc1
        {
            mEnc = nullptr;
            mEnc = std::make_unique<Enc<enc1>>();
        }
            break;
        default:
            mEnc = nullptr;
            break;

        }
    }
    static inline void periodic() {
        if (mEnc) {
            mEnc->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mEnc) {
            mEnc->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<IEnc> mEnc{};
};
