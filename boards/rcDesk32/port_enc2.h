#pragma once

#include <memory>
#include "port_iface.h"

template<typename Devs>
struct Enc2s {
    using debug = Devs::debug;
    using enc2 = Devs::enc2;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Enc2 ", a);
        switch(a) {
        case 0: // Enc2
        {
            mEnc = nullptr;
            mEnc = std::make_unique<Enc<enc2>>();
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
