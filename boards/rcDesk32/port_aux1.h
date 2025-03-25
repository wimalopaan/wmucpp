#pragma once

#include <memory>

#include "port_iface.h"


template<typename Devs>
struct Auxes1 {
    using debug = Devs::debug;

#ifndef USE_SWD
    using hwext1 = Devs::hwext1;
#endif
    using sbus1 = Devs::sbus1;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Auxes1 ", a);
        switch(a) {
        case 0: //sbus
        {
            mAux = nullptr;
#ifndef USE_SWD
            mAux = std::make_unique<Aux<sbus1>>();
#endif
        }
            break;
        case 1: // hwext
        {
            mAux = nullptr;
            mAux = std::make_unique<Aux<hwext1>>();
        }
            break;
        default:
            mAux = nullptr;
            break;

        }
    }
    // static inline void update() {
    //     if (mAux) {
    //         mAux->update();
    //     }
    // }
    static inline void periodic() {
        if (mAux) {
            mAux->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mAux) {
            mAux->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<IAux> mAux{};

};
