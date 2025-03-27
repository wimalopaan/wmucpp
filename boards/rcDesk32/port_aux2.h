#pragma once

#include <memory>

#include "port_iface.h"

template<typename Devs>
struct Auxes2 {
    using debug = Devs::debug;
    using sbus2 = Devs::sbus2;
    using hwext2 = Devs::hwext2;

    static inline void set(const uint8_t a) {
        IO::outl<debug>("# Auxes2 ", a);
        switch(a) {
        case 0: // sbus
        {
            mAux = nullptr;
            mAux = std::make_unique<Aux<sbus2>>();
        }
            break;
        case 1: // inv sbus
        {
            mAux = nullptr;
            mAux = std::make_unique<Aux<sbus2>>();
            sbus2::invert(false);
        }
            break;
        case 2: // hwext
        {
            mAux = nullptr;
            mAux = std::make_unique<Aux<hwext2>>();
        }
            break;
        default:
            mAux = nullptr;
            break;

        }
    }
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
