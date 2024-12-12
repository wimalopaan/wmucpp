#pragma once

#include "output.h"
#include "iservo.h"

template<typename Devs>
struct Auxes {
    using devs = Devs;
    using debug = devs::debug;
    using relay = devs::relay_aux;
    // using gps = devs::gps;

    static inline void set(const uint8_t r) {
        IO::outl<debug>("# aux ", r);
        switch(r) {
        case 0: // crsf
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<relay>>();
            break;
        case 1: // gps
            mRelay = nullptr;
            break;
        case 2: // none
            mRelay = nullptr;
            break;
        default:
            break;
        }
    }
    static inline constexpr void forwardPacket(const std::byte type, const std::array<uint8_t, 64>& data, const uint16_t length) {
        if (mRelay) {
            mRelay->forwardPacket(type, data, length);
        }
    }
    static inline void command(const std::array<uint8_t, 64>& data, const uint16_t length){
        if (mRelay) {
            mRelay->command(data, length);
        }
    }
    static inline void ping() {
        if (mRelay) {
            mRelay->ping();
        }
    }
    static inline void update() {
        if (mRelay) {
            mRelay->update();
        }
    }
    static inline void periodic() {
        if (mRelay) {
            mRelay->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mRelay) {
            mRelay->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<IRelay> mRelay{};
};
