#pragma once

#include <array>
#include <utility>
#include <etl/output.h>
#include <etl/meta.h>

template<typename Config>
struct SPortCommandCallback {
    using debug = Config::debug;
    using ledGroups = Config::ledGroups;
    using leds = Meta::nth_element<0, ledGroups>;

    static inline void decode(const std::array<std::byte, 7>& data) {
        const uint8_t address = (uint8_t)data[1];
        const uint8_t app = (uint8_t)data[2];
        const uint16_t state4 = (((uint16_t)data[4]) << 8) + (uint8_t)data[3];
        etl::outl<debug>("decode"_pgm, ", "_pgm, data[0], ", "_pgm, address, ", "_pgm, app, ", "_pgm, state4);

        if (address == mAddress) {
            uint16_t mask = 0b11;
            for(uint8_t i = 0; i < 8; ++i) {
                if ((state4 & mask) != 0) {
                    leds::set(i, true);
                }
                else {
                    leds::set(i, false);
                }
                mask <<= 2;
            }
        }
    }
    static inline void address(const uint8_t adr) {
        mAddress = adr;
    }
    static inline uint8_t address() {
        return mAddress;
    }
    private:
    static inline uint8_t mAddress = DEFAULT_ADDRESS;
};

