#pragma once

#include <cstdint>
#include <cstdlib>
#include <numbers>

#include "fastmath.h"
#include "util.h"

template<uint8_t N, typename PA, typename NVM>
struct Polar {
    static inline constexpr uint8_t number = N;
    using pa = PA;
    using storage = NVM;
    static inline auto& eeprom = storage::eeprom;

    static inline void update() {
        const int v1 = pa::values()[eeprom.channels[number].first] - pa::mid;
        const int v2 = pa::values()[eeprom.channels[number].second] - pa::mid;
        const int amin = std::min(std::abs(v1), std::abs(v2));

        mPhi = FastMath::uatan2<820, 4096>(v1, v2);
        const unsigned amp = FastMath::usqrt4(v1 * v1 + v2 * v2);
        const unsigned amax = FastMath::usqrt4(amin * amin + pa::amp * pa::amp); // 96us
        mAmp = (pa::amp * amp) / amax;
        if (mAmp < eeprom.deadbands[N]) {
            mDead = true;
        }
        else {
            mDead = false;
        }
    }
    static inline uint16_t phi() {
        const uint16_t p = mPhi + mOffset;
        if (mDead) {
            return mLastPhi;
        }
        else {
            return mLastPhi = normalize<4096>(p);
        }
    }
    static inline uint16_t amp() {
        if (mDead) {
            return 0;
        }
        return mAmp;
    }
    static inline void offset(const uint16_t o) {
        mOffset = o;
    }
    static inline uint16_t offset() {
        return mOffset;
    }
    private:
    static inline bool mDead = true;
    static inline uint16_t mLastPhi = 0;
    static inline uint16_t mPhi = 0;
    static inline uint16_t mAmp = 0;
    static inline uint16_t mOffset = 0;
};