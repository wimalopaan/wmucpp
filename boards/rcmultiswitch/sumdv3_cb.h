/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

enum class SumDV3FailsafeMode : uint8_t {Hold, Reset};

template<typename Config, uint16_t FailsafeTicks = 0, SumDV3FailsafeMode FailsafeMode = SumDV3FailsafeMode::Reset>
struct SumDV3CommandCallback {
    using debug = Config::debug;
    using allLeds = Config::allLeds;
    using tp = Config::tp;
    
    static inline constexpr std::array<uint8_t, 8> index2offset{1, 0, 3, 2, 5, 4, 7, 6};

    static inline void received(const std::byte status = 0_B) {
        if constexpr (FailsafeTicks > 0) {
            mReceiveTicks = 0;
        }
        if (std::any(status & 0x80_B)) {
            enterFailsafe();
        }
        else {
            mFailsafeActive = false;
        }
    }
    static inline void ratePeriodic() {
        if constexpr (FailsafeTicks > 0) {
            if (!mFailsafeActive && (++mReceiveTicks >= FailsafeTicks)) {
                enterFailsafe();
            }
        }
    }

    static inline void decode(const std::array<std::byte, 36>& data, const uint8_t length) {
        if (mFailsafeActive) {
            return;
        }
        if (length != data.size()) {
            etl::outl<debug>("sumd length: "_pgm, length);
            return;
        }
        const std::byte fcode = data[32];
        if (fcode == 0x05_B) {
            tp::toggle();
            ++mCommandPackagesCounter;
            const uint8_t offset = 24 + index2offset[mGroup]; // 24: start of switches
            const uint8_t state = (uint16_t)data[offset];
            const uint8_t changed = (state ^ mLastState);
            for(uint8_t i = 0; i < 8; ++i) {
                const uint8_t mask = (uint8_t(0x01) << i);
                if (changed & mask) {
                    const bool on = (state & mask);
                    etl::outl<debug>("ch: "_pgm, i, ": "_pgm, (uint8_t)on);
                    allLeds::set(i, on);
                }
            }
            mLastState = state;
        }
    }
    static inline void address(const uint8_t adr) {
        if (adr < index2offset.size()) {
            mGroup = adr;
        }
    }
    static inline uint8_t address() {
        return mGroup;
    }
    private:
    static inline void enterFailsafe() {
        if (mFailsafeActive) {
            return;
        }
        mFailsafeActive = true;
        if constexpr (FailsafeMode == SumDV3FailsafeMode::Reset) {
            allLeds::allOff();
            mLastState = 0;
            etl::outl<debug>("sumd failsafe reset"_pgm);
        }
        else {
            etl::outl<debug>("sumd failsafe hold"_pgm);
        }
    }

    static inline uint8_t mLastState = 0;
    static inline uint16_t mCommandPackagesCounter = 0;
    static inline uint8_t mGroup = DEFAULT_ADDRESS;
    static inline uint16_t mReceiveTicks = 0;
    static inline bool mFailsafeActive = false;
};
