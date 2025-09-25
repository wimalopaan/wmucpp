/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>
#include <array>

#include "output.h"

template<typename Config>
struct Router {
    using storage = Config::storage;
    using debug = Config::debug;

    static inline uint8_t forwardDestAddress(const uint8_t id, const uint8_t a) {
        if (a == 0) {
            return 0;
        }
        for(const auto& e : mMap) {
            if ((a == e.rew) && (id == e.id)) {
                return e.orig;
            }
        }
        return 0;
    }
    static inline std::pair<uint8_t, uint8_t> entry(const uint8_t id, const uint8_t a) {
        for(const auto& e : mMap) {
            if ((a == e.orig) && (id == e.id)) {
                return {e.id, e.rew};
            }
        }
        return {};
    }
    static inline int8_t freeIndex() {
        for(uint8_t i = 0; i < mMap.size(); ++i) {
            if (mMap[i].id == 0) {
                return i;
            }
        }
        return -1;
    }
    static inline bool hasRewriteAddress(const uint8_t rew) {
        if (rew == storage::eeprom.address) { //exclude own address (crsf-switch address)
            return true;
        }
        for(const auto& e : mMap) {
            if (e.rew == rew) {
                return true;
            }
        }
        return false;
    }
    static inline uint8_t createRewriteAddress(const uint8_t a) {
        if (!hasRewriteAddress(a)) {
            return a;
        }
        for(uint8_t rew = minRewriteAddress; rew <= maxRewriteAddress; ++rew) {
            if (!hasRewriteAddress(rew)) {
                return rew;
            }
        }
        overrun = true;
        return a;
    }
    static inline uint8_t insertRewriteEntry(const uint8_t id, const uint8_t a) {
        const int8_t nextFreeIndex = freeIndex();
        if (nextFreeIndex < 0) {
            overrun = true;
            return a;
        }
        mMap[nextFreeIndex].id = id;
        mMap[nextFreeIndex].orig = a;
        mMap[nextFreeIndex].rew = createRewriteAddress(a);

        IO::outl<debug>("# Router i: ", id, " ", a, " ", mMap[nextFreeIndex].rew);

        return mMap[nextFreeIndex].rew;
    }
    static inline uint8_t backwardSrcAddress(const uint8_t id, const uint8_t a) {
        const auto e = entry(id, a);
        IO::outl<debug>("# router bw e: ", e.first, " ", e.second);
        if (e.first == 0) { // not present or present at other interface
            return insertRewriteEntry(id, a);
        }
        else { // found on this interface
            return e.second;
        }
    }
    // private:
    static inline bool overrun = false;
    static inline uint8_t minRewriteAddress = 0xc0;
    static inline uint8_t maxRewriteAddress = 0xcf;
    struct Entry {
        uint8_t id   = 0;
        uint8_t orig = 0;
        uint8_t rew  = 0;
    };
    static inline std::array<Entry, 16> mMap{};
};
