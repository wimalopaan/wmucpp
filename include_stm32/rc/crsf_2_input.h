/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <cstddef>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <chrono>
#include <cstring>
#include <type_traits>

#include "rc_2.h"

namespace RC {
    namespace Protokoll {
        namespace Crsf {
            namespace V4 {
                using namespace std::literals::chrono_literals;
                using namespace etl::literals;

                template<typename Master>
                struct Input {
                    friend Master;
                    using values_t = std::array<uint16_t, 16>;

                    static inline constexpr uint16_t mid = RC::Protokoll::Crsf::V4::mid;
                    static inline constexpr uint16_t amp = RC::Protokoll::Crsf::V4::amp;

                    static inline void init() {
                        for(auto& c : mChannels) {
                            c = mid;
                        }
                    }
                    static inline const auto& values() {
                        return mChannels;
                    }
                    static inline uint16_t value(const uint8_t ch) {
                        if (ch < mChannels.size()) {
                            return mChannels[ch];
                        }
                        return RC::Protokoll::Crsf::V4::mid;
                    }
                    template<bool Reset = false>
                    static inline uint16_t pingPackages() {
                        if constexpr(Reset) {
                            const auto v = mPingPackagesCounter;
                            mPingPackagesCounter = 0;
                            return v;
                        }
                        else {
                            return mPingPackagesCounter;
                        }
                    }
                    template<bool Reset = false>
                    static inline uint16_t linkPackages() {
                        if constexpr(Reset) {
                            const auto v = mLinkPackagesCounter;
                            mLinkPackagesCounter = 0;
                            return v;
                        }
                        else {
                            return mLinkPackagesCounter;
                        }
                    }
                    template<bool Reset = false>
                    static inline uint16_t channelPackages() {
                        if constexpr(Reset) {
                            const auto v = mChannelsPackagesCounter;
                            mChannelsPackagesCounter = 0;
                            return v;
                        }
                        else {
                            return mChannelsPackagesCounter;
                        }
                    }
                    private:
                    static inline void decodePing(auto) {
                        ++mPingPackagesCounter;
                    }
                    static inline void decodeLink(auto) {
                        ++mLinkPackagesCounter;
                    }
                    static inline void decodeChannels(auto payload) {
                        ++mChannelsPackagesCounter;
                        std::array<volatile uint8_t, 22>* d = (std::array<volatile uint8_t, 22>*)payload;
                        Channels ch = std::bit_cast<Channels>(*d);
                        mChannels[0] = ch.ch0;
                        mChannels[1] = ch.ch1;
                        mChannels[2] = ch.ch2;
                        mChannels[3] = ch.ch3;
                        mChannels[4] = ch.ch4;
                        mChannels[5] = ch.ch5;
                        mChannels[6] = ch.ch6;
                        mChannels[7] = ch.ch7;
                        mChannels[8] = ch.ch8;
                        mChannels[9] = ch.ch9;
                        mChannels[10] = ch.ch10;
                        mChannels[11] = ch.ch11;
                        mChannels[12] = ch.ch12;
                        mChannels[13] = ch.ch13;
                        mChannels[14] = ch.ch14;
                        mChannels[15] = ch.ch15;
                    }
                    static inline uint16_t mPingPackagesCounter{};
                    static inline uint16_t mLinkPackagesCounter{};
                    static inline uint16_t mChannelsPackagesCounter{};
                    inline static values_t mChannels;
                };
            }
        }
    }
}
