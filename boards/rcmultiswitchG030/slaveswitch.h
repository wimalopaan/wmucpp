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

template<typename Config>
struct SlaveSwitchProtocol {
    using buffer = Config::messagebuffer;
    using storage = Config::storage;

    static inline constexpr void set(const uint8_t p) {
        mState |= (1 << p);
        sendState();
    }
    static inline constexpr void reset(const uint8_t p) {
        mState &= ~(1 << p);
        sendState();
    }
    static inline void active(const bool a) {
        mActive = a;
    }
private:
    static inline void sendState() {
        if constexpr(!std::is_same_v<buffer, void>) {
            if (mActive) {
                buffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::Command, [&](auto& d){
                    d.push_back(RC::Protokoll::Crsf::V4::Address::Broadcast);
                    d.push_back(storage::eeprom.crsf_address);
                    d.push_back(RC::Protokoll::Crsf::V4::CommandType::Switch);
                    d.push_back(RC::Protokoll::Crsf::V4::SwitchCommand::InterModuleSlaveSet);
                    d.push_back(storage::eeprom.addresses[0]); // master switch address
                    d.push_back(mState);
                });
            }
        }
    }
    static inline uint8_t mState = 0;
    static inline bool mActive = false;
};

