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

namespace Crsf {
    template<typename Config>
    struct Command {
        using callback = Config::callback;
        using debug = Config::debug;

        static inline bool decode(const std::byte type, const auto& data) {
            if (type != Crsf::Type::Command) {
                return false;
            }
            ++mCommandPackagesCounter;
            if ((data[0] == Crsf::Address::Controller) || (data[0] == Crsf::Address::Broadcast)) {
                if (data[2] == Crsf::CommandType::Switch) {
                    const std::byte command = data[3];
                    if (command == Crsf::SwitchCommand::Set) {
                        if (data[4] == mModuleAddress) {
                            etl::outl<debug>("set: "_pgm, data[5]);
                            callback::set(data[5]);
                            return true;
                        }
                    }
                    else if (command == Crsf::SwitchCommand::Set4) {
                        const uint8_t address = (uint8_t)data[4];
                        const int8_t adrIndex = address - (uint8_t)mModuleAddress;
                        const uint16_t sw = (((uint16_t)data[5]) << 8) + (uint8_t)data[6];
                        etl::outl<debug>("set4: "_pgm, address);
                        for(uint8_t i = 0; i < 8; ++i) {
                            const uint8_t s = (sw >> (2 * i)) & 0b11;
                            callback::setIndex(adrIndex, i, (s > 0));
                        }
                        return true;
                    }
                    else if (command == Crsf::SwitchCommand::Set4M) {
                        const uint8_t count = (uint8_t)data[4];
                        for(uint8_t i = 0; i < count; ++i) {
                            const uint8_t address = (uint8_t)data[5 + 3 * i];
                            const uint16_t sw = ((uint16_t)data[6 + 3 * i] << 8) + (uint8_t)data[7 + 3 * i];
                            etl::outl<debug>("set4M: "_pgm, i, " adr: "_pgm, address);
                            const uint8_t adrIndex = address - (uint8_t)mModuleAddress;
                            for(uint8_t k = 0; k < 8; ++k) {
                                const uint8_t s = (sw >> (2 * k)) & 0b11;
                                callback::setIndex(adrIndex, k, s > 0);
                            }
                        }
                        return true;
                    }
                }
            }
            return false;
        }
        template<bool Reset = true>
        static inline uint16_t commandPackages() {
            if constexpr(Reset) {
                const auto c = mCommandPackagesCounter;
                mCommandPackagesCounter = 0;
                return c;
            }
            else {
                return mCommandPackagesCounter;
            }
        }
        static inline void address(const uint8_t adr) {
            mModuleAddress = (std::byte)adr;
        }
        static inline uint8_t address() {
            return (uint8_t)mModuleAddress;
        }
    private:
        static inline uint16_t mCommandPackagesCounter{};
        inline static std::byte mModuleAddress{DEFAULT_ADDRESS};
    };
}
