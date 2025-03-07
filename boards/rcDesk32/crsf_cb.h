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

#include "mcu/eeprom.h"
#include "meta.h"
#include "eeprom.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"

template<typename Config, typename Debug = void>
struct CrsfCallback {
    using debug = Debug;
    using timer = Config::timer;
    using tp = Config::tp;
    using storage = Config::storage;
    using Param_t = RC::Protokoll::Crsf::V4::Parameter<uint8_t>;
    using PType = Param_t::Type;
    using src = Config::src;

    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;
    static inline constexpr const char* const title = "RC-Desk32@";

    using name_t = std::array<char, 32>;

    static inline constexpr void disableTelemetry() {
    }
    static inline constexpr void gotLinkStats() {
    }
    static inline constexpr void gotChannels() {
    }
    static inline constexpr void forwardPacket(const auto /*data*/, const uint16_t /*length*/) {
    }
    static inline constexpr void ratePeriodic() {
    }
    static inline constexpr void updateName(name_t& n) {
        strncpy(&n[0], title, n.size());
        auto r = std::to_chars(std::begin(n) + strlen(title), std::end(n), eeprom.address);
        *r.ptr++ = '\0';
    }

    static inline void update() {
        updateName(mName);
    }

    static inline void save() {
        if (auto [ok, err] = Mcu::Stm32::savecfg(eeprom, eeprom_flash); ok) {
            IO::outl<debug>("# EEPROM OK");
        }
        else {
            IO::outl<debug>("# EEPROM NOK: ", err);
        }
    }
    static inline void setParameterValue(const uint8_t index, const auto data, const uint8_t paylength) {
        IO::outl<debug>("# SetPV i: ", index, " size: ", params.size());
        if (index == 0) return;
        if (index < params.size()) {
            mLastChangedParameter = index;
            bool mustSave = true;
            if (params[index].type == Param_t::Str) {
                IO::outl<debug>("# String");
                if (params[index].stringValue) {
                    for(uint8_t i = 0; (i < 16) && (i < paylength); ++i) {
                        params[index].stringValue[i] = data[i];
                        if (data[i] == '\0') {
                            break;
                        }
                    }
                    params[index].stringValue[paylength] = '\0';
                }
            }
            else {
                Param_t::value_type value{};
                if (params[index].type <= Param_t::I8) {
                    value = data[0];
                    IO::outl<debug>("# I8: v: ", value);
                }
                else if (params[index].type <= Param_t::I16) {
                    value = (data[0] << 8) + data[1];
                    IO::outl<debug>("# I16: v: ", value);

                }
                else if (params[index].type <= Param_t::F32) {
                    value = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];
                    IO::outl<debug>("# F32: v: ", value);
                }
                else if (params[index].type == Param_t::Sel) {
                    value = data[0];
                    IO::outl<debug>("# Sel: v: ", value);
                }
                params[index].value(value);
                if (params[index].cb) {
                    mustSave = params[index].cb(value);
                }
            }
            update();
            if (mustSave) {
                save();
            }
        }
    }
    static inline Param_t parameter(const uint8_t index) {
        if (index < params.size()) {
            return params[index];
        }
        return {};
    }
    static inline bool isCommand(const uint8_t index) {
        if (index < params.size()) {
            return params[index].type == PType::Command;
        }
        return false;
    }
    static inline const char* name() {
        return &mName[0];
    }
    static inline uint32_t serialNumber() {
        return mSerialNumber;
    }
    static inline uint32_t hwVersion() {
        return mHWVersion;
    }
    static inline uint32_t swVersion() {
        return mSWVersion;
    }
    static inline uint8_t numberOfParameters() {
        return params.size() - 1;
    }
    static inline uint8_t protocolVersion() {
        return 0;
    }
    static inline void whenParameterChanged(auto f) {
    }
    static inline void command(const auto& data, const uint8_t /*payload*/) {
    }
    static inline void callbacks(const bool eepromMode = false) {
        const bool prevMode = mEepromMode;
        mEepromMode = eepromMode;
        for(const auto& p: params) {
            if (p.type != PType::Command) {
                if (p.cb) {
                    p.cb(p.value());
                }
            }
        }
        mEepromMode = prevMode;
    }
    static inline void serialize(const uint8_t index, auto& buffer, const RC::Protokoll::Crsf::V4::Lua::CmdStep step = RC::Protokoll::Crsf::V4::Lua::CmdStep::Idle) {
        params[index].serialize(buffer, params, step, index);
    }
private:
    static inline name_t mName = []{
        name_t name{};
        updateName(name);
        return name;
    }();

    static inline bool mEepromMode = false;
    static inline uint8_t mLastChangedParameter{};
    static inline constexpr uint32_t mSerialNumber{1234};
    static inline constexpr uint32_t mHWVersion{HW_VERSION};
    static inline constexpr uint32_t mSWVersion{SW_VERSION};
    static inline constexpr auto mVersionString = [](){
        std::array<char, 16> s{};
        auto [ptr, e] = std::to_chars(std::begin(s), std::end(s), mHWVersion);
        *ptr++ = ':';
        auto r = std::to_chars(ptr, std::end(s), mSWVersion);
        *r.ptr = '\0';
        return s;
    }();
    template<typename T>
    static inline uint8_t addParent(auto& c, const RC::Protokoll::Crsf::V4::Parameter<T>& p) {
        c.push_back(p);
        return c.size() - 1;
    }
    template<typename T>
    static inline void addNode(auto& c, const RC::Protokoll::Crsf::V4::Parameter<T>& p) {
        c.push_back(p);
    }
    using params_t = etl::FixedVector<Param_t, 32>;
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Folder, ""});
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::U8,  "CRSF Address", nullptr, &eeprom.address, 192, 207, [](const uint8_t a){updateName(mName); src::address(std::byte(a)); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Controller Number", nullptr, &eeprom.controllerNumber, 0, 8, [](const uint8_t){return true;}});
        addNode(p, Param_t{0, PType::Sel, "Bluetooth", "off;on", &eeprom.bluetooth, 0, 1, [](const uint8_t){return true;}});
        addNode(p, Param_t{0, PType::Sel, "CRSF Channels", "off;on", &eeprom.crsf_in, 0, 1, [](const uint8_t){return true;}});
        auto parent = addParent(p, Param_t{0, PType::Folder, "I2C"});
        addNode(p, Param_t{parent, PType::Command, "I2C scan", "Scanning...", nullptr, 0, 0, [](const uint8_t){return false;}});
        parent = addParent(p, Param_t{0, PType::Folder, "Devices"});
        return p;
    }();
};

