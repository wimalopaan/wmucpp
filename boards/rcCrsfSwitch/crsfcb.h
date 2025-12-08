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

template<typename Config>
struct CrsfCallback {
    using debug = Config::debug;
    using timer = Config::timer;
    using Param_t = RC::Protokoll::Crsf::V4::Parameter<uint8_t>;
    using PType = Param_t::Type;
    using src = Config::src;

    using crsf_ifaces = Config::crsf_ifaces;

    using storage = Config::storage;
    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;
    static inline constexpr const char* const title = "CrsfSwitch@";

    using name_t = std::array<char, 32>;

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
            if (RC::Protokoll::Crsf::V4::Util::setParameter(params, index, data, paylength)) {
                save();
            }
            update();
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
    static inline void command(const auto& /*data*/, const uint8_t /*payload*/) {
    }
    static inline constexpr void forwardPacket(volatile uint8_t* const data, const uint16_t length) {
        Meta::visit<crsf_ifaces>([&](auto P) {
			decltype(P)::type::forwardPacket(data, length);
		});
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
        if (index < params.size()) {
            params[index].serialize(buffer, params, step, index);
        }
    }

    private:
    static inline name_t mName = []{
        name_t name{};
        updateName(name);
        return name;
    }();

    static inline bool mEepromMode = false;
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

	template<uint8_t index>
    static inline constexpr void addOutput(auto& p, const uint8_t parent, const char* const name) {
        const uint8_t parent2 = addParent(p, Param_t{parent, PType::Folder, name});
		addNode(p, Param_t{parent2, PType::Sel, "Fwd LinkStat Pkgs", "Off;On", &eeprom.forwardLinkStats[index], 0, 1, [](const uint8_t v){Meta::nth_element<index, crsf_ifaces>::activateLinkStats(v > 0); return false;}});
		addNode(p, Param_t{parent2, PType::Sel, "Fwd RC-Channels Pkgs", "Off;On", &eeprom.forwardRCChannels[index], 0, 1, [](const uint8_t v){Meta::nth_element<index, crsf_ifaces>::activateChannels(v > 0); return false;}});
		addNode(p, Param_t{parent2, PType::Sel, "Fwd BCast Pkgs", "Off;On", &eeprom.forwardBCast[index], 0, 1, [](const uint8_t v){Meta::nth_element<index, crsf_ifaces>::activateBroadcast(v > 0); return false;}});
	}
    using params_t = etl::FixedVector<Param_t, 128>;
    static inline params_t params = [] {
        params_t p;
        addNode(p, Param_t{0, PType::Folder, "root"});
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::U8,  "CRSF Address", nullptr, &eeprom.address, 192, 207, [](const uint8_t a){updateName(mName); src::address(std::byte(a)); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Command B-Address", nullptr, &eeprom.commandBroadcastAddress, 192, 207, [](const uint8_t){return true;}});
		addOutput<0>(p, 0, "Port HD1");
		addOutput<0>(p, 0, "Port HD2");
		addOutput<0>(p, 0, "Port FD1");
		addOutput<0>(p, 0, "Port FD2");
		addOutput<0>(p, 0, "Port FD3");
		addOutput<0>(p, 0, "Port FD4");
        return p;
    }();
};
