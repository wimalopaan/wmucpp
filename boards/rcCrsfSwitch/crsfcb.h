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
    template<auto type, auto index>
    static inline void addForward(auto& p, const uint8_t parent, const char* const name) {
        addNode(p, Param_t{parent, PType::Sel, name, "Off;Forward;Tunnel", &eeprom.outputParams[index].telemetryMode[type], 0, 2, [](const uint8_t v){Meta::nth_element<index, crsf_ifaces>::forwardTelemetryMode(type, v); return true;}});
    }

    template<uint8_t index>
    static inline constexpr void addOutput(auto& p, const uint8_t parent, const char* const name) {
        const uint8_t parent2 = addParent(p, Param_t{parent, PType::Folder, name});
        addNode(p, Param_t{parent2, PType::Sel, "Fwd LinkStat Pkgs", "Off;On", &eeprom.outputParams[index].forwardLinkStats, 0, 1, [](const uint8_t v){Meta::nth_element<index, crsf_ifaces>::activateLinkStats(v > 0); return true;}});
        addNode(p, Param_t{parent2, PType::Sel, "Fwd RC-Channels Pkgs", "Off;On", &eeprom.outputParams[index].forwardRCChannels, 0, 1, [](const uint8_t v){Meta::nth_element<index, crsf_ifaces>::activateChannels(v > 0); return true;}});
        addNode(p, Param_t{parent2, PType::Sel, "Fwd BCast Pkgs", "Off;On", &eeprom.outputParams[index].forwardBCast, 0, 1, [](const uint8_t v){Meta::nth_element<index, crsf_ifaces>::activateBroadcast(v > 0); return true;}});

        addNode(p, Param_t{parent2, PType::Sel, "Relay", "Off;On", &eeprom.outputParams[index].asRelay, 0, 1, [](const uint8_t v){
                               if (v > 0) {
                                Meta::nth_element<index, crsf_ifaces>::insertRoute((uint8_t)RC::Protokoll::Crsf::V4::Address::RX);
                                Meta::nth_element<index, crsf_ifaces>::insertRoute((uint8_t)RC::Protokoll::Crsf::V4::Address::TX);
                                   }
                                return true;}});
        addNode(p, Param_t{parent2, PType::Sel, "Baudrate", "400k;420k", &eeprom.outputParams[index].baudrate, 0, 1, [](const uint8_t v){Meta::nth_element<index, crsf_ifaces>::baud((v == 0) ? 400'000 : 420'000); return true;}});

        addNode(p, Param_t{parent2, PType::Str, "TX Rewrite Name", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &eeprom.outputParams[index].tx_name[0]});
        addNode(p, Param_t{parent2, PType::Sel, "Rewrite Name", "Off;On", &eeprom.outputParams[index].rewrite_name, 0, 1, [](const uint8_t v){Meta::nth_element<index, crsf_ifaces>::rewriteName(v == 1); return true;}});

        addNode(p, Param_t{parent2, PType::Sel, "LinkStat", "Off;Forward;Transform;Tunnel", &eeprom.outputParams[index].link_stat_mode, 0, 3, [](const uint8_t v){Meta::nth_element<index, crsf_ifaces>::linkStatMode(v); return true;}});
        addNode(p, Param_t{parent2, PType::Sel, "Failsafe", "Forward;Hold", &eeprom.outputParams[index].failsafe_mode, 0, 1, [](const uint8_t){return true;}});

        if constexpr((index != 1) && (index < 5)) {
            const uint8_t parent3 = addParent(p, Param_t{parent2, PType::Folder, "Telemetry forwarding"});
            addNode(p, Param_t{parent3, PType::U8,  "Forward Rate", nullptr, &eeprom.outputParams[index].telemetry_rate, 1, 10, [](const uint8_t a){Meta::nth_element<index, crsf_ifaces>::telemetryRate(a); return true;}});
            addForward<0x02, index>(p, parent3, "GPS");
            addForward<0x03, index>(p, parent3, "GPS Time");
            addForward<0x06, index>(p, parent3, "GPS Ext");
            addForward<0x07, index>(p, parent3, "Vario");
            addForward<0x08, index>(p, parent3, "Battery");
            addForward<0x09, index>(p, parent3, "Barometer");
            addForward<0x0a, index>(p, parent3, "Airspeed");
            addForward<0x0b, index>(p, parent3, "Heartbeat");
            addForward<0x0c, index>(p, parent3, "RPM");
            addForward<0x0d, index>(p, parent3, "Temperature");
            addForward<0x0e, index>(p, parent3, "Voltages");
            addForward<0x10, index>(p, parent3, "VTX Telemetry");
            addForward<0x1c, index>(p, parent3, "Link RX");
            addForward<0x1d, index>(p, parent3, "Link TX");
            addForward<0x1e, index>(p, parent3, "Attitude");
            addForward<0x1f, index>(p, parent3, "MAVLink FC");
            addForward<0x21, index>(p, parent3, "Flight Mode");
            addForward<0x22, index>(p, parent3, "ESP Now");
        }
        if constexpr(!Meta::nth_element<index, crsf_ifaces>::halfDuplex) {
#ifdef USE_IRDA
            addNode(p, Param_t{parent2, PType::Sel, "IrDA", "Off;On", &eeprom.outputParams[index].irda, 0, 1, [](const uint8_t v){
                                   if (v == 0) {
                                       Meta::nth_element<index, crsf_ifaces>::baud(RC::Protokoll::Crsf::V4::baudrate);
                                       Meta::nth_element<index, crsf_ifaces>::setIrDA(false);
                                       Meta::nth_element<index, crsf_ifaces>::updateRate(10ms);
                                   }
                                   else {
                                       Meta::nth_element<index, crsf_ifaces>::updateRate(40ms);
#ifdef USE_IRDA_TX_INVERT
                                       Meta::nth_element<index, crsf_ifaces>::setIrDA(true, true);
#else
                                       Meta::nth_element<index, crsf_ifaces>::setIrDA(true);
#endif
                                   }
                                   return true;}});
#endif
        }
    }
    using params_t = etl::FixedVector<Param_t, 254>;
    static inline params_t params = [] {
        params_t p;
        addNode(p, Param_t{0, PType::Folder, "root"});
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::U8,  "CRSF Address", nullptr, &eeprom.address, 192, 207, [](const uint8_t a){updateName(mName); src::address(std::byte(a)); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Command B-Address", nullptr, &eeprom.commandBroadcastAddress, 192, 207, [](const uint8_t){return true;}});
        addOutput<0>(p, 0, "Port HD1");
        addOutput<1>(p, 0, "Port HD2");
        addOutput<2>(p, 0, "Port FD1");
        addOutput<3>(p, 0, "Port FD2");
        addOutput<4>(p, 0, "Port FD3");
        addOutput<5>(p, 0, "Port FD4");

        if (p.size() >= p.capacity()) {
            void fp();
            fp(); // compile-time check (call to undefined function)
        }

        return p;
    }();
};
