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
    using relay = Config::relay;
    using src = Config::src;

    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;
    static inline constexpr const char* const title = "RC-Relay32@";

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
    static inline constexpr void forwardPacket(volatile uint8_t* const data, const uint16_t length) {
        relay::forwardPacket(data, length);
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
    static inline uint8_t mPulsesCalibCommand;

    static inline std::array<const char*, 6> mPulsesCalibTexts {
        "Calibrate",
        "Start ...",
        "Calib: move all sticks/pots ...",
        "Finished calibrating"
    };

    // Step         Result(bool)    Next-Step       Action
    // Click        false           Executing       start task
    //              true            AskConfirm
    // Confirmed    false           Executing       start task
    // Cancel       false                           cancel task
    // Update       false           Idle
    //              true            Executing

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
    template<auto type>
    static inline void addForward(auto& p, const uint8_t parent, const char* const name) {
        addNode(p, Param_t{parent, PType::Sel, name, "Off;Forward;Tunnel", &eeprom.telemetryMode[type], 0, 2, [](const uint8_t v){relay::forwardTelemetryMode(type, v); return true;}});
    }
    using params_t = etl::FixedVector<Param_t, 32>;
    static inline params_t params = [] {
        params_t p;
        addNode(p, Param_t{0, PType::Folder, "root"});
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::U8,  "CRSF Address", nullptr, &eeprom.address, 192, 207, [](const uint8_t a){
                               updateName(mName); src::address(std::byte(a));
                    return true;}});

        addNode(p, Param_t{0, PType::U8,  "TX Rewrite Address", nullptr, &eeprom.tx_rewrite_address, 192, 207, [](const uint8_t a){relay::txAddress(a); return true;}});
        addNode(p, Param_t{0, PType::U8,  "RX Rewrite Address", nullptr, &eeprom.rx_rewrite_address, 192, 207, [](const uint8_t a){relay::rxAddress(a); return true;}});
        addNode(p, Param_t{0, PType::Str, "TX Rewrite Name", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &eeprom.txname[0]});
		addNode(p, Param_t{0, PType::Sel, "Rewrite Name", "Off;On", &eeprom.rewrite_name, 0, 1, [](const uint8_t v){relay::rewriteName(v == 1); return true;}});

        addNode(p, Param_t{0, PType::Sel, "LinkStat", "Off;Forward;Transform;Tunnel", &eeprom.link_stat_mode, 0, 3, [](const uint8_t v){relay::linkStatMode(v); return true;}});
        addNode(p, Param_t{0, PType::Sel, "Failsafe", "Forward;Hold", &eeprom.failsafe_mode, 0, 1, [](const uint8_t){return true;}});

        const uint8_t parent = addParent(p, Param_t{0, PType::Folder, "Telemetry forwarding"});
        addNode(p, Param_t{parent, PType::U8,  "Forward Rate", nullptr, &eeprom.telemetry_rate, 1, 10, [](const uint8_t a){relay::telemetryRate(a); return true;}});
        addForward<0x02>(p, parent, "GPS");
        addForward<0x03>(p, parent, "GPS Time");
        addForward<0x06>(p, parent, "GPS Ext");
        addForward<0x07>(p, parent, "Vario");
        addForward<0x08>(p, parent, "Battery");
        addForward<0x09>(p, parent, "Barometer");
        addForward<0x0a>(p, parent, "Airspeed");
        addForward<0x0b>(p, parent, "Heartbeat");
        addForward<0x0c>(p, parent, "RPM");
        addForward<0x0d>(p, parent, "Temperature");
        addForward<0x0e>(p, parent, "Voltages");
        addForward<0x10>(p, parent, "VTX Telemetry");
        addForward<0x1c>(p, parent, "Link RX");
        addForward<0x1d>(p, parent, "Link TX");
        addForward<0x1e>(p, parent, "Attitude");
        addForward<0x1f>(p, parent, "MAVLink FC");
        addForward<0x21>(p, parent, "Flight Mode");
        addForward<0x22>(p, parent, "ESP Now");

        addNode(p, Param_t{0, PType::Sel, "Half-Duplex", "Off;On", &eeprom.half_duplex, 0, 1, [](const uint8_t v){relay::setHalfDuplex(v == 1); return true;}});

        // detects overflow by calling undefined function f();
        if (p.size() >= p.capacity()) {
            void f();
            f();
        }
        return p;
    }();
};

