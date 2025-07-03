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

    using auxes1 = Config::auxes1;
    using auxes2 = Config::auxes2;

    using smes1 = Config::smes1;
    using smes2 = Config::smes2;

    using encs1 = Config::encs1;
    using encs2 = Config::encs2;

    using busses = Config::busses;
    using forwarder = Config::forward;

    using pulse_in = Config::pulse_in;

    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;
    static inline constexpr const char* const title = "RC-Desk32@";

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
    static inline constexpr void forwardPacket(auto& data, const uint16_t length) {
        busses::forwardPacket(data, length);
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
    static inline void setI2CDev(const uint8_t bus, const uint8_t dev, const uint8_t adr) {
        if (bus < mI2CDevs.size()) {
            if (dev < mI2CDevs[bus].size()) {
                auto r = std::to_chars(std::begin(mI2CDevs[bus][dev]), std::end(mI2CDevs[bus][dev]), adr);
                *r.ptr = '\0';
            }
        }
    }
private:
    static inline name_t mName = []{
        name_t name{};
        updateName(name);
        return name;
    }();

    using i2c_strings_t = std::array<std::array<std::array<char, 16>, 4>, 2>;
    static inline auto mI2CDevs = []{
        i2c_strings_t s;
        for(auto& b : s) {
            for(auto& d : b) {
                strcpy(&d[0], "---");
            }
        }
        return s;
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
    using params_t = etl::FixedVector<Param_t, 128>;
    static inline params_t params = [] {
        params_t p;
        addNode(p, Param_t{0, PType::Folder, "root"});
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::U8,  "CRSF Address", nullptr, &eeprom.address, 192, 207, [](const uint8_t a){updateName(mName); src::address(std::byte(a)); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Controller Number", nullptr, &eeprom.controllerNumber, 0, 8, [](const uint8_t){return true;}});
        addNode(p, Param_t{0, PType::Sel, "Aux1", "SBus2;Inv.SBus2;Hw/Ext;Cppm/In;Off", &eeprom.aux1mode, 0, 4, [](const uint8_t v){
                               if (v == 3) {
                                   encs2::set(1);
                               }
                               auxes1::set(v);
                               return true;}});
        addNode(p, Param_t{0, PType::Sel, "Aux2", "SBus2;Inv.SBus2;Hw/Ext;Off", &eeprom.aux2mode, 0, 3, [](const uint8_t v){auxes2::set(v); return true;}});
        addNode(p, Param_t{0, PType::Sel, "Sm1", "SpaceMouse;Off", &eeprom.sm1mode, 0, 1, [](const uint8_t v){smes1::set(v); return true;}});
        addNode(p, Param_t{0, PType::Sel, "Sm2", "SpaceMouse;Off", &eeprom.sm2mode, 0, 1, [](const uint8_t v){smes2::set(v); return true;}});
        addNode(p, Param_t{0, PType::Sel, "Enc1", "Encoder;Off", &eeprom.enc1mode, 0, 1, [](const uint8_t v){encs1::set(v); return true;}});
        addNode(p, Param_t{0, PType::Sel, "Enc2", "Encoder;Off", &eeprom.enc2mode, 0, 1, [](const uint8_t v){
                               if (v == 0) {
                                   auxes1::set(4);
                               }
                               encs2::set(v);
                               return true;}});
        addNode(p, Param_t{0, PType::Sel, "Bus", "CRSF;Off", &eeprom.busmode, 0, 1, [](const uint8_t v){busses::set(v); return true;}});

        auto parent = addParent(p, Param_t{0, PType::Folder, "Bus"});
        addNode(p, Param_t{parent, PType::U8,  "TX Rewrite Address", nullptr, &eeprom.tx_rewrite_address, 192, 207, [](const uint8_t a){forwarder::txAddress(a); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "RX Rewrite Address", nullptr, &eeprom.rx_rewrite_address, 192, 207, [](const uint8_t a){forwarder::rxAddress(a); return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Analog1"});
        addNode(p, Param_t{parent, PType::Sel, "Stream", "SBus;Hw/Ext;Off", (uint8_t*)&eeprom.analogMaps[0].stream, 0, 2, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Position", nullptr, &eeprom.analogMaps[0].position, 1, 16, [](const uint8_t){return true;}});
        parent = addParent(p, Param_t{0, PType::Folder, "Analog2"});
        addNode(p, Param_t{parent, PType::Sel, "Stream", "SBus;Hw/Ext;Off", (uint8_t*)&eeprom.analogMaps[1].stream, 0, 2, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Position", nullptr, &eeprom.analogMaps[1].position, 1, 16, [](const uint8_t){return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "SpaceMouse1"});
        addNode(p, Param_t{parent, PType::Sel, "Stream", "SBus;Hw/Ext;Off", (uint8_t*)&eeprom.smMaps[0].stream, 0, 2, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Position", nullptr, &eeprom.smMaps[0].position, 1, 16, [](const uint8_t){return true;}});
        parent = addParent(p, Param_t{0, PType::Folder, "SpaceMouse2"});
        addNode(p, Param_t{parent, PType::Sel, "Stream", "SBus;Hw/Ext;Off", (uint8_t*)&eeprom.smMaps[1].stream, 0, 2, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Position", nullptr, &eeprom.smMaps[1].position, 1, 16, [](const uint8_t){return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Incrementals"});
        addNode(p, Param_t{parent, PType::Sel, "Stream1", "SBus;Hw/Ext;Off", (uint8_t*)&eeprom.incMaps[0].stream, 0, 2, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Position1", nullptr, &eeprom.incMaps[0].position, 1, 16, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Stream2", "SBus;Hw/Ext;Off", (uint8_t*)&eeprom.incMaps[1].stream, 0, 2, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Position2", nullptr, &eeprom.incMaps[1].position, 1, 16, [](const uint8_t){return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "BlueTooth"});
        addNode(p, Param_t{parent, PType::Sel, "Bluetooth", "off;on", &eeprom.bluetooth, 0, 1, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Stream", "SBus;Hw/Ext;Off", (uint8_t*)&eeprom.bluetoothMap.stream, 0, 2, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Position", nullptr, &eeprom.bluetoothMap.position, 1, 16, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Count", nullptr, &eeprom.bluetoothMap.count, 0, 16, [](const uint8_t){return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Receiver"});
        addNode(p, Param_t{parent, PType::Sel, "CRSF Channels", "off;on", &eeprom.crsf_in, 0, 1, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Stream", "SBus;Hw/Ext;Off", (uint8_t*)&eeprom.crsfInMap.stream, 0, 2, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Position", nullptr, &eeprom.crsfInMap.position, 1, 16, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Count", nullptr, &eeprom.crsfInMap.count, 0, 16, [](const uint8_t){return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "I2C"});
        // for(uint8_t k = 0; k < 4; ++k) {
        //     addNode(p, Param_t{parent, PType::Info, "Bus1", &mI2CDevs[0][k][0]});
        // }
        addNode(p, Param_t{parent, PType::Info, "Bus1", &mI2CDevs[0][0][0]});
        addNode(p, Param_t{parent, PType::Info, "Bus1", &mI2CDevs[0][1][0]});
        addNode(p, Param_t{parent, PType::Info, "Bus1", &mI2CDevs[0][2][0]});
        addNode(p, Param_t{parent, PType::Info, "Bus1", &mI2CDevs[0][3][0]});
        addNode(p, Param_t{parent, PType::Info, "Bus2", &mI2CDevs[1][0][0]});
        addNode(p, Param_t{parent, PType::Info, "Bus2", &mI2CDevs[1][1][0]});
        addNode(p, Param_t{parent, PType::Info, "Bus2", &mI2CDevs[1][2][0]});
        addNode(p, Param_t{parent, PType::Info, "Bus2", &mI2CDevs[1][3][0]});

        parent = addParent(p, Param_t{0, PType::Folder, "CPPM/In"});
        addNode(p, Param_t{parent, PType::U8,  "Exp N", nullptr, &eeprom.cppm_exp_n, 1, 99, [](const uint8_t v){pulse_in::setExpN(v); return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Settings"});
        addNode(p, Param_t{parent, PType::Command, "Calibate", "Calibrate...", nullptr, 0, 0, [](const uint8_t){return false;}});

        // detects overflow by calling undefined function f();
        if (p.size() >= p.capacity()) {
            void f();
            f();
        }
        return p;
    }();
};

