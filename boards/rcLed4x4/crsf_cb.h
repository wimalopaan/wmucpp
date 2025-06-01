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

template<typename Config, typename Trace = void>
struct CrsfCallback {
    using trace = Trace;
    using debug = trace;

    using Param_t = RC::Protokoll::Crsf::V4::Parameter<uint8_t>;
    using PType = Param_t::Type;

    using timer = Config::timer;
    using crsf = Config::crsf;
    using messageBuffer = crsf::messageBuffer;
    using storage = Config::storage;

    using switchcallback = Config::switchcallback;

    // static_assert(sizeof(EEProm) < 2048);
    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;

    static inline constexpr const char* const title = "Led4x4-E@";

    using name_t = std::array<char, 32>;

    enum class State : uint8_t {Undefined, GotStats, GotChannels};

    static inline State mStreamState{State::Undefined};

    static inline constexpr void ratePeriodic() {
    }
    static inline constexpr void updateName(name_t& n) {
        strncpy(&n[0], title, n.size());
        auto r = std::to_chars(std::begin(n) + strlen(title), std::end(n), eeprom.address);
        *r.ptr++ = ':';
        r = std::to_chars(r.ptr, std::end(n), eeprom.crsf_address);
        *r.ptr++ = '\0';
    }
    static inline void update() {
        updateName(mName);
    }
    static inline void save() {
        if (const auto [ok, err] = Mcu::Stm32::savecfg(eeprom, eeprom_flash); ok) {
            IO::outl<trace>("# EEPROM OK");
        }
        else {
            IO::outl<trace>("# EEPROM NOK: ", err);
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
        IO::outl<debug>("# cb param i: ", index);
        if (index < params.size()) {
            return params[index];
        }
        return {};
    }
    static inline bool isCommand(const uint8_t index) {
        IO::outl<debug>("# cb isCom i: ", index);
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
    static inline void command(const auto payload, const uint8_t ) {
        const uint8_t destAddress = payload[3];
        const uint8_t srcAddress = payload[4];
        const uint8_t realm = payload[5];
        const uint8_t cmd = payload[6];
        const uint8_t address = (uint8_t)payload[7];
        if ((srcAddress == (uint8_t)RC::Protokoll::Crsf::V4::Address::Handset) && (destAddress >= 0xc0) && (destAddress <= 0xcf)) {
            if (realm == (uint8_t)RC::Protokoll::Crsf::V4::CommandType::Switch) {
                if (eeprom.address == address) {
                    if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::Set) {
                        const uint8_t sw = (uint8_t)payload[8];
                        IO::outl<trace>("# Cmd Set: ", address, " sw: ", sw);
                        switchcallback::set(sw);
                    }
                    else if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::Set4) {
                        const uint16_t sw = (((uint16_t)payload[8]) << 8) + payload[9];
                        IO::outl<trace>("# Cmd Set4: ", address, " sw: ", sw);
                        uint8_t sw8 = 0;
                        for(uint8_t i = 0; i < 8; ++i) {
                            const uint8_t s = (sw >> (2 * i)) & 0b11;
                            if (s > 0) {
                                sw8 |= (1 << i);
                            }
                        }
                        switchcallback::set(sw8);
                    }
                    else if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::Prop) {
                        const uint8_t ch = (uint8_t)payload[8];
                        const uint8_t duty = (uint8_t)payload[9];
                        IO::outl<trace>("# Cmd Prop: ", address, " ch: ", ch, " d: ", duty);
                        switchcallback::prop(ch, duty);
                    }
#ifdef USE_AUTO_CONF
                    else if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::RequestConfigItem) {
                        const uint8_t item = (uint8_t)payload[8];
                        IO::outl<trace>("# Cmd Req CI: ", item);
                        messageBuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ArduPilot, [&](auto& d){
                            d.push_back(RC::Protokoll::Crsf::V4::Address::Handset);
                            d.push_back((uint8_t)eeprom.crsf_address);
                            d.push_back(RC::Protokoll::Crsf::V4::ArduPilotTunnel::Switch::AppId);
                            d.push_back((uint8_t)eeprom.address);
                            d.push_back(RC::Protokoll::Crsf::V4::ArduPilotTunnel::Switch::Type::ConfigItem);
                            d.push_back(item);
                            etl::push_back_ntbs(&eeprom.outputs[item].name[0], d);
                            d.push_back(eeprom.outputs[item].ls);
                            d.push_back(eeprom.outputs[item].type);
                        });
                    }
                    else if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::RequestDeviceInfo) {
                        IO::outl<trace>("# Cmd Req DI");
                        messageBuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ArduPilot, [&](auto& d){
                            d.push_back(RC::Protokoll::Crsf::V4::Address::Handset);
                            d.push_back((uint8_t)eeprom.crsf_address);
                            d.push_back(RC::Protokoll::Crsf::V4::ArduPilotTunnel::Switch::AppId);
                            d.push_back((uint8_t)eeprom.address);
                            d.push_back(RC::Protokoll::Crsf::V4::ArduPilotTunnel::Switch::Type::DeviceInfo);
                            d.push_back(RC::Protokoll::Crsf::V4::SwitchCommand::Set); // use this protocol as set protocol
                            d.push_back(uint8_t{HW_VERSION});
                            d.push_back(uint8_t{SW_VERSION});
                        });
                    }
#endif
                }
            }
        }
    }
    static inline void serialize(const uint8_t index, auto& buffer, const RC::Protokoll::Crsf::V4::Lua::CmdStep step = RC::Protokoll::Crsf::V4::Lua::CmdStep::Idle) {
        params[index].serialize(buffer, params, step, index);
    }
    static inline void callbacks(const bool eepromMode = false) {
        const bool prevMode = mEepromMode;
        mEepromMode = eepromMode;
        for(const auto p: params) {
            if (p.cb) {
                p.cb(p.value());
            }
        }
        mEepromMode = prevMode;
    }
private:
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
    static inline name_t mName = []{
        name_t name{};
        updateName(name);
        return name;
    }();

    static inline uint8_t addParent(auto& c, const Param_t& p) {
        c.push_back(p);
        return c.size() - 1;
    }
    static inline void addNode(auto& c, const Param_t& p) {
        c.push_back(p);
    }
    static inline bool setAddress(const uint8_t) {
        update();
        return true;
    }
#ifdef TEST1
    static inline auto mNameTest = []{
        std::array<char, 32> a;
        strcpy(&a[0], "bla");
        return a;
    }();
#endif
#ifdef USE_AUTO_CONF
    using params_t = etl::FixedVector<Param_t, 200>;
#else
    using params_t = etl::FixedVector<Param_t, 125>;
#endif
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Folder, ""}); // unvisible top folder
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
#ifdef TEST1
        // String-type not supported by elrsv3.lua (Issue 2859)
        addNode(p, Param_t{0, PType::Str, "Name", nullptr, nullptr, 0, 0, nullptr, 0, 8, 0, &mNameTest[0]}); // maxlen 8
        addNode(p, Param_t{0, PType::U8, "Address", nullptr, &eeprom.address, 0, 255, [](const uint8_t){return true;}});
        // addNode(p, Param_t{0, PType(PType::Str | PType::Hidden), "N0", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &mNameTest[0]});
        // addNode(p, Param_t{0, PType(PType::Str | PType::Hidden), "N2", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &mNameTest[0]});
        // addNode(p, Param_t{0, PType(PType::Str | PType::Hidden), "N3", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &mNameTest[0]});
        // addNode(p, Param_t{0, PType(PType::U8 | PType::Hidden), "HiddenInfo", nullptr, nullptr, 0, 255});
#endif
#ifndef TEST1
        auto parent = addParent(p, Param_t{0, PType::Folder, "Global"});
        addNode(p, Param_t{parent, PType::U8, "Address", nullptr, &eeprom.address, 0, 255, setAddress});

        addNode(p, Param_t{parent, PType::U8, "CRSF Address", nullptr, &eeprom.crsf_address, 0xc0, 0xcf, [](const uint8_t v){
                               if (!mEepromMode) {
                                   const uint8_t slot = 2 * (v - 0xc0);
                                   eeprom.response_slot = slot;
                                   crsf::output::telemetrySlot(slot);
                               }
                               crsf::address(std::byte{v});
                               return true;
                           }});
        addNode(p, Param_t{parent, PType::U8, "Response Slot", nullptr, &eeprom.response_slot, 0, 15, [](const uint8_t v){crsf::output::telemetrySlot(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Config resp.", "Button;Allways on", &eeprom.telemetry, 0, 1});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 0"});
        addNode(p, Param_t{parent, PType::Info, "Output 0 : ", &mName[0]});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM Duty", .value_ptr = &eeprom.outputs[0].pwmDuty, .min = 1, .max = 99, .cb = [](const uint8_t){return true;}, .unitString = "%"});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t){return false;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Operate"});
        addNode(p, Param_t{parent, PType::Sel, "Output 0", "Off;On", 0, 0, 1, [](const uint8_t){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 1", "Off;On", 0, 0, 1, [](const uint8_t){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 2", "Off;On", 0, 0, 1, [](const uint8_t){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 3", "Off;On", 0, 0, 1, [](const uint8_t){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 4", "Off;On", 0, 0, 1, [](const uint8_t){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 5", "Off;On", 0, 0, 1, [](const uint8_t){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 6", "Off;On", 0, 0, 1, [](const uint8_t){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 7", "Off;On", 0, 0, 1, [](const uint8_t){return false;}});
#endif
        if (p.size() >= p.capacity()) {
            void f();
            f();
        }
        return p;
    }();
};
