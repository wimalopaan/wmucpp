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

    using pca = Config::pca;

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
        auto r = std::to_chars(std::begin(n) + strlen(title), std::end(n), eeprom.address1);
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
                if (eeprom.address1 == address) {
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
                        if constexpr(requires(){switchcallback::prop(ch, duty);}) {
                            IO::outl<trace>("# Cmd Prop: ", address, " ch: ", ch, " d: ", duty);
                            switchcallback::prop(ch, duty);
                        }
                        else {
                            IO::outl<trace>("# Cmd Prop: not implemented");
                        }
                    }
                }
                else if (eeprom.address2 == address) {
                    // out 8-17
                }
                else if (eeprom.address3 == address) {
                    // group 0-3
                    // 0-3: single shot
                    // 4-7: continous
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

    static inline constexpr uint8_t addParent(auto& c, const Param_t& p) {
        c.push_back(p);
        return c.size() - 1;
    }
    static inline constexpr void addNode(auto& c, const Param_t& p) {
        c.push_back(p);
    }
    static inline constexpr bool setAddress(const uint8_t) {
        update();
        return true;
    }
    // need to write the lambdas without capture: capturing lambdas cannot be converted to function pointer
    // so we need to write the whole thing as template
    template<uint8_t index>
    static inline constexpr void addOutput(auto& p, const uint8_t parent, const char* const name) {
        const uint8_t parent2 = addParent(p, Param_t{parent, PType::Folder, name});
        addNode(p, Param_t{parent2, PType::Info, name, &mName[0]});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "PWM", .value_ptr = &eeprom.outputs[index].pwm, .min = 1, .max = 255, .cb = [](const uint8_t v){pca::ledPwm(index, v); return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "Current", .value_ptr = &eeprom.outputs[index].iref, .min = 1, .max = 255, .cb = [](const uint8_t v){pca::ledIRef(index, v); return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::Sel, .name = "Group", .options = "None;0;1;2;3", .value_ptr = &eeprom.outputs[index].group, .min = 0, .max = 4, .cb = [](const uint8_t v){if (v  == 0) {pca::ledGradationMode(index, false);} else {pca::ledGradationMode(index, true); pca::ledGroup(index, (v - 1));} return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::Sel, .name = "Control", .options = "Off;Full;Individual;Group", .value_ptr = &eeprom.outputs[index].control, .min = 0, .max = 3, .cb = [](const uint8_t v){pca::ledControl(index, (v & 0x03)); return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::Sel, .name = "Test", .options = "Off;On", .min = 0, .max = 1, .cb = [](const uint8_t v){switchcallback::setIndex(index, (v > 0)); return false;}});
    }
    template<uint8_t index>
    static inline constexpr void addGroup(auto& p, const uint8_t parent, const char* const name) {
        const uint8_t parent2 = addParent(p, Param_t{parent, PType::Folder, name});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "Current", .value_ptr = &eeprom.groups[index].iref, .min = 1, .max = 255, .cb = [](const uint8_t v){pca::groupIRef(index, v); return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::Sel, .name = "Ramp", .options = "Off;Down;Up;Both", .value_ptr = &eeprom.groups[index].ramp, .min = 0, .max = 3, .cb = [](const uint8_t v){pca::groupRamp(index, v); return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "Rate", .value_ptr = &eeprom.groups[index].rate, .min = 0, .max = 63, .cb = [](const uint8_t v){pca::groupRampRate(index, v); return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "Step time", .value_ptr = &eeprom.groups[index].stepTime, .min = 0, .max = 63, .cb = [](const uint8_t v){pca::groupStepTime(index, v); return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "Hold On time", .value_ptr = &eeprom.groups[index].holdOnTime, .min = 0, .max = 7, .cb = [](const uint8_t v){pca::groupHoldOnTime(index, v); return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "Hold Off time", .value_ptr = &eeprom.groups[index].holdOffTime, .min = 0, .max = 7, .cb = [](const uint8_t v){pca::groupHoldOffTime(index, v); return true;}});
    }
#ifdef TEST1
    static inline auto mNameTest = []{
        std::array<char, 32> a;
        strcpy(&a[0], "bla");
        return a;
    }();
#endif
    using params_t = etl::FixedVector<Param_t, 200>;

    static inline params_t params = [] {
        params_t p;
        addNode(p, Param_t{0, PType::Folder, ""}); // unvisible top folder
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        auto parent = addParent(p, Param_t{0, PType::Folder, "Global"});
        addNode(p, Param_t{parent, PType::U8, "Address Out 0-7", nullptr, &eeprom.address1, 0, 255, setAddress});
        addNode(p, Param_t{parent, PType::U8, "Address Out 8-15", nullptr, &eeprom.address2, 0, 255, setAddress});
        addNode(p, Param_t{parent, PType::U8, "Address Grp 0-3", nullptr, &eeprom.address3, 0, 255, setAddress});

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

        parent = addParent(p, Param_t{0, PType::Folder, "Group 0"});
        addOutput<0>(p, parent, "Output 0");
        addOutput<1>(p, parent, "Output 1");
        addOutput<2>(p, parent, "Output 2");
        addOutput<3>(p, parent, "Output 3");
        addGroup<0>(p, parent, "Group 0");

        parent = addParent(p, Param_t{0, PType::Folder, "Group 1"});
        addOutput<4>(p, parent, "Output 4");
        addOutput<5>(p, parent, "Output 5");
        addOutput<6>(p, parent, "Output 6");
        addOutput<7>(p, parent, "Output 7");
        addGroup<1>(p, parent, "Group 1");

        parent = addParent(p, Param_t{0, PType::Folder, "Group 2"});
        addOutput<8>(p, parent, "Output 8");
        addOutput<9>(p, parent, "Output 9");
        addOutput<10>(p, parent, "Output 10");
        addOutput<11>(p, parent, "Output 11");
        addGroup<2>(p, parent, "Group 2");

        parent = addParent(p, Param_t{0, PType::Folder, "Group 3"});
        addOutput<12>(p, parent, "Output 12");
        addOutput<13>(p, parent, "Output 13");
        addOutput<14>(p, parent, "Output 14");
        addOutput<15>(p, parent, "Output 15");
        addGroup<3>(p, parent, "Group 3");

        if (p.size() >= p.capacity()) {
            void fp();
            fp();
        }
        return p;
    }();
};
