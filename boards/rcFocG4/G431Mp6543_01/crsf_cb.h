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

	using servo = Config::servo;
	
    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;

    static inline constexpr const char* const title = "ServoFOC@";

    using name_t = std::array<char, 32>;

    enum class State : uint8_t {Undefined, GotStats, GotChannels};

    static inline State mStreamState{State::Undefined};

	static inline constexpr void updateName(name_t& n) {
        strncpy(&n[0], title, n.size());
        auto r = std::to_chars(std::begin(n) + strlen(title), std::end(n), storage::eeprom.switch_address);
        *r.ptr++ = ':';
        r = std::to_chars(r.ptr, std::end(n), storage::eeprom.crsf_address);
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
    using params_t = etl::FixedVector<Param_t, 50>;

    static inline params_t params = [] {
        params_t p;
        addNode(p, Param_t{0, PType::Folder, ""}); // unvisible top folder
        addNode(p, Param_t{0, PType::Info, "Ver (HW/SW)", &mVersionString[0]});
        auto parent = addParent(p, Param_t{0, PType::Folder, "Crsf"});
		addNode(p, Param_t{parent, PType::U8, "Address", nullptr, &eeprom.switch_address, 0, 255, [](const uint8_t){update(); return true;}});
		addNode(p, Param_t{parent, PType::U8, "CRSF Addr", nullptr, &storage::eeprom.crsf_address, 0xc0, 0xcf, [](const uint8_t v){
                               if (!mEepromMode) {
                                   const uint8_t slot = 2 * (v - 0xc0);
                                   storage::eeprom.response_slot = slot;
                                   crsf::output::telemetrySlot(slot);
                               }
                               crsf::address(std::byte{v});
                               return true;
                           }});
		addNode(p, Param_t{.parent = parent, .type = PType::Sel, .name = "Mode", .options = "Servo;Winch;Crane;Custom", .value_ptr = &eeprom.mode, .min = 0, .max = 3, .cb = [](const uint8_t v){servo::preset(v); return true;}});

		parent = addParent(p, Param_t{0, PType::Folder, "Servo"});
		addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Angle", .value_ptr = &eeprom.mode_params[0].max_throw, .min = 1, .max = 250, .cb = [](const uint8_t v){servo::maxThrow(v); return true;}, .def = 100});
		addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Speed", .value_ptr = &eeprom.mode_params[0].max_speed, .min = 1, .max = 250, .cb = [](const uint8_t v){servo::speed(v); return true;}, .def = 100});
		
		parent = addParent(p, Param_t{0, PType::Folder, "Winch"});
		addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Force", .value_ptr = &eeprom.mode_params[1].max_force, .min = 1, .max = 250, .cb = [](const uint8_t v){servo::force(v); return true;}, .def = 100});
		addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Speed", .value_ptr = &eeprom.mode_params[1].max_speed, .min = 1, .max = 250, .cb = [](const uint8_t v){servo::speed(v); return true;}, .def = 100});

		parent = addParent(p, Param_t{0, PType::Folder, "Crane"});
		addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Angle", .value_ptr = &eeprom.mode_params[2].max_throw, .min = 1, .max = 250, .cb = [](const uint8_t v){servo::maxThrow(v); return true;}, .def = 100});
		addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Speed", .value_ptr = &eeprom.mode_params[2].max_speed, .min = 1, .max = 250, .cb = [](const uint8_t v){servo::speed(v); return true;}, .def = 100});
		addNode(p, Param_t{.parent = parent, .type = PType::Sel, .name = "Stop", .options = "On;Off", .value_ptr = &eeprom.mode_params[2].auto_stop, .min = 0, .max = 1, .cb = [](const uint8_t v){servo::preset(v); return true;}});

		// parent = addParent(p, Param_t{0, PType::Folder, "Custom"});
		// addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Angle", .value_ptr = &eeprom.mode_params[3].max_throw, .min = 1, .max = 250, .cb = [](const uint8_t v){servo::maxThrow(v); return true;}, .def = 100});
		// addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Speed", .value_ptr = &eeprom.mode_params[3].max_speed, .min = 1, .max = 250, .cb = [](const uint8_t v){servo::speed(v); return true;}, .def = 100});

        if (p.size() >= p.capacity()) {
            void fp();
            fp();
        }
        return p;
    }();
};
