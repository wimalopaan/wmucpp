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
    static inline void command(const auto /*payload*/, const uint8_t ) {
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
    using params_t = etl::FixedVector<Param_t, 10>;

    static inline params_t params = [] {
        params_t p;
        addNode(p, Param_t{0, PType::Folder, ""}); // unvisible top folder
        addNode(p, Param_t{0, PType::Info, "Ver (HW/SW)", &mVersionString[0]});
        auto parent = addParent(p, Param_t{0, PType::Folder, "Global"});


        if (p.size() >= p.capacity()) {
            void fp();
            fp();
        }
        return p;
    }();
};
