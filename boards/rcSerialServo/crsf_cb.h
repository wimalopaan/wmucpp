/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
    using watchdog = Config::watchdog;
    using tp = Config::tp;
    using storage = Config::storage;
    using Param_t = RC::Protokoll::Crsf::V4::Parameter<uint8_t>;
    using PType = Param_t::Type;
    using src = Config::src;
    using input = src::input;
    using srv = Config::srv;

    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;
    static inline constexpr const char* const title = "RC-SerialServo@";

    using name_t = std::array<char, 32>;
    
    // static inline constexpr void disableTelemetry() {
    // }
    static inline void gotChannels() {
        srv::set(0, input::value(0));
        srv::set(1, input::value(1));
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
    static inline std::array<char, 16> mWdgString = {};

    static inline void makeWdgString()
        requires(!std::is_same_v<watchdog, void>)
    {
#ifdef USE_WATCHDOG_TEST
        auto [ptr, e] = std::to_chars(std::begin(mWdgString), std::end(mWdgString), watchdog::testCount());
        *ptr++ = ':';
        auto r = std::to_chars(ptr, std::end(mWdgString), watchdog::wdgResets());
        *r.ptr = '\0';
#else
        auto [ptr, e] = std::to_chars(std::begin(mWdgString), std::end(mWdgString), watchdog::pinResets());
        *ptr++ = ':';
        auto r = std::to_chars(ptr, std::end(mWdgString), watchdog::wdgResets());
        *r.ptr = '\0';
#endif
    }
    static inline const char* name() {
        return &mName[0];
    }
    static inline uint32_t serialNumber() {
        if constexpr(!std::is_same_v<watchdog, void>) {
            makeWdgString();
        }
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
    static inline void makeServoStrings() {
        IO::outl<debug>("# makeServoStrings");
        for(uint8_t i = 0; i < srv::servoIds().size(); ++i) {
            auto r = std::to_chars(std::begin(mServos[i]), std::end(mServos[i]), srv::servoIds()[i]);
            *r.ptr++ = ':';
            *r.ptr++ = ' ';
            *r.ptr++ = 'H';
            *r.ptr++ = 'w';
            *r.ptr++ = ':';
            r = std::to_chars(r.ptr, std::end(mServos[i]), srv::hwVersion(i).first);
            *r.ptr++ = '.';
            r = std::to_chars(r.ptr, std::end(mServos[i]), srv::hwVersion(i).second);
            *r.ptr++ = ' ';
            *r.ptr++ = 'F';
            *r.ptr++ = 'w';
            *r.ptr++ = ':';
            r = std::to_chars(r.ptr, std::end(mServos[i]), srv::fwVersion(i).first);
            *r.ptr++ = '.';
            r = std::to_chars(r.ptr, std::end(mServos[i]), srv::fwVersion(i).second);
            *r.ptr = '\0';
        }
    }    
private:
    static inline std::array<const char*, 4> mSetIdTexts {
        "Set Servo Id",
        "Be sure to connect only one servo ...",
        "Set ...",
        "Finished"
    };
    static inline uint8_t mSetIdCommand = 0;
    static inline uint8_t mIdToBeSet = 1;
    static inline bool setIdCb(const uint8_t v) {
        const RC::Protokoll::Crsf::V4::Lua::CmdStep step{v};
        bool res = false;
        switch(step) {
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Click:
            params[mSetIdCommand].options = mSetIdTexts[1];
            res = true;
            break;
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Confirmed:
            params[mSetIdCommand].options = mSetIdTexts[2];
            srv::setId(mIdToBeSet);
            break;
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Cancel:
            params[mSetIdCommand].options = mSetIdTexts[3];
            break;
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Query:
            params[mSetIdCommand].options = mSetIdTexts[2];
            res = false;
            break;
        default:
            break;
        }
        IO::outl<debug>("# setId v: ", v, " r: ", (uint8_t)res);
        return res;
    }
    static inline std::array<const char*, 4> mPingTexts {
        "Ping Servo Bus",
        "Start ...",
        "Pinging ...",
        "Finished pinging"
    };
    static inline uint8_t mPingCommand = 0;

    // Step         Result(bool)    Next-Step       Action
    // Click        false           Executing       start task
    //              true            AskConfirm
    // Confirmed    false           Executing       start task
    // Cancel       false                           cancel task
    // Update       false           Idle
    //              true            Executing

    static inline bool pingCb(const uint8_t v) {
        const RC::Protokoll::Crsf::V4::Lua::CmdStep step{v};
        bool res = false;
        switch(step) {
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Click:
            params[mPingCommand].options = mPingTexts[1];
            res = true;
            break;
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Confirmed:
            params[mPingCommand].options = mPingTexts[2];
            srv::startPing();
            break;
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Cancel:
            params[mPingCommand].options = mPingTexts[3];
            srv::stopPing();
            break;
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Query:
            params[mPingCommand].options = mPingTexts[2];
            res = srv::isPinging();
            break;
        default:
            break;
        }
        IO::outl<debug>("# ping v: ", v, " r: ", (uint8_t)res);
        return res;
    }

    static inline name_t mName = []{
        name_t name{};
        updateName(name);
        return name;
    }();

    using srv_strings_t = std::array<std::array<char, 32>, 8>;
    static inline auto mServos = []{
        srv_strings_t s;
        for(auto& d : s) {
            strcpy(&d[0], "---");
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
    using params_t = etl::FixedVector<Param_t, 32>;
    static inline params_t params = [] {
        params_t p;
        uint8_t parent = 0;
        addNode(p, Param_t{0, PType::Folder, "root"});
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::Info, "Watchdog", &mWdgString[0]});
        addNode(p, Param_t{0, PType::U8,  "CRSF Address", nullptr, &eeprom.address, 192, 207, [](const uint8_t a){
                               updateName(mName); src::address(std::byte(a));
                    return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Configuration"});
        addNode(p, Param_t{parent, PType::Command, "Ping", mPingTexts[0], nullptr, 
                           // 10, // timeout = 10 * 100ms 
                           250, // timeout = 250ms
                           0, [](const uint8_t v){
                               return pingCb(v);
                           }
                });
        mPingCommand = p.size() - 1;

        parent = addParent(p, Param_t{0, PType::Folder, "Advanced"});
        addNode(p, Param_t{parent, PType::U8,  "Servo ID to be set", nullptr, &mIdToBeSet, 1, 250, [](const uint8_t){return false;}});
        addNode(p, Param_t{parent, PType::Command, "Set Servo ID", mSetIdTexts[0], nullptr, 
                           // 10, // timeout = 10 * 100ms 
                           250, // timeout = 250ms
                           0, [](const uint8_t v){
                               return setIdCb(v);
                           }
                });
        mSetIdCommand = p.size() - 1;
        
        parent = addParent(p, Param_t{0, PType::Folder, "Servos"});
        addNode(p, Param_t{parent, PType::Info, "Servo 0:", &mServos[0][0]});
        addNode(p, Param_t{parent, PType::Info, "Servo 1:", &mServos[1][0]});
        addNode(p, Param_t{parent, PType::Info, "Servo 2:", &mServos[2][0]});
        addNode(p, Param_t{parent, PType::Info, "Servo 3:", &mServos[3][0]});
        addNode(p, Param_t{parent, PType::Info, "Servo 4:", &mServos[4][0]});
        addNode(p, Param_t{parent, PType::Info, "Servo 5:", &mServos[5][0]});
        addNode(p, Param_t{parent, PType::Info, "Servo 6:", &mServos[6][0]});
        addNode(p, Param_t{parent, PType::Info, "Servo 7:", &mServos[7][0]});
        
        
        // detects overflow by calling undefined function f();
        if (p.size() >= p.capacity()) {
            void f();
            f();
        }
        return p;
    }();
};

