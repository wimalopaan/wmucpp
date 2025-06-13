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
#include "uuid.h"
#include "rc/rc_2.h"
#include "rc/crsf_2.h"

template<typename Config, typename Debug = void>
struct CrsfCallback {
    using debug = Debug;
    using timer = Config::timer;
    using tp = Config::tp;
    using storage = Config::storage;
    using servos = Config::servos;
    using escs = Config::escs;
    using relays = Config::relays;
    using auxes = Config::auxes;
    using mapper = Config::mapper;
    using polars = Config::polars;

    using mpx1 = Config::mpx1;
    using sumdv3 = Config::sumdv3;

    using sport_aux = Config::sport_aux;

    using bluetooth = Config::bluetooth;
    using compass = Config::compass;
    using telemetry = Config::telemetry;

    using messageBuffer = Config::messageBuffer;

    using src = Config::src;
    using input  = src::input;

    using p1 = Meta::nth_element<0, polars>;
    using p2 = Meta::nth_element<1, polars>;

    using store_t = EEProm::eeprom_value_t;
    using Param_t = RC::Protokoll::Crsf::V4::Parameter<store_t>;
    using PType = Param_t::Type;

    using esc32ascii_1 = Config::esc32ascii_1;
    using esc32ascii_2 = Config::esc32ascii_2;

    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;
    static inline constexpr const char* const title = "RC-720-E-32@";

    using name_t = std::array<char, 32>;

    enum class State : uint8_t {Undefined, GotStats, GotChannels};

    static inline State mStreamState{State::Undefined};

    static inline constexpr void disableTelemetry() {
        IO::outl<debug>("# disable telemetry");
        telemetry::disableWithAutoOn();
    }
    static inline constexpr void gotLinkStats() {
    }
    static inline constexpr void gotChannels() {
    }
    static inline constexpr void forwardPacket(const auto data, const uint16_t length) {
        relays::forwardPacket(data, length);
        auxes::forwardPacket(data, length);
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
        updateCalib(mCalTexts);
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
    static inline void command(const auto& data, const uint8_t /*payload*/) {
        const uint8_t destAddress = data[3];
        const uint8_t srcAddress = data[4];
        const uint8_t realm = data[5];
        const uint8_t cmd = data[6];
        if ((srcAddress == (uint8_t)RC::Protokoll::Crsf::V4::Address::Handset) && (destAddress >= 0xc0) && (destAddress <= 0xcf)) {
            if (realm == (uint8_t)RC::Protokoll::Crsf::V4::CommandType::Schottel) {
                if (eeprom.address == destAddress) {
                    if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SchottelCommand::Reset) {
                        IO::outl<debug>("# Cmd Reset: ", destAddress);
                        servos::zero();
                    }
                }
            }
            else if (realm == (uint8_t)RC::Protokoll::Crsf::V4::CommandType::Switch) {
                if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::Set) {
                    const uint8_t swAddress = data[7];
                    const uint16_t sw = data[8];
                    if (eeprom.switchAddress == swAddress) {
                        IO::outl<debug>("# Switch set address: ", swAddress, " v: ", sw);
                        for(uint8_t i = 0; i < 8; ++i) {
                            const uint8_t s = (sw >> i) & 0b01;
                            mpx1::set(i, s);
                            sumdv3::setSwitch(i, s);
                        }
                    }
                }
                else if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::Set4M) {
                    const uint8_t count = data[7];
                    for(uint8_t i = 0; i < count; ++i) {
                        const uint8_t swAddress = data[8 + 3 * i];
                        const uint16_t sw = (data[9 + 3 * i] << 8) + data[10 + 3 * i];
                        IO::outl<debug>("# Switch set4M: ", i, " adr: ", swAddress);
                        if (eeprom.switchAddressContiguous == 0) {
                            if (eeprom.switchAddress == swAddress) {
                                IO::outl<debug>("# Switch set4M adr: ", swAddress, " v: ", sw);
                                for(uint8_t k = 0; k < 8; ++k) {
                                    const uint8_t s = (sw >> (2 * k)) & 0b11;
                                    mpx1::set(k, s);
                                    sumdv3::setSwitch(k, s);
                                }
                            }
                        }
                        else {
                            const uint8_t minAdr = eeprom.switchAddress;
                            const uint8_t maxAdr = eeprom.switchAddress + 7;
                            if ((swAddress >= minAdr) && (swAddress <= maxAdr)) {
                                IO::outl<debug>("# Switch set4M adr: ", swAddress, " v: ", sw);
                                const uint8_t swGroup = swAddress - minAdr;
                                for(uint8_t k = 0; k < 8; ++k) {
                                    const uint8_t n = swGroup * 8 + k;
                                    const uint8_t s = (sw >> (2 * k)) & 0b11;
                                    mpx1::set(n, s);
                                    sumdv3::setSwitch(n, s);
                                }
                            }
                        }
                    }
                }
                else if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::Set4) {
                    const uint8_t swAddress = data[7];
                    const uint16_t sw = (data[8] << 8) + data[9];
                    if (eeprom.switchAddress == swAddress) {
                        IO::outl<debug>("# Switch set4 address: ", swAddress, " v: ", sw);
                        for(uint8_t i = 0; i < 8; ++i) {
                            const uint8_t s = (sw >> (2 * i)) & 0b11;
                            mpx1::set(i, s);
                            sumdv3::setSwitch(i, s);
                        }
                    }
                }
                else if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::Set64) {
                    const uint8_t swAddress = data[7];
                    const uint8_t swGroup = (data[8] & 0x07);
                    // const uint16_t sw = (data[8] << 8) + data[9];
                    // todo: 2bit state
                    const uint8_t swSwitches = data[9];
                    if (eeprom.switchAddress == swAddress) {
                        IO::outl<debug>("# Switch set64 address: ", swAddress, " v: ", swSwitches, " g: ", swGroup);
                        for(uint8_t i = 0; i < 8; ++i) {
                            const uint8_t n = swGroup * 8 + i;
                            const uint8_t s = (swSwitches >> i) & 0b01;
                            mpx1::set(n, s);
                            sumdv3::setSwitch(n, s);
                        }
                    }
                }
                else if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::RequestDeviceInfo) {
                    const uint8_t swAddress = data[7];
                    if (eeprom.switchAddress == swAddress) {
                        IO::outl<debug>("# Cmd ReqDevInfo adr: ", swAddress);
                        messageBuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ArduPilot, [&](auto& d){
                            d.push_back(RC::Protokoll::Crsf::V4::Address::Handset);
                            d.push_back((uint8_t)eeprom.address);
                            d.push_back(RC::Protokoll::Crsf::V4::ArduPilotTunnel::Switch::AppId);
                            d.push_back((uint8_t)eeprom.switchAddress);
                            d.push_back(RC::Protokoll::Crsf::V4::ArduPilotTunnel::Switch::Type::DeviceInfo);

                            d.push_back(uuid);
                            d.push_back(eeprom.config_counter);

                            d.push_back(RC::Protokoll::Crsf::V4::SwitchCommand::Set4); // use this protocol as set protocol
                            d.push_back(uint8_t{HW_VERSION});
                            d.push_back(uint8_t{SW_VERSION});
                        });
                    }
                }
                else if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::RequestConfigItem) {
                    const uint8_t swAddress = data[7];
                    const uint8_t item = data[8];
                    if (eeprom.switchAddress == swAddress) {
                        IO::outl<debug>("# Cmd Req: ", item, " adr: ", swAddress, " name: ", eeprom.switches[item].name);
                        messageBuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ArduPilot, [&](auto& d){
                            d.push_back(RC::Protokoll::Crsf::V4::Address::Handset);
                            d.push_back((uint8_t)eeprom.address);
                            d.push_back(RC::Protokoll::Crsf::V4::ArduPilotTunnel::Switch::AppId);
                            d.push_back((uint8_t)eeprom.switchAddress);
                            d.push_back(RC::Protokoll::Crsf::V4::ArduPilotTunnel::Switch::Type::ConfigItem);
                            d.push_back(item);
                            etl::push_back_ntbs(&eeprom.switches[item].name[0], d);
                            d.push_back(eeprom.switches[item].ls);
                            d.push_back(eeprom.switches[item].type);
                        });
                    }
                }
            }
        }
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
    static inline void setI2CDev(const uint8_t dev, const uint8_t adr) {
        if (dev < mI2CDevs.size()) {
            auto r = std::to_chars(std::begin(mI2CDevs[dev]), std::end(mI2CDevs[dev]), adr);
            *r.ptr = '\0';
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
    static inline const char* const setServoZeroPosText = "Zeroing";
    static inline bool setZeroPosition(const uint16_t) {
        return true;
    }
    static inline std::array<const char*, 6> mCompassCalibTexts {
        "Calibrate",
        "Start ...",
        "Calib: move all axes ...",
        "Finished calibrating"
    };

    // Step         Result(bool)    Next-Step       Action
    // Click        false           Executing       start task
    //              true            AskConfirm
    // Confirmed    false           Executing       start task
    // Cancel       false                           cancel task
    // Update       false           Idle
    //              true            Executing

    static inline bool compassCalibCb(const uint8_t v) {
        IO::outl<debug>("# compassCalibCb v: ", v);
        const RC::Protokoll::Crsf::V4::Lua::CmdStep step{v};
        bool res = false;
        switch(step) {
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Click:
            params[mCompassCalibCommand].options = mCompassCalibTexts[1];
            res = true;
            break;
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Confirmed:
            params[mCompassCalibCommand].options = mCompassCalibTexts[2];
            compass::startCalibrate();
            break;
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Cancel:
            params[mCompassCalibCommand].options = mCompassCalibTexts[3];
            compass::stopCalibrate();
            break;
        case RC::Protokoll::Crsf::V4::Lua::CmdStep::Query:
            params[mCompassCalibCommand].options = mCompassCalibTexts[2];
            res = compass::isCalibrating();
            break;
        default:
            break;
        }
        return res;
    }
    static inline uint8_t mServoCalibCommand = 0;
    static inline uint8_t mServoAddressCommand = 0;
    static inline uint8_t mCompassCalibCommand = 0;
    static inline uint8_t mESCape321Folder{};
    static inline uint8_t mESCape321End{};
    static inline uint8_t mESCape322Folder{};
    static inline uint8_t mESCape322End{};

    static inline std::array<const char*, 6> mServoAddressTexts {
        "Set Address",
        "Start ...",
        "... End"
    };
    static inline bool setServoAdrCb(const uint8_t) {
        return false;
    }

    static inline std::array<const char*, 6> mServoCalibTexts {
        "Set Address",
        "Start ...",
        "... End"
    };
    static inline bool servoCalibCb(const uint8_t) {
        return false;
    }

    using i2c_strings_t = std::array<std::array<char, 16>, 4>;
    static inline auto mI2CDevs = []{
        i2c_strings_t s;
        for(auto& d : s) {
            strcpy(&d[0], "---");
        }
        return s;
    }();

    using cal_strings_t = std::array<std::array<char, 64>, 3>;
    static inline constexpr void updateCalib(cal_strings_t& t) {
        for(uint8_t i = 0; ((i < t.size()) && (i < eeprom.compass_calib.size())); ++i) {
            snprintf(&t[i][0], t[i].size(), "[m: %d, s: %d]", eeprom.compass_calib[i].mean, eeprom.compass_calib[i].d);
        }
    }
    static inline auto mCalTexts = []{
        cal_strings_t s;
        updateCalib(s);
        return s;
    }();

    static inline void btBaudrate(const store_t v) {
        switch(v) {
        case 0:
            bluetooth::baud(9600);
            break;
        case 1:
            bluetooth::baud(57600);
            break;
        case 2:
            bluetooth::baud(115200);
            break;
        default:
            bluetooth::baud(9600);
            break;
        }
    }

    static inline std::array<char, 16> mDeadLowString{'a'};

    static inline void hide(const uint8_t n, const bool b) {
        params[n].hide(b);
    }
    static inline void hide(const uint8_t start, const uint8_t end, const bool b) {
        for(uint8_t i = start; i <= end; ++i) {
            hide(i, b);
        }
    }
    using params_t = etl::FixedVector<Param_t, 200>;
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Folder, ""});
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::Sel,  "Mode", "Dual-Schottel-Controller;Cruise-Controller", &eeprom.mode, 0, 1, [](const store_t){return true;}});
        uint8_t parent = addParent(p, Param_t{0, PType::Folder, "Channels"});
        addNode(p, Param_t{.parent = parent, .type = PType::Sel, .name = "Stream", .options = "Main/CRSF;Alternative;SBus/Aux;Bluetooth/Aux", .value_ptr = &eeprom.input_stream, .max = 3, .cb = [](const store_t s){mapper::stream(s); return true;}});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Schottel 1: f/b", .value_ptr = &eeprom.channels[0].first, .max = 15, .cb = [](const store_t){return true;}});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Schottel 1: l/r", .value_ptr = &eeprom.channels[0].second, .max = 15, .cb = [](const store_t){return true;}});
        addNode(p, Param_t{parent, PType::U8, "Schottel 2: f/b", nullptr, &eeprom.channels[1].first, 0, 15, [](const store_t){return true;}});
        addNode(p, Param_t{parent, PType::U8, "Schottel 2: l/r", nullptr, &eeprom.channels[1].second, 0, 15, [](const store_t){return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Outputs"});
        addNode(p, Param_t{parent, PType::Sel, "Srv1 Out", "PWM/Analog;PWM/PWM;Serial/WaveShare;PWM/None;MultiSwitch/Graupner-A;None", &eeprom.out_mode_srv[0], 0, 5, [](const store_t s){servos::template servo<0>(s); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Srv1 Fb", "Analog;PWM;WaveShare;None", &eeprom.out_mode_srv[0], 0, 3});
#ifdef ESCAPE32_ASCII
        addNode(p, Param_t{parent, PType::Sel, "Esc1 Out", "PWM/-;ESCape32/Serial;ESCape32/Ascii;VEsc/Serial;None", &eeprom.out_mode_esc[0], 0, 4, [](const store_t s){escs::template esc<0>(s); if (s == 2) {hide(mESCape321Folder, mESCape321End, false);} else {hide(mESCape321Folder, mESCape321End, true);} return true;}});
#else
        addNode(p, Param_t{parent, PType::Sel, "Esc1 Out", "PWM/-;ESCape32/Serial;ESCape32/Ascii;VEsc/Serial;None", &eeprom.out_mode_esc[0], 0, 4, [](const store_t s){escs::template esc<0>(s); return true;}});
#endif
#ifdef SERIAL_DEBUG
        addNode(p, Param_t{parent, PType::Sel, "Esc1 Tlm", "Debug;S.Port;VEsc/Bidirectional", &eeprom.tlm_mode_esc[0], 0, 4, [](const store_t){return true;}});
#else
        addNode(p, Param_t{parent, PType::Sel, "Esc1 Tlm", "S.Port;VEsc/Bidirectional", &eeprom.tlm_mode_esc[0], 0, 4, [](const store_t){return true;}});
#endif
        addNode(p, Param_t{parent, PType::Sel, "Srv2 Out", "PWM/Analog;PWM/PWM;Serial/WaveShare;PWM/None;None", &eeprom.out_mode_srv[1], 0, 4, [](const store_t s){servos::template servo<1>(s); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Srv2 Fb", "Analog;PWM;WaveShare;None", &eeprom.out_mode_srv[1], 0, 3});
#ifdef ESCAPE32_ASCII
        addNode(p, Param_t{parent, PType::Sel, "Esc2 Out", "PWM/-;ESCape32/Serial;ESCape32/Ascii;VEsc/Serial;None;Bluetooth", &eeprom.out_mode_esc[1], 0, 5, [](const store_t s){escs::template esc<1>(s); if (s == 2) {hide(mESCape322Folder, false);} return true;}});
#else
        addNode(p, Param_t{parent, PType::Sel, "Esc2 Out", "PWM/-;ESCape32/Serial;ESCape32/Ascii;VEsc/Serial;None;Bluetooth", &eeprom.out_mode_esc[1], 0, 5, [](const store_t s){escs::template esc<1>(s); return true;}});
#endif
        addNode(p, Param_t{parent, PType::Sel, "Esc2 Tlm", "S.Port;VEsc/Bidirectional", &eeprom.tlm_mode_esc[1], 0, 2, [](const store_t){return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Settings"});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Sch1 ESC Deadband", .value_ptr = &eeprom.deadbands[0], .min = 1, .max = 20, .cb = [](const store_t){return true;}, .unitString = "%"});
        addNode(p, Param_t{.parent = parent, .type = PType::I8, .name = "Sch1 ESC PWM Mid", .value_ptr = &eeprom.esc_mid[0], .min = (store_t)-100, .max = 100, .cb = [](const store_t){return true;}, .unitString ="us"});
        addNode(p, Param_t{.parent = parent, .type = PType::I8, .name = "Sch1 Servo zPosition", .value_ptr = &eeprom.offset1, .min = (store_t)-90, .max = 90, .cb = [](const store_t v){servos::template offset<0>((uint32_t{v} * 4096) / 360); return true;}, .unitString ="°"});
        addNode(p, Param_t{parent, PType::U8,  "Sch1 Servo Speed", nullptr, &eeprom.speed1, 1, 100, [](const store_t v){servos::template speed<0>(34 * v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Sch2 ESC Deadband", nullptr, &eeprom.deadbands[1], 1, 20, [](const store_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Sch2 ESC PWM Mid", nullptr, &eeprom.esc_mid[1], 0, 200, [](const store_t){return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Sch2 Servo zPosition", nullptr,  &eeprom.offset2, 0, 359, [](const store_t v){servos::template offset<1>((uint32_t{v} * 4096) / 360); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Sch2 Servo Speed", nullptr, &eeprom.speed2, 1, 100, [](const store_t v){servos::template speed<1>(34 * v); return true;}});
        addNode(p, Param_t{parent, PType::Command, "Act. Pos. as zPos 1", setServoZeroPosText, nullptr, 0, 0, [](const store_t v){return setZeroPosition(v);}});
        addNode(p, Param_t{parent, PType::Command, "Act. Pos. as zPos 2", setServoZeroPosText, nullptr, 0, 0, [](const store_t v){return setZeroPosition(v);}});

#ifdef ESCAPE32_ASCII
        parent = addParent(p, Param_t{0, PType(PType::Folder | PType::Hidden), "ESCape32 1"});
        mESCape321Folder = parent;
        addNode(p, Param_t{parent, PType::Command, "Beep", nullptr, nullptr, 0, 0, [](const store_t){esc32ascii_1::beep(); return false;}});
        addNode(p, Param_t{parent, PType::Command, "Save", nullptr, nullptr, 0, 0, [](const store_t){esc32ascii_1::save(); return false;}});
        [&]<size_t... II>(std::integer_sequence<size_t, II...>){
            (addNode(p, Param_t{parent, PType::U16, esc32ascii_1::params()[II].name, nullptr, &esc32ascii_1::params()[II].value, esc32ascii_1::params()[II].min, esc32ascii_1::params()[II].max, [](const store_t v){esc32ascii_1::setParam(II, v); return false;}}), ...);
        }(std::make_index_sequence<esc32ascii_1::params().size()>{});
        mESCape321End = p.size() - 1;

        // parent = addParent(p, Param_t{0, PType(PType::Folder | PType::Hidden), "ESCape32 2"});
        // mESCape322Folder = parent;
        // addNode(p, Param_t{parent, PType::Command, "Beep", nullptr, nullptr, 0, 0, [](const store_t){esc32ascii_1::beep(); return false;}});
        // addNode(p, Param_t{parent, PType::Command, "Save", nullptr, nullptr, 0, 0, [](const store_t){esc32ascii_1::save(); return false;}});

        // [&]<size_t... II>(std::integer_sequence<size_t, II...>){
        //     (addNode(p, Param_t{parent, PType::U16, esc32ascii_2::params()[II].name, nullptr, &esc32ascii_2::params()[II].value, esc32ascii_2::params()[II].min, esc32ascii_2::params()[II].max, [](const store_t v){esc32ascii_2::setParam(II, v); return false;}}), ...);
        // }(std::make_index_sequence<esc32ascii_2::params().size()>{});

        // mESCape322End = p.size() - 1;
#endif

#ifdef SERVO_CALIBRATION
        parent = addParent(p, Param_t{0, PType::Folder, "Calibration"});
        mServoCalibCommand = parent;
        addNode(p, Param_t{parent, PType::Command, "Calibrate", mServoCalibTexts[0], nullptr, 0, 0, [](const store_t v){return servoCalibCb(v);}});
        addNode(p, Param_t{parent, PType::Info, "Srv1 DeadL", &mDeadLowString[0]});
#endif

        parent = addParent(p, Param_t{0, PType::Folder, "CRSF"});
        addNode(p, Param_t{parent, PType::U8,  "CRSF Address", nullptr, &eeprom.address, 192, 207, [](const store_t a){updateName(mName); src::address(std::byte(a)); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Mode (not persistant)", "Full;Fwd Only", nullptr, 0, 1, [](const store_t){return false;}});
        parent = addParent(p, Param_t{0, PType::Folder, "Advanced"});
        addNode(p, Param_t{parent, PType::Sel, "Crsf-HD/SBus", "SBus/Out;Crsf;SBus2/Master;CPPM/N;CPPM/P;CombinedPWMChannels/P;IBus/In;SBus/In;SumDV3/In;SumDV3/Out;None", &eeprom.crsf_hd_mode, 0, 10, [](const store_t r){relays::set(r); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Crsf-FD/Aux", "Crsf;GPS;SBus/S.Port;Bluetooth;None", &eeprom.crsf_fd_aux_mode, 0, 4, [](const store_t a){
                               auxes::set(a);
                               return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Inject (SBus)", "Yes;No", &eeprom.inject, 0, 1, [](const store_t){return true;}});
#ifdef SERVO_ADDRESS_SET
        addNode(p, Param_t{parent, PType::Command, "Set Servo ID", mServoAddressTexts[0], nullptr, 0, 0, [](const store_t v){return setServoAdrCb(v);}});
        mServoAddressCommand = p.size() - 1;
        addNode(p, Param_t{parent, PType::U8,  "Servo ID to set", nullptr, nullptr, 1, 16, [](const store_t){return false;}});
#endif
        addNode(p, Param_t{parent, PType::U8,  "Switch Address", nullptr, &eeprom.switchAddress, 0, 255, [](const store_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Contiguous Switch Addresses", nullptr, &eeprom.switchAddressContiguous, 0, 1, [](const store_t){return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "S.Port"});
        addNode(p, Param_t{parent, PType::U8,  "Physical-ID", nullptr, &eeprom.sport_physicalId_switch, 0, 0x1b, [](const store_t a){sport_aux::setPhysID0(a); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "App-ID Switch", nullptr, &eeprom.sport_appId_switch, 0x00, 0xff, [](const store_t a){sport_aux::setAppID0(a); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Physical-ID", nullptr, &eeprom.sport_physicalId_telemetry, 0, 0x1b, [](const store_t a){sport_aux::setPhysID1(a); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "App-ID Telemetry", nullptr, &eeprom.sport_appId_telemetry, 0x00, 0xff, [](const store_t a){sport_aux::setAppID1(a); return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "I2C"});
        addNode(p, Param_t{parent, PType::Info, "Address 0:", &mI2CDevs[0][0]});
        addNode(p, Param_t{parent, PType::Info, "Address 1:", &mI2CDevs[1][0]});
        addNode(p, Param_t{parent, PType::Info, "Address 2:", &mI2CDevs[2][0]});
        addNode(p, Param_t{parent, PType::Info, "Address 3:", &mI2CDevs[3][0]});

        parent = addParent(p, Param_t{0, PType::Folder, "Compass"});
        addNode(p, Param_t{parent, PType::Command, "Calibrate", mCompassCalibTexts[0], nullptr, 0, 0, [](const store_t v){return compassCalibCb(v);}, 200});
        mCompassCalibCommand = p.size() - 1;
        addNode(p, Param_t{parent, PType::Info, "Cal. x-axis", &mCalTexts[0][0]});
        addNode(p, Param_t{parent, PType::Info, "Cal. y-axis", &mCalTexts[1][0]});
        addNode(p, Param_t{parent, PType::Info, "Cal. z-axis", &mCalTexts[2][0]});

        parent = addParent(p, Param_t{0, PType::Folder, "BlueTooth"});
        addNode(p, Param_t{parent, PType::Sel, "Baudrate", "9600;57600;115200", &eeprom.bt_baudrate, 0, 2, [](const store_t v){ btBaudrate(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Telm. Voltage 0", "off;on", &eeprom.bt_telem_voltage0, 0, 1, [](const store_t){return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Telm. Voltage 1", "off;on", &eeprom.bt_telem_voltage1, 0, 1, [](const store_t){return true;}});
        addNode(p, Param_t{.parent = parent, .type = PType::U16, .name = "Voltage Alarm", .value_ptr = &eeprom.bt_telem_voltage_thresh, .min = 0, .max = 300, .cb = [](const store_t){return true;}, .unitString = " *0.1V"});

        return p;
    }();
    static inline const uint32_t uuid = Mcu::Stm::Uuid::get();
};

