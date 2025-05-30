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

// #define TEST1 // use only test elrs menu
// #define USE_TP1 // enable test point
// #define USE_AUTO_CONF
// #define USE_MORSE
#define USE_EEPROM_TEST // switches telemetry default on (instead off)
#define USE_BUTTON
#define SERIAL_DEBUG // use with care (e.g. with USE_MORSE) because of RAM overflow

#define NDEBUG // do not change: dev option

#define HW_VERSION 1
#define SW_VERSION 1

#include <cstdint>
#include <array>

#include "devices.h"

#define EEPROM_MAGIC 42

struct EEProm {
    constexpr EEProm() {
        etl::copy("Output 0", outputs[0].name);
        etl::copy("Output 1", outputs[1].name);
        etl::copy("Output 2", outputs[2].name);
        etl::copy("Output 3", outputs[3].name);
        etl::copy("Output 4", outputs[4].name);
        etl::copy("Output 5", outputs[5].name);
        etl::copy("Output 6", outputs[6].name);
        etl::copy("Output 7", outputs[7].name);
#ifdef USE_MORSE
        etl::copy("SOS", morse_text);
#endif
    }
    struct Output {
        uint8_t pwm = 0;
        uint8_t pwmDuty = 1;
        uint8_t pwmScale = 0;
        uint8_t blink = 0;
        uint8_t blinkOnTime = 1;
        uint8_t blinkOffTime = 1;
        std::array<char, 16> name;
#ifdef USE_AUTO_CONF
        uint8_t ls = 0;
        uint8_t type = 0;
        uint8_t flags = 0;
#endif
    };

    uint8_t magic = EEPROM_MAGIC;

    uint8_t address = 0;
#ifdef USE_EEPROM_TEST
    uint8_t crsf_address = 0xcf;
    uint8_t response_slot = 16; // formula?
#else
    uint8_t crsf_address = 0xc8;
    uint8_t response_slot = 8;
#endif
#ifdef USE_EEPROM_TEST
    uint8_t telemetry = 1;
#else
    uint8_t telemetry = 0;
#endif
    std::array<Output, 8> outputs{};
#ifdef USE_MORSE
    std::array<char, 64> morse_text{};
    uint8_t morse_dit = 3;
    uint8_t morse_dah = 6;
    uint8_t morse_gap = 3;
    uint8_t morse_igap = 3;
#endif
};

struct Storage {
    static inline void init() {
        std::memcpy(&eeprom, &eeprom_flash, sizeof(EEProm));
    }
    static inline void reset() {
        eeprom = EEProm{};
    }
    __attribute__((__section__(".eeprom")))
    static inline const EEProm eeprom_flash{};
    __attribute__ ((aligned (8)))
    static inline EEProm eeprom;
};

using namespace std::literals::chrono_literals;

template<typename Config, typename SwitchCallback,
         typename Trace = void>
struct CrsfCallback {
    using trace = Trace;
    using debug = trace;
    // using debug::_;
    using Param_t = RC::Protokoll::Crsf::V4::Parameter<uint8_t>;
    using PType = Param_t::Type;

    using timer = Config::timer;
    using crsf = Config::crsf;
    using messageBuffer = crsf::messageBuffer;

    static_assert(sizeof(EEProm) < 2048);
    static inline constexpr auto& eeprom = Storage::eeprom;
    static inline constexpr auto& eeprom_flash = Storage::eeprom_flash;

    static inline constexpr const char* const title = "Led4x4-E@";

    using name_t = std::array<char, 32>;

    enum class State : uint8_t {Undefined, GotStats, GotChannels};

    static inline State mStreamState{State::Undefined};

    static inline constexpr void disableTelemetry() {
    }
    static inline constexpr void gotLinkStats() {
    }
    static inline constexpr void gotChannels() {
    }
    static inline constexpr void forwardPacket(const auto /*data*/, const uint16_t /*length*/) {
    }
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
        if (index == 0) return;
        if (index < params.size()) {
            bool mustSave = true;
            if (params[index].type == Param_t::Str) {
                IO::outl<debug>("# String");
                if (params[index].stringValue) {
                    for(uint8_t i = 0; (i < 16) && (i < paylength); ++i) {
                        params[index].stringValue[i] = data[i];
                        if (data[i] == '\0') {
                            break;
                        }
                    }
                    params[index].stringValue[paylength] = '\0';
                }
            }
            else {
                Param_t::value_type value{};
                if (params[index].type <= Param_t::I8) {
                    value = data[0];
                    IO::outl<debug>("# I8: v: ", value);
                }
                else if (params[index].type <= Param_t::I16) {
                    value = (data[0] << 8) + data[1];
                    IO::outl<debug>("# I16: v: ", value);

                }
                else if (params[index].type <= Param_t::F32) {
                    value = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + data[0];
                    IO::outl<debug>("# F32: v: ", value);
                }
                else if (params[index].type == Param_t::Sel) {
                    value = data[0];
                    IO::outl<debug>("# Sel: v: ", value);
                }
                params[index].value(value);
                if (params[index].cb) {
                    mustSave = params[index].cb(value);
                }
            }
            update();
            if (mustSave) {
                save();
            }
        }
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
                        SwitchCallback::set(sw);
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
                        SwitchCallback::set(sw8);
                    }
                    else if (cmd == (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::Prop) {
                        const uint8_t ch = (uint8_t)payload[8];
                        const uint8_t duty = (uint8_t)payload[9];
                        IO::outl<trace>("# Cmd Prop: ", address, " ch: ", ch, " d: ", duty);
                        SwitchCallback::prop(ch, duty);
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
#ifdef USE_AUTO_CONF
        addNode(p, Param_t{.parent = parent, .type = PType::Str, .name = "Alias", .units = 8, .stringValue = &eeprom.outputs[0].name[0]}); // not supported by elrsv3.lua?
#endif
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM Duty", .value_ptr = &eeprom.outputs[0].pwmDuty, .min = 1, .max = 99, .cb = [](const uint8_t v){return true;}, .unitString = "%"});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t v){return false;}});
#ifdef USE_AUTO_CONF
        addNode(p, Param_t{parent, PType::U8,  "Log. Switch", nullptr, &eeprom.outputs[0].ls, 1, 64, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Type", "Toggle;Momentary;Proportional", &eeprom.outputs[0].type, 0, 2, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Popup", "None;Slider", &eeprom.outputs[0].flags, 0, 2, [](const uint8_t){return true;}});
#endif
#ifdef USE_MORSE
        parent = addParent(p, Param_t{0, PType::Folder, "Morse"});
        addNode(p, Param_t{parent, PType::Str, "Text", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &eeprom.morse_text[0]}); // not supported by elrsv3.lua?
        addNode(p, Param_t{parent, PType::U8,  "Dit duration", nullptr, &eeprom.morse_dit, 1, 10, [](const uint8_t v){
                               Meta::visit<bsws>([&]<typename T>(Meta::Wrapper<T>){
                                   T::morse_dit_dezi(v);
                               }); return true;}, 1, 0, 1, nullptr, " [100ms]"});
        addNode(p, Param_t{parent, PType::U8,  "Dah duration", nullptr, &eeprom.morse_dah, 1, 10, [](const uint8_t v){
                               Meta::visit<bsws>([&]<typename T>(Meta::Wrapper<T>){
                                   T::morse_dah_dezi(v);
                               });
                               return true;}, 1, 0, 1, nullptr, " [100ms]"});
        addNode(p, Param_t{parent, PType::U8,  "Intra S. Gap dur.", nullptr, &eeprom.morse_gap, 1, 10, [](const uint8_t v){
                               Meta::visit<bsws>([&]<typename T>(Meta::Wrapper<T>){
                                   T::morse_gap_dezi(v);
                               });
                               return true;}, 1, 0, 1, nullptr, " [100ms]"});
        addNode(p, Param_t{parent, PType::U8,  "Inter S. Gap dur.", nullptr, &eeprom.morse_igap, 1, 10, [](const uint8_t v){
                               Meta::visit<bsws>([&]<typename T>(Meta::Wrapper<T>){
                                   T::morse_igap_dezi(v);
                               });
                               return true;}, 1, 0, 1, nullptr, " [100ms]"});
#endif
        parent = addParent(p, Param_t{0, PType::Folder, "Operate"});
        addNode(p, Param_t{parent, PType::Sel, "Output 0", "Off;On", 0, 0, 1, [](const uint8_t v){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 1", "Off;On", 0, 0, 1, [](const uint8_t v){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 2", "Off;On", 0, 0, 1, [](const uint8_t v){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 3", "Off;On", 0, 0, 1, [](const uint8_t v){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 4", "Off;On", 0, 0, 1, [](const uint8_t v){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 5", "Off;On", 0, 0, 1, [](const uint8_t v){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 6", "Off;On", 0, 0, 1, [](const uint8_t v){return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 7", "Off;On", 0, 0, 1, [](const uint8_t v){return false;}});
#endif
        if (p.size() >= p.capacity()) {
            void f();
            f();
        }
        return p;
    }();
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using crsf = devs::crsf;
    using crsf_out = crsf::output;
    using crsf_pa = crsf::input;
    using led = devs::ledBlinker;
#ifdef USE_BUTTON
    using btn = devs::btn;
#endif

    using debug = devs::debug;
    using crsfCallback = crsf::callback;

    enum class State : uint8_t {Undefined, Init,
                                RunNoTelemetry, RunWithTelemetry,
                                NotConnected};

    enum class Event : uint8_t {None, ConnectionLost, DirectConnected, ReceiverConnected};

    static inline constexpr External::Tick<systemTimer> packagesCheckTicks{300ms};
    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};

    static inline void update(const bool eepromMode = true) {
        crsfCallback::callbacks(eepromMode);
    }
    static inline void prop(const uint8_t channel, const uint8_t duty) {
        IO::outl<debug>("# prop: ", channel, " duty: ", duty);
    }
    static inline void set(const uint8_t sw) {
        IO::outl<debug>("# set: ", sw);
    }
    static inline void init() {
        devs::init();
        if constexpr(!std::is_same_v<debug, void>) {
            debug::init();
        }
    }
    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void periodic() {
        crsf::periodic();
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
    }
    static inline void ratePeriodic() {
        crsf::ratePeriodic();
        led::ratePeriodic();
#ifdef USE_BUTTON
        btn::ratePeriodic();
#endif
        (++mPackagesCheckTick).on(packagesCheckTicks, []{
            const uint16_t ch_p = crsf_pa::template channelPackages<true>();
            const uint16_t l_p = crsf_pa::template linkPackages<true>();
            if (ch_p > 0) {
                if  (l_p == 0) {
                    event(Event::DirectConnected);
                }
                else {
                    event(Event::ReceiverConnected);
                }
            }
            else {
                event(Event::ConnectionLost);
            }
        });

        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                if (Storage::eeprom.telemetry) {
                    mState = State::RunWithTelemetry;
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            });
            break;
        case State::RunNoTelemetry:
            mStateTick.on(debugTicks, []{
                IO::outl<debug>("# ch0: ", crsf_pa::value(0), " cp: ", crsf_pa::template channelPackages<false>(), " lp: ", crsf_pa::template linkPackages<false>());
                // IO::outl<debug>("# ch0: ", crsf_pa::value(0));
            });
#ifdef USE_BUTTON
            if (const auto e = btn::event(); e == btn::Press::Long) {
                mState = State::RunWithTelemetry;
            }
#endif
            if (mEvent.is(Event::ConnectionLost)) {
                mState = State::NotConnected;
            }
            break;
        case State::RunWithTelemetry:
            mStateTick.on(debugTicks, []{
                IO::outl<debug>("# ch0: ", crsf_pa::value(0), " psize: ", crsfCallback::numberOfParameters(), " cp: ", crsf_pa::template channelPackages<false>(), " lp: ", crsf_pa::template linkPackages<false>());
            });
#ifdef USE_BUTTON
            if (const auto e = btn::event(); e == btn::Press::Long) {
                if (Storage::eeprom.telemetry) {
                    led::count(2);
                    led::event(led::Event::Slow);
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            }
#endif
            if (mEvent.is(Event::ConnectionLost)) {
                IO::outl<debug>("# E CL");
                mState = State::NotConnected;
            }
            break;
        case State::NotConnected:
            if (mEvent.is(Event::ReceiverConnected) || mEvent.is(Event::DirectConnected)) {
                if (Storage::eeprom.telemetry) {
                    mState = State::RunWithTelemetry;
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            }
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                IO::outl<debug>("# Init eep magic: ", Storage::eeprom.magic);
                break;
            case State::RunNoTelemetry:
                IO::outl<debug>("# Run NT");
                IO::outl<debug>("# adr: ", Storage::eeprom.address);
                crsf_out::enableReply(false);
                led::count(1);
                led::event(led::Event::Slow);
                break;
            case State::RunWithTelemetry:
                IO::outl<debug>("# Run WT");
                IO::outl<debug>("# adr: ", Storage::eeprom.address);
                crsf_out::enableReply(true);
                if (Storage::eeprom.telemetry) {
                    led::count(2);
                    led::event(led::Event::Slow);
                }
                else {
                    led::event(led::Event::Steady);
                }
                break;
            case State::NotConnected:
                IO::outl<debug>("# Run NC");
                led::count(1);
                led::event(led::Event::Fast);
                break;
            }
        }
    }
    private:
    static inline etl::Event<Event> mEvent;
    static inline External::Tick<systemTimer> mPackagesCheckTick;
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState = State::Undefined;
};

struct Setter;

template<typename L, typename T>
using CrsfCallback_WithSetter = CrsfCallback<L, Setter, T>;

using devs = Devices<Led01, CrsfCallback_WithSetter, Storage>;
using gfsm = GFSM<devs>;

struct Setter {
    static inline void set(const uint8_t sw) {
        gfsm::set(sw);
    }
    static inline void prop(const uint8_t channel, const uint8_t duty) {
        gfsm::prop(channel, duty);
    }
};

int main() {
    Storage::init();
    gfsm::init();
    gfsm::update(true);

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(SPI1_IRQn);
    __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

extern "C" {

void USART1_IRQHandler(){
    using crsf = devs::crsf;
    static_assert(crsf::number == 1);
    crsf::Isr::onTransferComplete([]{});
    crsf::Isr::onIdle([]{});
}
void SPI1_IRQHandler(){
    using spi = devs::pca9745::spi;
    static_assert(spi::number == 1);
    spi::Isr::onTransferComplete([]{});
}
void HardFault_Handler() {
    while(true) {
#ifdef USE_TP1
        devs::tp1::set();
        devs::tp1::reset();
#endif
    }
}
}

void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}
