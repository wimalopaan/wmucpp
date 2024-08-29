#pragma once

#include "telemetry.h"

#ifdef USE_DEVICES3

template<typename Config, typename SwitchCallback,
         typename Trace = void>
struct CrsfCallback {
    using trace = Trace;
    using Param_t = RC::Protokoll::Crsf::Parameter<>;
    using PType = RC::Protokoll::Crsf::Parameter<>::Type;
    using telem_out = Config::telem_out;
    using timer = Config::timer;
    using tlc_1 = Config::tlc1;
    using storage = Config::storage;
    using crsfTelemetry = CrsfTelemetry<telem_out, timer, storage>;

    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;

    static inline constexpr const char* const title = "CruiseControl @ ";

    using name_t = std::array<char, 32>;

    // SM: only sending telemetry after getting own command
    // own-command -> channels -> send-telemetry
    // this avoids (mostly) collisions in half-duplex
    //
    // in full-duplex this acts as a request: the crsf-fd-switch
    // must leave the slot free

    enum class State : uint8_t {Undefined, GotStats, GotChannels};

    static inline State mStreamState{State::Undefined};

    static inline constexpr void gotLinkStats() {
        if (mStreamState == State::Undefined) {
            mStreamState = State::GotStats;
        }
    }

    static inline constexpr void gotChannels() {
        if (mStreamState == State::GotStats) {
            mStreamState = State::GotChannels;
            crsfTelemetry::event(crsfTelemetry::Event::SendNext);
        }
    }

    static inline constexpr void ratePeriodic() {
        crsfTelemetry::ratePeriodic();
    }

    static inline constexpr void updateName(name_t& n) {
        strncpy(&n[0], title, n.size());
        std::to_chars(std::begin(n) + strlen(title), std::end(n), eeprom.address);
    }
    static inline void update() {
        updateName(mName);
    }

    static inline void save() {
        if (Mcu::Stm32::savecfg(eeprom, eeprom_flash)) {
            IO::outl<trace>("EEPROM OK");
        }
        else {
            IO::outl<trace>("EEPROM NOK");
        }
    }
    static inline void setParameter(const uint8_t index, const uint8_t value) {
        IO::outl<trace>("SetP i: ", index, " v: ", value);
        if ((index >= 1) && (index <= params.size())) {
            params[index - 1].value(value);
            mLastChangedParameter = index;
            bool mustSave = true;
            if (params[index - 1].cb) {
                mustSave = params[index - 1].cb(value);
            }
            update();
            if (mustSave) {
                save();
            }
        }
    }
    static inline RC::Protokoll::Crsf::Parameter<> parameter(const uint8_t index) {
        if ((index >= 1) && (index <= params.size())) {
            return params[index - 1];
        }
        return {};
    }
    static inline bool isCommand(const uint8_t index) {
        if ((index >= 1) && (index <= params.size())) {
            return params[index - 1].mType == PType::Command;
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
        return params.size();
    }
    static inline uint8_t protocolVersion() {
        return 0;
    }
    static inline void whenParameterChanged(auto f) {
        if (mLastChangedParameter > 0) {
            f(mLastChangedParameter);
            mLastChangedParameter = 0;
        }
    }
    template<auto L>
    static inline void command(const std::array<uint8_t, L>& payload) {
        if (payload[0] == (uint8_t)RC::Protokoll::Crsf::Address::Controller) {
            if (payload[2] == (uint8_t)RC::Protokoll::Crsf::CommandType::Switch) {
                if (payload[3] == (uint8_t)RC::Protokoll::Crsf::SwitchCommand::Set) {
                    const uint8_t address = (uint8_t)payload[4];
                    const uint8_t sw = (uint8_t)payload[5];
                    if (eeprom.address == address) {
                        IO::outl<trace>("Command: ", address, " sw: ", sw);
                        SwitchCallback::set(sw);
                    }
                    else if (eeprom.tlc_1_address == address) {
                        IO::outl<trace>("Command: ", address, " sw: ", sw);
                        for(uint8_t i = 0; i < 8; ++i) {
                            if (sw & (0b01 << i)) {
                                if (eeprom.tlc_1[i].mode == 0) {
                                    tlc_1::on(i);
                                }
                                else {
                                    tlc_1::pwmon(i);
                                }
                            }
                            else {
                                tlc_1::off(i);
                            }
                        }
                    }
                }
            }
        }
    }
    static inline void callbacks() {
        for(const auto p: params) {
            if (p.cb) {
                p.cb(p.value());
            }
        }
    }
    static inline bool bootMode = true;

    static inline name_t mVescName{};
    static inline name_t mVescFirmware{};

    private:
    static inline uint8_t mLastChangedParameter{};
    static inline constexpr uint32_t mSerialNumber{1234};
    static inline constexpr uint32_t mHWVersion{2};
    static inline constexpr uint32_t mSWVersion{10};
    static inline constexpr auto mVersionString = [](){
        std::array<char, 16> s{};
        auto r = std::to_chars(std::begin(s), std::end(s), mHWVersion);
        *r.ptr++ = ':';
        r  = std::to_chars(r.ptr, std::end(s), mSWVersion);
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
        return c.size();
    }
    static inline void addNode(auto& c, const Param_t& p) {
        c.push_back(p);
    }

    static inline void testWrapper(const uint8_t n, const uint8_t v) {
        if (bootMode) return;
        if (v == 0) {
            tlc_1::off(n);
        }
        else {
            if (eeprom.tlc_1[n].mode == 0) {
                tlc_1::on(n);
            }
            else {
                tlc_1::pwmon(n);
            }
        }
    }
    using params_t = etl::FixedVector<Param_t, 128>;
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        auto parent = addParent(p, Param_t{0, PType::Folder, "Global"});
        addNode(p, Param_t{parent, PType::U8, "Switch Address", nullptr, &eeprom.address, 0, 255});

        parent = addParent(p, Param_t{0, PType::Folder, "PWM Outputs"});
        {
            auto lparent = addParent(p, Param_t{parent, PType::Folder, "Output 0"});
            addNode(p, Param_t{lparent, PType::Info, "Output 0 : ", &mName[0]});
            addNode(p, Param_t{lparent, PType::Sel, "Mode", "Pwm;Switch", &eeprom.outputs[0].mode, 0, 1});
            addNode(p, Param_t{lparent, PType::U8,  "PWM Mode Channel", nullptr, &eeprom.outputs[0].channel, 1, 16});
            addNode(p, Param_t{lparent, PType::U8,  "Switch Mode Nr", nullptr, &eeprom.outputs[0].sw, 0, 7});

            lparent = addParent(p, Param_t{parent, PType::Folder, "Output 1"});
            addNode(p, Param_t{lparent, PType::Info, "Output 1 : ", &mName[0]});
            addNode(p, Param_t{lparent, PType::Sel, "Mode", "Pwm;Switch", &eeprom.outputs[1].mode, 0, 1});
            addNode(p, Param_t{lparent, PType::U8,  "PWM Mode Channel", nullptr, &eeprom.outputs[1].channel, 1, 16});
            addNode(p, Param_t{lparent, PType::U8,  "Switch Mode Nr", nullptr, &eeprom.outputs[1].sw, 0, 7});

            lparent = addParent(p, Param_t{parent, PType::Folder, "Output 2"});
            addNode(p, Param_t{lparent, PType::Info, "Output 2 : ", &mName[0]});
            addNode(p, Param_t{lparent, PType::Sel, "Mode", "Pwm;Switch", &eeprom.outputs[2].mode, 0, 1});
            addNode(p, Param_t{lparent, PType::U8,  "PWM Mode Channel", nullptr, &eeprom.outputs[2].channel, 1, 16});
            addNode(p, Param_t{lparent, PType::U8,  "Switch Mode Nr", nullptr, &eeprom.outputs[2].sw, 0, 7});

            lparent = addParent(p, Param_t{parent, PType::Folder, "Output 3"});
            addNode(p, Param_t{lparent, PType::Info, "Output 3 : ", &mName[0]});
            addNode(p, Param_t{lparent, PType::Sel, "Mode", "Pwm;Switch", &eeprom.outputs[3].mode, 0, 1});
            addNode(p, Param_t{lparent, PType::U8,  "PWM Mode Channel", nullptr, &eeprom.outputs[3].channel, 1, 16});
            addNode(p, Param_t{lparent, PType::U8,  "Switch Mode Nr", nullptr, &eeprom.outputs[3].sw, 0, 7});
        }

        parent = addParent(p, Param_t{0, PType::Folder, "Externals"});
        {
            auto lparent = addParent(p, Param_t{parent, PType::Folder, "Switch Ctrl 0"});
            {
                addNode(p, Param_t{lparent, PType::U8,  "Address", nullptr, &eeprom.tlc_1_address, 0, 255});
                addNode(p, Param_t{lparent, PType::Sel, "Group-Mode", "1;2;4;8", &eeprom.tlc_1_groupmode, 0, 3, [](uint8_t v){tlc_1::grouping(v); return true;}});
                {
                    auto llparent = addParent(p, Param_t{lparent, PType::Folder, "Output 0"});
                    addNode(p, Param_t{llparent, PType::Info, "Output 0 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{llparent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[0].mode, 0, 1});
                    addNode(p, Param_t{llparent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[0].pwmDuty, 1, 99, [](uint8_t v){tlc_1::pwm(0, v); return true;}});
                    addNode(p, Param_t{llparent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](uint8_t v){testWrapper(0, v); return false;}});
                }
                {
                    auto llparent = addParent(p, Param_t{lparent, PType::Folder, "Output 1"});
                    addNode(p, Param_t{llparent, PType::Info, "Output 1 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{llparent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[1].mode, 0, 1});
                    addNode(p, Param_t{llparent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[1].pwmDuty, 1, 99, [](uint8_t v){tlc_1::pwm(1, v); return true;}});
                    addNode(p, Param_t{llparent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](uint8_t v){testWrapper(1, v); return false;}});
                }
                {
                    auto llparent = addParent(p, Param_t{lparent, PType::Folder, "Output 2"});
                    addNode(p, Param_t{llparent, PType::Info, "Output 2 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{llparent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[2].mode, 0, 1});
                    addNode(p, Param_t{llparent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[2].pwmDuty, 1, 99, [](uint8_t v){tlc_1::pwm(2, v); return true;}});
                    addNode(p, Param_t{llparent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](uint8_t v){testWrapper(2, v); return false;}});
                }
                {
                    auto llparent = addParent(p, Param_t{lparent, PType::Folder, "Output 0"});
                    addNode(p, Param_t{llparent, PType::Info, "Output 3 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{llparent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[3].mode, 0, 1});
                    addNode(p, Param_t{llparent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[3].pwmDuty, 1, 99, [](uint8_t v){tlc_1::pwm(3, v); return true;}});
                    addNode(p, Param_t{llparent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](uint8_t v){testWrapper(3, v); return false;}});
                }
                {
                    auto llparent = addParent(p, Param_t{lparent, PType::Folder, "Output 4"});
                    addNode(p, Param_t{llparent, PType::Info, "Output 4 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{llparent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[4].mode, 0, 1});
                    addNode(p, Param_t{llparent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[4].pwmDuty, 1, 99, [](uint8_t v){tlc_1::pwm(4, v); return true;}});
                    addNode(p, Param_t{llparent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](uint8_t v){testWrapper(4, v); return false;}});
                }
                {
                    auto llparent = addParent(p, Param_t{lparent, PType::Folder, "Output 5"});
                    addNode(p, Param_t{llparent, PType::Info, "Output 5 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{llparent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[5].mode, 0, 1});
                    addNode(p, Param_t{llparent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[5].pwmDuty, 1, 99, [](uint8_t v){tlc_1::pwm(5, v); return true;}});
                    addNode(p, Param_t{llparent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](uint8_t v){testWrapper(5, v); return false;}});
                }
                {
                    auto llparent = addParent(p, Param_t{lparent, PType::Folder, "Output 6"});
                    addNode(p, Param_t{llparent, PType::Info, "Output 6 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{llparent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[6].mode, 0, 1});
                    addNode(p, Param_t{llparent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[6].pwmDuty, 1, 99, [](uint8_t v){tlc_1::pwm(6, v); return true;}});
                    addNode(p, Param_t{llparent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](uint8_t v){testWrapper(6, v); return false;}});
                }
                {
                    auto llparent = addParent(p, Param_t{lparent, PType::Folder, "Output 7"});
                    addNode(p, Param_t{llparent, PType::Info, "Output 7 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{llparent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[7].mode, 0, 1});
                    addNode(p, Param_t{llparent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[7].pwmDuty, 1, 99, [](uint8_t v){tlc_1::pwm(7, v); return true;}});
                    addNode(p, Param_t{llparent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](uint8_t v){testWrapper(7, v); return false;}});
                }
            }
            lparent = addParent(p, Param_t{parent, PType::Folder, "V/ESC"});
            {
                addNode(p, Param_t{lparent, PType::Info, "Type : ", &mVescName[0]});
                addNode(p, Param_t{lparent, PType::Info, "Firmware : ", &mVescFirmware[0]});
            }
        }
        parent = addParent(p, Param_t{0, PType::Folder, "Serials"});
        addNode(p, Param_t{parent, PType::Sel, "Serial 1", "VEsc;SBus;IBus;SBus2", &eeprom.serial1_mode, 0, 1});
        addNode(p, Param_t{parent, PType::Sel, "Serial 2", "VEsc;SBus;IBus;SBus2", &eeprom.serial2_mode, 0, 1});
        addNode(p, Param_t{parent, PType::Sel, "Aux 1", "Off;GPS", &eeprom.aux1_mode, 0, 1});
        addNode(p, Param_t{parent, PType::Sel, "Aux 2", "Off;GPS", &eeprom.aux2_mode, 0, 1});

        parent = addParent(p, Param_t{0, PType::Folder, "Telemetry"});
        addNode(p, Param_t{parent, PType::U8,  "RPM1 PolePairs", nullptr, &eeprom.telemetry_polepairs, 2, 12});
        addNode(p, Param_t{parent, PType::Sel, "Current", "ESC In;Motor Peak", &eeprom.telemetry_currentSelect, 0, 1});

        return p;
    }();
};

#endif

#if defined(USE_DEVICES1) || defined(USE_DEVICES2)
template<typename Config, typename SwitchCallback,
         typename Trace = void>
struct CrsfCallback {
    using trace = Trace;
    using Param_t = RC::Protokoll::Crsf::Parameter;
    using PType = RC::Protokoll::Crsf::Parameter::Type;
    using telem_out = Config::telem_out;
    using timer = Config::timer;
    using crsfTelemetry = CrsfTelemetry<telem_out, timer>;
    using tlc_1 = Config::tlc1;

    static inline constexpr const char* const title = "CruiseControl @ ";

    using name_t = std::array<char, 32>;

    // SM: only sending telemetry after getting own command
    // own-command -> channels -> send-telemetry
    // this avoids (mostly) collisions in half-duplex
    //
    // in full-duplex this acts as a request: the crsf-fd-switch
    // must leave the slot free

    enum class State : uint8_t {Undefined, GotStats, GotChannels};

    static inline State mStreamState{State::Undefined};

    static inline constexpr void gotLinkStats() {
        if (mStreamState == State::Undefined) {
            mStreamState = State::GotStats;
        }
    }

    static inline constexpr void gotChannels() {
        if (mStreamState == State::GotStats) {
            mStreamState = State::GotChannels;
            crsfTelemetry::event(crsfTelemetry::Event::SendNext);
        }
    }

    static inline constexpr void ratePeriodic() {
        crsfTelemetry::ratePeriodic();
    }

    static inline constexpr void updateName(name_t& n) {
        strncpy(&n[0], title, n.size());
        std::to_chars(std::begin(n) + strlen(title), std::end(n), eeprom.address);
    }
    static inline void update() {
        updateName(mName);
    }

    static inline constexpr auto& eeprom = Storage::eeprom;
    static inline constexpr auto& eeprom_flash = Storage::eeprom_flash;
    static inline void save() {
        if (Mcu::Stm32::savecfg(eeprom, eeprom_flash)) {
            IO::outl<trace>("EEPROM OK");
        }
        else {
            IO::outl<trace>("EEPROM NOK");
        }
    }
    static inline void setParameter(const uint8_t index, const uint8_t value) {
        IO::outl<trace>("SetP i: ", index, " v: ", value);
        if ((index >= 1) && (index <= params.size())) {
            params[index - 1].value(value);
            mLastChangedParameter = index;
            if (params[index - 1].cb) {
                params[index - 1].cb(value);
            }
            update();
            save();
        }
    }
    static inline RC::Protokoll::Crsf::Parameter parameter(const uint8_t index) {
        if ((index >= 1) && (index <= params.size())) {
            return params[index - 1];
        }
        return {};
    }
    static inline bool isCommand(const uint8_t index) {
        if ((index >= 1) && (index <= params.size())) {
            return params[index - 1].mType == PType::Command;
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
        return params.size();
    }
    static inline uint8_t protocolVersion() {
        return 0;
    }
    static inline void whenParameterChanged(auto f) {
        if (mLastChangedParameter > 0) {
            f(mLastChangedParameter);
            mLastChangedParameter = 0;
        }
    }
    template<auto L>
    static inline void command(const std::array<uint8_t, L>& payload) {
        if (payload[0] == (uint8_t)RC::Protokoll::Crsf::Address::Controller) {
            if (payload[2] == (uint8_t)RC::Protokoll::Crsf::CommandType::Switch) {
                if (payload[3] == (uint8_t)RC::Protokoll::Crsf::SwitchCommand::Set) {
                    const uint8_t address = (uint8_t)payload[4];
                    const uint8_t sw = (uint8_t)payload[5];
                    if (eeprom.address == address) {
                        IO::outl<trace>("Command: ", address, " sw: ", sw);
                        SwitchCallback::set(sw);
                    }
                    else if (eeprom.tlc_1_address == address) {
                        IO::outl<trace>("Command: ", address, " sw: ", sw);
                        for(uint8_t i = 0; i < 8; ++i) {
                            if (sw & (0b01 << i)) {
                                if (eeprom.tlc_1[i].mode == 1) {
                                    tlc_1::pwmon(i);
                                }
                                else {
                                    tlc_1::on(i);
                                }
                            }
                            else {
                                tlc_1::off(i);
                            }
                        }
                    }
                }
            }
        }
    }
    static inline void callbacks() {
        for(const auto p: params) {
            if (p.cb) {
                p.cb(p.value());
            }
        }
    }
    private:
    static inline uint8_t mLastChangedParameter{};
    static inline constexpr uint32_t mSerialNumber{1234};
    static inline constexpr uint32_t mHWVersion{2};
    static inline constexpr uint32_t mSWVersion{10};
    static inline constexpr auto mVersionString = [](){
        std::array<char, 16> s{};
        auto [ptr, e] = std::to_chars(std::begin(s), std::end(s), mHWVersion);
        *ptr++ = ':';
        std::to_chars(ptr, std::end(s), mSWVersion);
        return s;
    }();

    static inline name_t mName = []{
        name_t name{};
        updateName(name);
        return name;
    }();

    static inline uint8_t addParent(auto& c, const Param_t& p) {
        c.push_back(p);
        return c.size();
    }
    static inline void addNode(auto& c, const Param_t& p) {
        c.push_back(p);
    }

    // static inline constexpr auto& eeprom = Storage::eeprom;

    using params_t = etl::FixedVector<Param_t, 128>;
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        auto parent = addParent(p, Param_t{0, PType::Folder, "Global"});
        addNode(p, Param_t{parent, PType::U8, "Switch Address", nullptr, &eeprom.address, 0, 255});

        parent = addParent(p, Param_t{0, PType::Folder, "PWM Outputs"});
        {
            auto lparent = addParent(p, Param_t{parent, PType::Folder, "Output 0"});
            addNode(p, Param_t{lparent, PType::Info, "Output 0 : ", &mName[0]});
            addNode(p, Param_t{lparent, PType::Sel, "Mode", "Pwm;Switch", &eeprom.outputs[0].mode, 0, 1});
            addNode(p, Param_t{lparent, PType::U8,  "PWM Mode Channel", nullptr, &eeprom.outputs[0].channel, 1, 16});
            addNode(p, Param_t{lparent, PType::U8,  "Switch Mode Nr", nullptr, &eeprom.outputs[0].sw, 0, 7});

            lparent = addParent(p, Param_t{parent, PType::Folder, "Output 1"});
            addNode(p, Param_t{lparent, PType::Info, "Output 1 : ", &mName[0]});
            addNode(p, Param_t{lparent, PType::Sel, "Mode", "Pwm;Switch", &eeprom.outputs[1].mode, 0, 1});
            addNode(p, Param_t{lparent, PType::U8,  "PWM Mode Channel", nullptr, &eeprom.outputs[1].channel, 1, 16});
            addNode(p, Param_t{lparent, PType::U8,  "Switch Mode Nr", nullptr, &eeprom.outputs[1].sw, 0, 7});

            lparent = addParent(p, Param_t{parent, PType::Folder, "Output 2"});
            addNode(p, Param_t{lparent, PType::Info, "Output 2 : ", &mName[0]});
            addNode(p, Param_t{lparent, PType::Sel, "Mode", "Pwm;Switch", &eeprom.outputs[2].mode, 0, 1});
            addNode(p, Param_t{lparent, PType::U8,  "PWM Mode Channel", nullptr, &eeprom.outputs[2].channel, 1, 16});
            addNode(p, Param_t{lparent, PType::U8,  "Switch Mode Nr", nullptr, &eeprom.outputs[2].sw, 0, 7});

            lparent = addParent(p, Param_t{parent, PType::Folder, "Output 3"});
            addNode(p, Param_t{lparent, PType::Info, "Output 3 : ", &mName[0]});
            addNode(p, Param_t{lparent, PType::Sel, "Mode", "Pwm;Switch", &eeprom.outputs[3].mode, 0, 1});
            addNode(p, Param_t{lparent, PType::U8,  "PWM Mode Channel", nullptr, &eeprom.outputs[3].channel, 1, 16});
            addNode(p, Param_t{lparent, PType::U8,  "Switch Mode Nr", nullptr, &eeprom.outputs[3].sw, 0, 7});
        }

        parent = addParent(p, Param_t{0, PType::Folder, "Externals"});
        {
            auto lparent = addParent(p, Param_t{parent, PType::Folder, "Switch Ctrl 0"});
            {
                addNode(p, Param_t{lparent, PType::U8,  "Address", nullptr, &eeprom.tlc_1_address, 0, 255});
                {
                    auto llparent = addParent(p, Param_t{lparent, PType::Folder, "Output 0"});
                    addNode(p, Param_t{llparent, PType::Info, "Output 0 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{llparent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[0].mode, 0, 1});
                    addNode(p, Param_t{llparent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[0].pwmDuty, 1, 99, [](uint8_t v){tlc_1::pwm(0, v);}});
                    addNode(p, Param_t{llparent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](uint8_t v){
                                           if (v == 0) {
                                               tlc_1::off(0);
                                           }
                                           else {
                                               if (eeprom.tlc_1[0].mode == 0) {
                                                   tlc_1::on(0);
                                               }
                                               else {
                                                   tlc_1::pwmon(0);
                                               }
                                           }
                                       }});
                }
                {
                    auto llparent = addParent(p, Param_t{lparent, PType::Folder, "Output 1"});
                    addNode(p, Param_t{llparent, PType::Info, "Output 1 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{llparent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[1].mode, 0, 1});
                    addNode(p, Param_t{llparent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[1].pwmDuty, 1, 99});
                    addNode(p, Param_t{llparent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](uint8_t v){
                                           if (v == 0) {
                                               tlc_1::off(1);
                                           }
                                       }});
                }
#if 0
                {
                    auto parent = addParent(p, Param_t{parent, PType::Folder, "Output 2"});
                    addNode(p, Param_t{parent, PType::Info, "Output 2 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{parent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[2].mode, 0, 1});
                    addNode(p, Param_t{parent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[2].pwmDuty, 1, 99});
                }
                {
                    auto parent = addParent(p, Param_t{parent, PType::Folder, "Output 0"});
                    addNode(p, Param_t{parent, PType::Info, "Output 3 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{parent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[3].mode, 0, 1});
                    addNode(p, Param_t{parent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[3].pwmDuty, 1, 99});
                }
                {
                    auto parent = addParent(p, Param_t{parent, PType::Folder, "Output 4"});
                    addNode(p, Param_t{parent, PType::Info, "Output 4 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{parent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[4].mode, 0, 1});
                    addNode(p, Param_t{parent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[4].pwmDuty, 1, 99});
                }
                {
                    auto parent = addParent(p, Param_t{parent, PType::Folder, "Output 5"});
                    addNode(p, Param_t{parent, PType::Info, "Output 5 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{parent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[5].mode, 0, 1});
                    addNode(p, Param_t{parent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[5].pwmDuty, 1, 99});
                }
                {
                    auto parent = addParent(p, Param_t{parent, PType::Folder, "Output 6"});
                    addNode(p, Param_t{parent, PType::Info, "Output 6 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{parent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[6].mode, 0, 1});
                    addNode(p, Param_t{parent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[6].pwmDuty, 1, 99});
                }
                {
                    auto parent = addParent(p, Param_t{parent, PType::Folder, "Output 7"});
                    addNode(p, Param_t{parent, PType::Info, "Output 7 : ", "Switch Ctrl 0"});
                    addNode(p, Param_t{parent, PType::Sel, "PWM", "Off;On", &eeprom.tlc_1[7].mode, 0, 1});
                    addNode(p, Param_t{parent, PType::U8,  "PWM duty", nullptr, &eeprom.tlc_1[7].pwmDuty, 1, 99});
                }
#endif
            }

            // parent = addParent(p, Param_t{parent, PType::Folder, "LED Ctrl 0"});

        }

        parent = addParent(p, Param_t{0, PType::Folder, "Serials"});
        addNode(p, Param_t{parent, PType::Sel, "Serial 1", "VEsc;SBus;IBus;SBus2", &eeprom.serial1_mode, 0, 1});
        addNode(p, Param_t{parent, PType::Sel, "Serial 2", "VEsc;SBus;IBus;SBus2", &eeprom.serial2_mode, 0, 1});
        addNode(p, Param_t{parent, PType::Sel, "Aux 1", "Off;GPS", &eeprom.aux1_mode, 0, 1});
        addNode(p, Param_t{parent, PType::Sel, "Aux 2", "Off;GPS", &eeprom.aux2_mode, 0, 1});

        parent = addParent(p, Param_t{0, PType::Folder, "Telemetry"});
        addNode(p, Param_t{parent, PType::U8,  "RPM1 PolePairs", nullptr, &eeprom.telemetry_polepairs, 2, 12});

        return p;
    }();
};
#endif
