#pragma once

#include "mcu/eeprom.h"

#include "meta.h"

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

    using responder = Config::adapter::responder;

    using p1 = Meta::nth_element<0, polars>;
    using p2 = Meta::nth_element<1, polars>;

    using Param_t = RC::Protokoll::Crsf::Parameter<uint16_t>;
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
    }
    static inline constexpr void gotLinkStats() {
    }
    static inline constexpr void gotChannels() {
    }
    static inline constexpr void forwardPacket(const std::byte type, const std::array<uint8_t, 64>& data, const uint16_t length) {
        relays::forwardPacket(type, data, length);
        auxes::forwardPacket(type, data, length);
    }
    static inline constexpr void gotCommand(const std::array<uint8_t, 64>& data, const uint16_t length) {
        relays::command(data, length);
        auxes::command(data, length);
    }
    static inline void gotPing() {
        relays::ping();
        auxes::ping();
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
    }

    static inline void save() {
        if (auto [ok, err] = Mcu::Stm32::savecfg(eeprom, eeprom_flash); ok) {
            IO::outl<debug>("# EEPROM OK");
        }
        else {
            IO::outl<debug>("# EEPROM NOK: ", err);
        }
    }
    static inline void setParameter(const uint8_t index, const uint16_t value, const auto& data, const uint8_t dlen) {
        IO::outl<debug>("# SetP i: ", index, " v: ", value);
        if ((index >= 1) && (index <= params.size())) {
            mLastChangedParameter = index;
            bool mustSave = true;
            if (params[index - 1].mType == Param_t::Str) {
                IO::outl<debug>("# String");
                if (params[index - 1].mStringValue) {
                    for(uint8_t i = 0; (i < 16) && (i < dlen); ++i) {
                        params[index - 1].mStringValue[i] = data[i];
                        if (data[i] == '\0') {
                            break;
                        }
                    }
                    params[index - 1].mStringValue[dlen] = '\0';
                }
            }
            else {
                IO::outl<debug>("# Num: ", value);
                params[index - 1].value(value);
                if (params[index - 1].cb) {
                    mustSave = params[index - 1].cb(value);
                }
            }
            update();
            if (mustSave) {
                save();
            }
        }
    }
    static inline Param_t parameter(const uint8_t index) {
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
        const uint8_t destAddress = payload[0];
        const uint8_t srcAddress = payload[1];
        const uint8_t realm = payload[2];
        const uint8_t cmd = payload[3];
        if ((srcAddress == (uint8_t)RC::Protokoll::Crsf::Address::Handset) && (destAddress >= 0xc0) && (destAddress <= 0xcf)) {
            if (realm == (uint8_t)RC::Protokoll::Crsf::CommandType::Schottel) {
                if (eeprom.address == destAddress) {
                    if (cmd == (uint8_t)RC::Protokoll::Crsf::SchottelCommand::Reset) {
                        IO::outl<debug>("# Cmd Reset: ", destAddress);
                        servos::zero();
                    }
                }
            }
        }
    }
    static inline void callbacks(const bool eepromMode = false) {
        const bool prevMode = mEepromMode;
        mEepromMode = eepromMode;
        for(const auto& p: params) {
            if (p.mType != PType::Command) {
                if (p.cb) {
                    p.cb(p.value());
                }
            }
        }
        mEepromMode = prevMode;
    }
private:
    static inline name_t mName = []{
        name_t name{};
        updateName(name);
        return name;
    }();

    static inline bool mEepromMode = false;
    static inline uint8_t mLastChangedParameter{};
    static inline constexpr uint32_t mSerialNumber{1234};
    static inline constexpr uint32_t mHWVersion{2};
    static inline constexpr uint32_t mSWVersion{2};
    static inline constexpr auto mVersionString = [](){
        std::array<char, 16> s{};
        auto [ptr, e] = std::to_chars(std::begin(s), std::end(s), mHWVersion);
        *ptr++ = ':';
        auto r = std::to_chars(ptr, std::end(s), mSWVersion);
        *r.ptr = '\0';
        return s;
    }();

    static inline uint8_t addParent(auto& c, const Param_t& p) {
        c.push_back(p);
        return c.size();
    }
    static inline void addNode(auto& c, const Param_t& p) {
        c.push_back(p);
    }

    static inline bool setZeroPosition(const uint16_t) {
        return true;
    }

    static inline std::array<const char*, 6> mCalibratingTexts {
        "Calibrate",
        "Start ...",
        "Measure Rm, Lm ...",
        "Measure Km (CW) ...",
        "Measure Km (CCW) ...",
        "Finished calibrating"
    };
    static inline bool calibCallback(const uint8_t v) {
        IO::outl<debug>("# calibCallback v: ", v, " s: ");
        // if (s < std::size(mCalibratingTexts)) {
        //     params[mCalibCommand].mOptions = mCalibratingTexts[s];
        // }
        // return res;
        return false;
    }
    static inline uint8_t mCalibCommand{};
    static inline uint8_t mESCape321Folder{};
    static inline uint8_t mESCape321End{};
    static inline uint8_t mESCape322Folder{};
    static inline uint8_t mESCape322End{};

    static inline std::array<const char*, 6> mServoAddressTexts {
        "Set Address",
        "Start ...",
        "... End"
    };
    static inline uint8_t mServoAddressCommand;

    static inline std::array<char, 16> mDeadLowString{'a'};

    static inline void hide(const uint8_t n, const bool b) {
        params[n].hide(b);
    }
    static inline void hide(const uint8_t start, const uint8_t end, const bool b) {
        for(uint8_t i = start; i <= end; ++i) {
            hide(i, b);
        }
    }

    using params_t = etl::FixedVector<Param_t, 128>;
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::Sel,  "Mode", "Dual-Schottel-Controller;Cruise-Controller", &eeprom.mode, 0, 1, [](const uint16_t){return true;}});
        uint8_t parent = addParent(p, Param_t{0, PType::Folder, "Channels"});
        addNode(p, Param_t{parent, PType::Sel,  "Stream", "Main/CRSF;Alternative;Aux", &eeprom.input_stream, 0, 2, [](const uint16_t s){mapper::stream(s); return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Schottel 1: f/b", nullptr, &eeprom.channels[0].first, 0, 15, [](const uint16_t){return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Schottel 1: l/r", nullptr, &eeprom.channels[0].second, 0, 15, [](const uint16_t){return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Schottel 2: f/b", nullptr, &eeprom.channels[1].first, 0, 15, [](const uint16_t){return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Schottel 2: l/r", nullptr, &eeprom.channels[1].second, 0, 15, [](const uint16_t){return true;}});
        parent = addParent(p, Param_t{0, PType::Folder, "Outputs"});
        addNode(p, Param_t{parent, PType::Sel, "Srv1 Out", "PWM/Analog;PWM/PWM;Serial/WaveShare;None", &eeprom.out_mode_srv[0], 0, 3, [](const uint16_t s){servos::template servo<0>(s); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Srv1 Fb", "Analog;PWM;WaveShare;None", &eeprom.out_mode_srv[0], 0, 3});
        addNode(p, Param_t{parent, PType::Sel, "Esc1 Out", "PWM/-;Escape32/Serial;ESCsper32/Ascii;VEsc/Serial;None", &eeprom.out_mode_esc[0], 0, 4, [](const uint16_t s){escs::template esc<0>(s); return true;}});
        // addNode(p, Param_t{parent, PType::Sel, "Esc1 Out", "PWM/-;Escape32/Serial;ESCsper32/Ascii;VEsc/Serial;None", &eeprom.out_mode_esc[0], 0, 4, [](const uint16_t s){escs::template esc<0>(s); if (s == 2) {hide(mESCape321Folder, mESCape321End, false);} else {hide(mESCape321Folder, mESCape321End, true);} return true;}});
#ifdef SERIAL_DEBUG
        addNode(p, Param_t{parent, PType::Sel, "Esc1 Tlm", "Debug;S.Port;VEsc/Bidirectional", &eeprom.tlm_mode_esc[0], 0, 4, [](const uint16_t){return true;}});
#else
        addNode(p, Param_t{parent, PType::Sel, "Esc1 Tlm", "S.Port;VEsc/Bidirectional", &eeprom.tlm_mode_esc[0], 0, 4, [](const uint16_t){return true;}});
#endif
        addNode(p, Param_t{parent, PType::Sel, "Srv2 Out", "PWM/Analog;PWM/PWM;Serial/WaveShare;None", &eeprom.out_mode_srv[1], 0, 3, [](const uint16_t s){servos::template servo<1>(s); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Srv2 Fb", "Analog;PWM;WaveShare;None", &eeprom.out_mode_srv[1], 0, 3});
        addNode(p, Param_t{parent, PType::Sel, "Esc2 Out", "PWM/-;Escape32/Serial;ESCsper32/Ascii;VEsc/Serial;None", &eeprom.out_mode_esc[1], 0, 4, [](const uint16_t s){escs::template esc<1>(s); return true;}});
        // addNode(p, Param_t{parent, PType::Sel, "Esc2 Out", "PWM/-;Escape32/Serial;ESCsper32/Ascii;VEsc/Serial;None", &eeprom.out_mode_esc[1], 0, 4, [](const uint16_t s){escs::template esc<1>(s); if (s == 2) {hide(mESCape322Folder, false);} return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Esc2 Tlm", "S.Port;VEsc/Bidirectional", &eeprom.tlm_mode_esc[1], 0, 2, [](const uint16_t){return true;}});
        parent = addParent(p, Param_t{0, PType::Folder, "Settings"});
        addNode(p, Param_t{parent, PType::U16,  "Sch1 ESC Deadband", nullptr, &eeprom.deadbands[0], 1, 20, [](const uint16_t){return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Sch1 ESC PWM Mid", nullptr, &eeprom.esc_mid[0], 0, 200, [](const uint16_t){return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Sch1 Servo zPosition", nullptr,  &eeprom.offset1, 0, 359, [](const uint16_t v){servos::template offset<0>((uint32_t{v} * 4096) / 360); return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Sch1 Servo Speed", nullptr, &eeprom.speed1, 1, 340, [](const uint16_t v){servos::template speed<0>(10 * v); return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Sch2 ESC Deadband", nullptr, &eeprom.deadbands[1], 1, 20, [](const uint16_t){return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Sch2 ESC PWM Mid", nullptr, &eeprom.esc_mid[1], 0, 200, [](const uint16_t){return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Sch2 Servo zPosition", nullptr,  &eeprom.offset2, 0, 360, [](const uint16_t v){servos::template offset<1>((uint32_t{v} * 4096) / 360); return true;}});
        addNode(p, Param_t{parent, PType::U16,  "Sch2 Servo Speed", nullptr, &eeprom.speed2, 1, 100, [](const uint16_t v){servos::template speed<1>(10 * v); return true;}});
        addNode(p, Param_t{parent, PType::Command, "Act. Pos. as zPos 1", mCalibratingTexts[0], nullptr, 0, 0, [](const uint16_t v){return setZeroPosition(v);}});
        addNode(p, Param_t{parent, PType::Command, "Act. Pos. as zPos 2", mCalibratingTexts[0], nullptr, 0, 0, [](const uint16_t v){return setZeroPosition(v);}});

#ifdef ESCAPE32_ASCII
        parent = addParent(p, Param_t{0, PType(PType::Folder | PType::Hidden), "ESCape32 1"});
        mESCape321Folder = p.size() - 1;
        addNode(p, Param_t{parent, PType::Command, "Beep", nullptr, nullptr, 0, 0, [](const uint16_t){esc32ascii_1::beep(); return false;}});
        addNode(p, Param_t{parent, PType::Command, "Save", nullptr, nullptr, 0, 0, [](const uint16_t){esc32ascii_1::save(); return false;}});
        [&]<size_t... II>(std::integer_sequence<size_t, II...>){
            (addNode(p, Param_t{parent, PType::U16, esc32ascii_1::params()[II].name, nullptr, &esc32ascii_1::params()[II].value, esc32ascii_1::params()[II].min, esc32ascii_1::params()[II].max, [](const uint16_t v){esc32ascii_1::setParam(II, v); return false;}}), ...);
        }(std::make_index_sequence<esc32ascii_1::params().size()>{});
        mESCape321End = p.size() - 1;

        parent = addParent(p, Param_t{0, PType(PType::Folder | PType::Hidden), "ESCape32 2"});
        mESCape322Folder = p.size() - 1;
        addNode(p, Param_t{parent, PType::Command, "Beep", nullptr, nullptr, 0, 0, [](const uint16_t){esc32ascii_1::beep(); return false;}});
        addNode(p, Param_t{parent, PType::Command, "Save", nullptr, nullptr, 0, 0, [](const uint16_t){esc32ascii_1::save(); return false;}});
        mESCape322End = p.size() - 1;
#endif

#ifdef SERVO_CALIBRATION
        parent = addParent(p, Param_t{0, PType::Folder, "Calibration"});
        addNode(p, Param_t{parent, PType::Command, "Calibrate", mCalibratingTexts[0], nullptr, 0, 0, [](const uint16_t v){return calibCallback(v);}});
        mCalibCommand = p.size() - 1;
        addNode(p, Param_t{parent, PType::Info, "Srv1 DeadL", &mDeadLowString[0]});
#endif
        parent = addParent(p, Param_t{0, PType::Folder, "CRSF"});
        addNode(p, Param_t{parent, PType::U16,  "CRSF Address", nullptr, &eeprom.address, 192, 207, [](const uint16_t a){updateName(mName); responder::address(std::byte(a)); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Mode (not persistant)", "Full;Fwd Only", nullptr, 0, 1, [](const uint16_t){return false;}});
        parent = addParent(p, Param_t{0, PType::Folder, "Special"});
        addNode(p, Param_t{parent, PType::Sel, "Crsf-HD/SBus", "SBus/Out;Crsf;SBus2/Master;CPPM/N;CPPM/P;CombinedPWMChannels/P;IBus/In;SBus/In;SumDV3/In;None", &eeprom.crsf_hd_mode, 0, 9, [](const uint16_t r){relays::set(r); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Crsf-FD/Aux", "Crsf;GPS;None", &eeprom.crsf_fd_aux_mode, 0, 1, [](const uint16_t a){
                               auxes::set(a);
                               return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Inject (SBus)", "Yes;No", &eeprom.inject, 0, 1, [](const uint16_t){return true;}});
#ifdef SERVO_ADDRESS_SET
        addNode(p, Param_t{parent, PType::Command, "Set Servo ID", mServoAddressTexts[0], nullptr, 0, 0, [](const uint16_t v){return calibCallback(v);}});
        mServoAddressCommand = p.size() - 1;
        addNode(p, Param_t{parent, PType::U16,  "Servo ID to set", nullptr, nullptr, 1, 16, [](const uint16_t){return false;}});
#endif

        return p;
    }();
};

