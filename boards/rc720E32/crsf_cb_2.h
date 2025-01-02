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
    using servos = Config::servos;
    using escs = Config::escs;
    using relays = Config::relays;
    using auxes = Config::auxes;
    using mapper = Config::mapper;

    using polars = Config::polars;

    using src = Config::src;
    using input  = src::input;

    using p1 = Meta::nth_element<0, polars>;
    using p2 = Meta::nth_element<1, polars>;

    using store_t = EEProm::eeprom_value_t;
    using Param_t = RC::Protokoll::Crsf::V4::Parameter<store_t>;
    using PType = Param_t::Type;

    // std::integral_constant<uint8_t, sizeof(RC::Protokoll::Crsf::V4::Parameter<uint8_t>)>::_; // 44
    // std::integral_constant<uint8_t, sizeof(RC::Protokoll::Crsf::V4::Parameter<store_t>)>::_; // 48

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
        if (index == 0) return;
        if (index < params.size()) {
            mLastChangedParameter = index;
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
    static inline void whenParameterChanged(auto f) {
        if (mLastChangedParameter > 0) {
            f(mLastChangedParameter);
            mLastChangedParameter = 0;
        }
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
private:
    static inline name_t mName = []{
        name_t name{};
        updateName(name);
        return name;
    }();

    static inline bool mEepromMode = false;
    static inline uint8_t mLastChangedParameter{};
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
    using params_t = etl::FixedVector<Param_t, 200>;
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Folder, ""});
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::Sel,  "Mode", "Dual-Schottel-Controller;Cruise-Controller", &eeprom.mode, 0, 1, [](const store_t){return true;}});
        uint8_t parent = addParent(p, Param_t{0, PType::Folder, "Channels"});
        addNode(p, Param_t{.parent = parent, .type = PType::Sel, .name = "Stream", .options = "Main/CRSF;Alternative;Aux", .value_ptr = &eeprom.input_stream, .max = 2, .cb = [](const store_t s){mapper::stream(s); return true;}});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Schottel 1: f/b", .value_ptr = &eeprom.channels[0].first, .max = 15, .cb = [](const store_t){return true;}});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "Schottel 1: l/r", .value_ptr = &eeprom.channels[0].second, .max = 15, .cb = [](const store_t){return true;}});
        addNode(p, Param_t{parent, PType::U8, "Schottel 2: f/b", nullptr, &eeprom.channels[1].first, 0, 15, [](const store_t){return true;}});
        addNode(p, Param_t{parent, PType::U8, "Schottel 2: l/r", nullptr, &eeprom.channels[1].second, 0, 15, [](const store_t){return true;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Outputs"});
        addNode(p, Param_t{parent, PType::Sel, "Srv1 Out", "PWM/Analog;PWM/PWM;Serial/WaveShare;None", &eeprom.out_mode_srv[0], 0, 3, [](const store_t s){servos::template servo<0>(s); return true;}});
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
        addNode(p, Param_t{parent, PType::Sel, "Srv2 Out", "PWM/Analog;PWM/PWM;Serial/WaveShare;None", &eeprom.out_mode_srv[1], 0, 3, [](const store_t s){servos::template servo<1>(s); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Srv2 Fb", "Analog;PWM;WaveShare;None", &eeprom.out_mode_srv[1], 0, 3});
#ifdef ESCAPE32_ASCII
        addNode(p, Param_t{parent, PType::Sel, "Esc2 Out", "PWM/-;ESCape32/Serial;ESCape32/Ascii;VEsc/Serial;None", &eeprom.out_mode_esc[1], 0, 4, [](const store_t s){escs::template esc<1>(s); if (s == 2) {hide(mESCape322Folder, false);} return true;}});
#else
        addNode(p, Param_t{parent, PType::Sel, "Esc2 Out", "PWM/-;ESCape32/Serial;ESCape32/Ascii;VEsc/Serial;None", &eeprom.out_mode_esc[1], 0, 4, [](const store_t s){escs::template esc<1>(s); return true;}});
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
        addNode(p, Param_t{parent, PType::Command, "Act. Pos. as zPos 1", mCalibratingTexts[0], nullptr, 0, 0, [](const store_t v){return setZeroPosition(v);}});
        addNode(p, Param_t{parent, PType::Command, "Act. Pos. as zPos 2", mCalibratingTexts[0], nullptr, 0, 0, [](const store_t v){return setZeroPosition(v);}});

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
        mCalibCommand = parent;
        addNode(p, Param_t{parent, PType::Command, "Calibrate", mCalibratingTexts[0], nullptr, 0, 0, [](const store_t v){return calibCallback(v);}});
        addNode(p, Param_t{parent, PType::Info, "Srv1 DeadL", &mDeadLowString[0]});
#endif

        parent = addParent(p, Param_t{0, PType::Folder, "CRSF"});
        addNode(p, Param_t{parent, PType::U8,  "CRSF Address", nullptr, &eeprom.address, 192, 207, [](const store_t a){updateName(mName); src::address(std::byte(a)); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Mode (not persistant)", "Full;Fwd Only", nullptr, 0, 1, [](const store_t){return false;}});
        parent = addParent(p, Param_t{0, PType::Folder, "Special"});
        addNode(p, Param_t{parent, PType::Sel, "Crsf-HD/SBus", "SBus/Out;Crsf;SBus2/Master;CPPM/N;CPPM/P;CombinedPWMChannels/P;IBus/In;SBus/In;SumDV3/In;None", &eeprom.crsf_hd_mode, 0, 9, [](const store_t r){relays::set(r); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Crsf-FD/Aux", "Crsf;GPS;None", &eeprom.crsf_fd_aux_mode, 0, 1, [](const store_t a){
                               auxes::set(a);
                               return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Inject (SBus)", "Yes;No", &eeprom.inject, 0, 1, [](const store_t){return true;}});
#ifdef SERVO_ADDRESS_SET
        addNode(p, Param_t{parent, PType::Command, "Set Servo ID", mServoAddressTexts[0], nullptr, 0, 0, [](const store_t v){return calibCallback(v);}});
        mServoAddressCommand = p.size() - 1;
        addNode(p, Param_t{parent, PType::U8,  "Servo ID to set", nullptr, nullptr, 1, 16, [](const store_t){return false;}});
#endif
        return p;
    }();
};

