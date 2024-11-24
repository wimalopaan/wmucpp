#pragma once

#include <mcu/eeprom.h>
#include "telemetry.h"

template<typename Config, typename Trace = void>
struct CrsfCallback {
    using trace = Trace;
    using Param_t = RC::Protokoll::Crsf::Parameter<uint8_t>;
    using PType = RC::Protokoll::Crsf::Parameter<uint8_t>::Type;
    using telem_out = Config::telem_out;
    using timer = Config::timer;
    using storage = Config::storage;
    using notifier = Config::notifier;
    using speed = Config::speed;
    using crsfTelemetry = CrsfTelemetry<telem_out, timer, storage>;

    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;

#ifdef CRSF_MODULE_NAME
    static inline constexpr const char* const title = CRSF_MODULE_NAME;
#else
    static inline constexpr const char* const title = "WM-BDC-32-S(8A)";
#endif

    using name_t = std::array<char, 32>;


    static inline constexpr void disableTelemetry() {
    }
    static inline constexpr void gotLinkStats() {
    }
    static inline constexpr void gotChannels() {
    }

    static inline constexpr void ratePeriodic() {
        crsfTelemetry::ratePeriodic();
    }

    static inline constexpr void updateName(name_t& n) {
        strncpy(&n[0], title, n.size());
    }
    static inline void update() {
        updateName(mName);
        const uint16_t maxRpm = (60 * eeprom.cutoff_freq) / eeprom.telemetry_polepairs;
        auto r = std::to_chars(std::begin(mMaxRpmString), std::end(mMaxRpmString), maxRpm * 100);
        *r.ptr = '\0';
        const uint16_t fpwm = eeprom.cutoff_freq * eeprom.n_fsample * eeprom.subsampling;
        r = std::to_chars(std::begin(mPwmFreqString), std::end(mPwmFreqString), fpwm * 100);
        *r.ptr = '\0';

        r = std::to_chars(std::begin(mResistanceString), std::end(mResistanceString), (uint16_t)(eeprom.resistance.dir1 * 1000));
        *r.ptr++ = ' ';
        r = std::to_chars(r.ptr, std::end(mResistanceString), (uint16_t)(eeprom.resistance.dir2 * 1000));
        *r.ptr = '\0';

        r = std::to_chars(std::begin(mInductanceString), std::end(mInductanceString), (uint16_t)(eeprom.inductance.dir1  * 1000));
        *r.ptr++ = ' ';
        r = std::to_chars(r.ptr, std::end(mInductanceString), (uint16_t)(eeprom.inductance.dir2  * 1000));
        *r.ptr = '\0';

        r = std::to_chars(std::begin(mKmString), std::end(mKmString), (uint16_t)(eeprom.eKm.dir1 / eeprom.telemetry_polepairs));
        *r.ptr++ = ' ';
        r = std::to_chars(r.ptr, std::end(mKmString), (uint16_t)(eeprom.eKm.dir2 / eeprom.telemetry_polepairs));
        *r.ptr = '\0';

        r = std::to_chars(std::begin(mCurrOffString), std::end(mCurrOffString), eeprom.current_offset);
        *r.ptr = '\0';
    }

    static inline void save() {
        if (auto [ok, err] = Mcu::Stm32::savecfg(eeprom, eeprom_flash); ok) {
            IO::outl<trace>("# EEPROM OK");
        }
        else {
            IO::outl<trace>("# EEPROM NOK: ", err);
        }
    }
    static inline void setParameter(const uint8_t index, const uint8_t value, const auto, const auto) {
        IO::outl<trace>("# SetP i: ", index, " v: ", value);
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
    static inline RC::Protokoll::Crsf::Parameter<uint8_t> parameter(const uint8_t index) {
        // IO::outl<trace>("# GetP i: ", index);
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
                    // const uint8_t address = (uint8_t)payload[4];
                    // const uint8_t sw = (uint8_t)payload[5];
                    // if (eeprom.address == address) {
                    //     IO::outl<trace>("# Command: ", address, " sw: ", sw);
                    //     // SwitchCallback::set(sw);
                    // }
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
    private:
    static inline uint8_t mLastChangedParameter{};
    static inline constexpr uint32_t mSerialNumber{1234};
    static inline constexpr uint32_t mHWVersion{2};
    static inline constexpr uint32_t mSWVersion{20};
    static inline std::array<char, 16> mPwmFreqString{};
    static inline std::array<char, 16> mMaxRpmString{};
    static inline std::array<char, 16> mResistanceString{};
    static inline std::array<char, 16> mInductanceString{};
    static inline std::array<char, 16> mKmString{};
    static inline std::array<char, 16> mCurrOffString{};
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

    static inline bool sendCalibrateEvent(const uint8_t v) {
        if (v == 1) {
            return true; // ask confirm
        }
        else if (v == 4) {
            return notifier::startCalibrate();
        }
        else if (v == 5) {
            return notifier::abortCalibrate();
        }
        else if (v == 6) {
            return notifier::isCalibrating();
        }
        return false;
    }

    static inline uint8_t addParent(auto& c, const Param_t& p) {
        c.push_back(p);
        return c.size();
    }
    static inline void addNode(auto& c, const Param_t& p) {
        c.push_back(p);
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
        const bool res = sendCalibrateEvent(v);
        const uint8_t s = notifier::getCalibrateStateNumber();
        IO::outl<trace>("# calibCallback v: ", v, " s: ", s);
        if (s < std::size(mCalibratingTexts)) {
            params[mCalibCommand].mOptions = mCalibratingTexts[s];
        }
        return res;
    }
    static inline uint8_t mCalibCommand{};

    using params_t = etl::FixedVector<Param_t, 64>;
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::U8,  "Rotor Segments", nullptr, &eeprom.telemetry_polepairs, 2, 12, [](const uint8_t){update(); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Cutoff Freq. [100Hz]", nullptr, &eeprom.cutoff_freq, 5, 15, [](const uint8_t){notifier::updatePwm(); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Sample Freq. / Cutoff Freq.", nullptr, &eeprom.n_fsample, 2, 4, [](const uint8_t){notifier::updatePwm(); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Pwm Freq. / Sample Freq.", nullptr, &eeprom.subsampling, 4, 16, [](const uint8_t){notifier::updatePwm(); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Inertia", nullptr, &eeprom.inertia, 0, 9, [](const uint8_t v){speed::updateDutyFilter(v); return true;}});
        addNode(p, Param_t{0, PType::Sel, "PWM Freq.", &mPwmFreqString[0], nullptr, 0, 0});
        addNode(p, Param_t{0, PType::Sel, "Max. RPM", &mMaxRpmString[0], nullptr, 0, 0});
        addNode(p, Param_t{0, PType::Command, "Calibrate", mCalibratingTexts[0], nullptr, 0, 0, [](const uint8_t v){return calibCallback(v);}});
        mCalibCommand = p.size() - 1;
        addNode(p, Param_t{0, PType::Sel, "Resistance [mOhm]", &mResistanceString[0], nullptr, 0, 0});
        addNode(p, Param_t{0, PType::Sel, "Inductance [uH]", &mInductanceString[0], nullptr, 0, 0});
        addNode(p, Param_t{0, PType::Sel, "Km [Rpm/V]", &mKmString[0], nullptr, 0, 0});
        addNode(p, Param_t{0, PType::Sel, "Current Offset", &mCurrOffString[0], nullptr, 0, 0});
        uint8_t parent = addParent(p, Param_t{0, PType::Folder, "Advanced"});
        addNode(p, Param_t{parent, PType::U8,  "Channel", nullptr, &eeprom.crsf_channel, 1, 16, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::Sel, "PreRun Check", "Off;On", &eeprom.prerun_check, 0, 1, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PreRun Hysteresis", nullptr, &eeprom.prerun_hyst, 0, 200, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Tone volume", nullptr, &eeprom.volume, 0, 200, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Calibrate UBatt [0.1%]", nullptr, &eeprom.calib_ubatt, 0, 200, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Temperature Filter", nullptr, &eeprom.temp_filter, 0, 9, [](const uint8_t v){speed::updateTempFilter(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Current", "Batt.-Mean;Motor-Peak", &eeprom.current_select, 0, 1, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Calibrate PWM", "100Hz;200Hz;400Hz", &eeprom.pwm_calib, 0, 2, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Window", "None;Blackman;Hann", &eeprom.timeDomainWindow, 0, 2, [](const uint8_t){notifier::updateWindow(); return true;}});
        addNode(p, Param_t{parent, PType::Command, "Reset to defaults", "Resetting...", nullptr, 0, 0, [](const uint8_t v){if (v == 1) return true; if (v == 4) notifier::resetParameter(); return false;}});
        parent = addParent(p, Param_t{0, PType::Folder, "PID-Controller"});
        addNode(p, Param_t{parent, PType::Sel, "PID Enable", "Off;On", &eeprom.use_pid, 0, 1});
        addNode(p, Param_t{parent, PType::Sel, "PID Mode", "relative;absolute", &eeprom.pid_mode, 0, 1});
        addNode(p, Param_t{parent, PType::U8,  "PID-P", nullptr, &eeprom.pid_p, 0, 100, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PID-I", nullptr, &eeprom.pid_i, 0, 100, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PID-D", nullptr, &eeprom.pid_d, 0, 100, [](const uint8_t){return true;}});
        return p;
    }();
};
