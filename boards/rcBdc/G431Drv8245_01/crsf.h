#pragma once

#include <mcu/eeprom.h>
#include "telemetry.h"

template<typename Config, typename Trace = void>
struct CrsfCallback {
    using trace = Trace;
    using Param_t = RC::Protokoll::Crsf::Parameter;
    using PType = RC::Protokoll::Crsf::Parameter::Type;
    using telem_out = Config::telem_out;
    using timer = Config::timer;
    using storage = Config::storage;
    using notifier = Config::notifier;
    using speed = Config::speed;
    using crsfTelemetry = CrsfTelemetry<telem_out, timer, storage>;

    static inline constexpr auto& eeprom = storage::eeprom;
    static inline constexpr auto& eeprom_flash = storage::eeprom_flash;

    static inline constexpr const char* const title = "WM-BDC32S";

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
    }
    static inline void update() {
        updateName(mName);
        const uint16_t maxRpm = (60 * eeprom.cutoff_freq) / eeprom.telemetry_polepairs;
        auto r = std::to_chars(std::begin(mMaxRpmString), std::end(mMaxRpmString), maxRpm * 100);
        *r.ptr = '\0';
        const uint16_t maxPwm = eeprom.cutoff_freq * eeprom.n_fsample * eeprom.subsampling;
        r = std::to_chars(std::begin(mPwmFreqString), std::end(mPwmFreqString), maxPwm * 100);
        *r.ptr = '\0';

        r = std::to_chars(std::begin(mResistanceString), std::end(mResistanceString), (uint16_t)(eeprom.resistance * 1000));
        *r.ptr = '\0';
        r = std::to_chars(std::begin(mInductanceString), std::end(mInductanceString), (uint16_t)(eeprom.inductance * 1000));
        *r.ptr = '\0';
        r = std::to_chars(std::begin(mKmString), std::end(mKmString), (uint16_t)(eeprom.eKm / eeprom.telemetry_polepairs));
        *r.ptr = '\0';
    }

    static inline void save() {
        if (Mcu::Stm32::savecfg(eeprom, eeprom_flash)) {
            IO::outl<trace>("# EEPROM OK");
        }
        else {
            IO::outl<trace>("# EEPROM NOK");
        }
    }
    static inline void setParameter(const uint8_t index, const uint8_t value) {
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
    static inline RC::Protokoll::Crsf::Parameter parameter(const uint8_t index) {
        IO::outl<trace>("# GetP i: ", index);
        if ((index >= 1) && (index <= params.size())) {
            return params[index - 1];
        }
        return {};
    }
    static inline RC::Protokoll::Crsf::Parameter& parameterRef(const uint8_t index) {
        IO::outl<trace>("# GetPRef i: ", index);
        return params[index - 1];
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
    static inline constexpr uint32_t mSWVersion{10};
    static inline std::array<char, 16> mPwmFreqString{};
    static inline std::array<char, 16> mMaxRpmString{};
    static inline std::array<char, 16> mResistanceString{};
    static inline std::array<char, 16> mInductanceString{};
    static inline std::array<char, 16> mKmString{};
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

    static inline bool sendCalibrateEvent(const uint8_t v) {
        if (v == 1) {
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
    using params_t = etl::FixedVector<Param_t, 64>;
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::U8,  "PolePairs", nullptr, &eeprom.telemetry_polepairs, 2, 12, [](const uint8_t v){update(); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Cutoff_Freq [100Hz]", nullptr, &eeprom.cutoff_freq, 5, 15, [](const uint8_t v){update(); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Sample_Freq / Cutoff_Freq", nullptr, &eeprom.n_fsample, 2, 4, [](const uint8_t v){update(); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Pwm_Freq / Sample_Freq", nullptr, &eeprom.subsampling, 4, 16, [](const uint8_t v){update(); return true;}});
        addNode(p, Param_t{0, PType::U8,  "Inertia", nullptr, &eeprom.inertia, 0, 9, [](const uint8_t v){speed::dutyFilter.factor((10 - v) * 0.1f); return true;}});
        addNode(p, Param_t{0, PType::Sel, "PWM Freq", &mPwmFreqString[0], nullptr, 0, 0});
        addNode(p, Param_t{0, PType::Sel, "Max RPM", &mMaxRpmString[0], nullptr, 0, 0});
        addNode(p, Param_t{0, PType::Command, "Calibrate", "Calibrating...", nullptr, 0, 0, [](uint8_t v){return sendCalibrateEvent(v); }});
        addNode(p, Param_t{0, PType::Sel, "Resistance [mOhm]", &mResistanceString[0], nullptr, 0, 0});
        addNode(p, Param_t{0, PType::Sel, "Inductance [uH]", &mInductanceString[0], nullptr, 0, 0});
        addNode(p, Param_t{0, PType::Sel, "Km [Rpm/V]", &mKmString[0], nullptr, 0, 0});
        return p;
    }();
};
