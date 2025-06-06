#define USE_MCU_STM_V3
#define USE_CRSF_V2

// #define TEST1
// #define SERIAL_DEBUG

#define NDEBUG
//#define USE_SWD // for SWD debugging (do not reconfigure SWD/SWCLK Pins)

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
    }
    struct Output {
        uint8_t pwm = 0;
        uint8_t pwmDuty = 1;
        uint8_t pwmScale = 0;
        uint8_t blink = 0;
        uint8_t blinkOnTime = 1;
        uint8_t blinkOffTime = 1;
        std::array<char, 16> name;
    };

    uint8_t magic = EEPROM_MAGIC;

    uint8_t address = 0;
    uint8_t pwm1 = 10;
    uint8_t pwm2 = 10;
    uint8_t pwm3 = 10;
    uint8_t pwm4 = 10;
    uint8_t crsf_address = 0xc8;
    uint8_t response_slot = 8;
    uint8_t telemetry = 0;

    std::array<Output, 8> outputs{};
};

__attribute__((__section__(".eeprom"))) constexpr EEProm eeprom_flash{};

namespace {
    EEProm eeprom;
    bool mismatch = false;
}

using namespace std::literals::chrono_literals;

template<typename Out, typename Timer>
struct CrsfTelemetry {
    using out = Out;
    static inline constexpr External::Tick<Timer> telemTicks{100ms};

    enum class State : uint8_t {Idle, Gps, Batt, Temp1, Temp2, Rpm1, Rpm2};

    enum class Event : uint8_t {None, SendNext};

    static inline void event(const Event e) {
        mEvent = e;
    }

    static inline void ratePeriodic() {
        const Event e = std::exchange(mEvent, Event::None);
        (++mTelemTick).on(telemTicks, []{
            switch(mState) {
            case State::Idle:
                break;
            case State::Gps:
                out::data(RC::Protokoll::Crsf::Type::Gps, mGps); // crsf gps data
                mState = State::Batt;
                break;
            case State::Batt:
                out::data(RC::Protokoll::Crsf::Type::Battery, mBatt);
                mState = State::Temp1;
                break;
            case State::Temp1:
                out::data(RC::Protokoll::Crsf::Type::Temp1, mTemp);
                mState = State::Temp2;
                break;
            case State::Temp2:
                out::data(RC::Protokoll::Crsf::Type::Temp1, mTemp);
                mState = State::Rpm1;
                break;
            case State::Rpm1:
                out::data(RC::Protokoll::Crsf::Type::Rpm1, mRpm);
                mState = State::Rpm2;
                break;
            case State::Rpm2:
                out::data(RC::Protokoll::Crsf::Type::Rpm1, mRpm);
                mState = State::Gps;
                break;
            }
        });
    }
    private:
    static inline State mState{State::Idle};
    static inline Event mEvent{Event::None};

    static inline std::array<std::byte, 15> mGps{};
    static inline std::array<std::byte, 8> mBatt{}; //volts amps mAh percent
    static inline std::array<std::byte, 2> mTemp{};
    static inline std::array<std::byte, 2> mRpm{};
    static inline External::Tick<Timer> mTelemTick;
};

template<typename Config, typename SwitchCallback,
         typename Trace = void>
struct CrsfCallback {
    using trace = Trace;
    using Param_t = RC::Protokoll::Crsf::Parameter<>;
    using PType = RC::Protokoll::Crsf::Parameter<>::Type;

    using bsws = Config::bswList;
    using pwms = Config::pwmList;
    using telem_out = Config::telem_out;
    using timer = Config::timer;

    using pa = Config::pa;
    using responder = pa::Responder;

    using crsfTelemetry = CrsfTelemetry<telem_out, timer>;

    static inline constexpr const char* const title = "MultiSwitch-E@";

    using name_t = std::array<char, 32>;

    enum class State : uint8_t {Undefined, GotStats, GotChannels};

    static inline State mStreamState{State::Undefined};

    static inline constexpr void disableTelemetry() {
    }
    static inline constexpr void gotLinkStats() {
    }
    static inline constexpr void gotChannels() {
    }
    static inline constexpr void ratePeriodic() {
        // crsfTelemetry::ratePeriodic();
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
        if (auto [ok, err] = Mcu::Stm32::savecfg(eeprom, eeprom_flash); ok) {
            IO::outl<trace>("# EEPROM OK");
        }
        else {
            IO::outl<trace>("# EEPROM NOK: ", err);
        }
    }
    static inline void setParameter(const uint8_t index, const uint8_t value, const auto& data, const uint8_t dlen) {
        IO::outl<trace>("# SetP i: ", index, " v: ", value);
        if ((index >= 1) && (index <= params.size())) {
            mLastChangedParameter = index;
            bool mustSave = true;
            if (params[index - 1].mType == Param_t::Str) {
                IO::outl<trace>("# String");
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
        const uint8_t destAddress = payload[0];
        const uint8_t srcAddress = payload[1];
        const uint8_t realm = payload[2];
        const uint8_t cmd = payload[3];
        const uint8_t address = (uint8_t)payload[4];
        if ((srcAddress == (uint8_t)RC::Protokoll::Crsf::Address::Handset) && (destAddress >= 0xc0) && (destAddress <= 0xcf)) {
            if (realm == (uint8_t)RC::Protokoll::Crsf::CommandType::Switch) {
                if (eeprom.address == address) {
                    if (cmd == (uint8_t)RC::Protokoll::Crsf::SwitchCommand::Set) {
                        const uint8_t sw = (uint8_t)payload[5];
                        IO::outl<trace>("# Cmd Set: ", address, " sw: ", sw);
                        SwitchCallback::set(sw);
                    }
                    else if (cmd == (uint8_t)RC::Protokoll::Crsf::SwitchCommand::Prop) {
                        const uint8_t ch = (uint8_t)payload[5];
                        const uint8_t duty = (uint8_t)payload[6];
                        IO::outl<trace>("# Cmd Prop: ", address, " ch: ", ch, " d: ", duty);
                        SwitchCallback::prop(ch, duty);
                    }
                }
            }
        }
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
    static inline uint8_t mLastChangedParameter{};
    static inline constexpr uint32_t mSerialNumber{1234};
    static inline constexpr uint32_t mHWVersion{1};
    static inline constexpr uint32_t mSWVersion{13};
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
        return c.size();
    }
    static inline void addNode(auto& c, const Param_t& p) {
        c.push_back(p);
    }

    static inline bool setAddress(const uint8_t) {
        update();
        return true;
    }

    static inline auto mNameTest = []{
        std::array<char, 32> a;
        strcpy(&a[0], "bla");
        return a;
    }();

    using params_t = etl::FixedVector<Param_t, 128>;
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
#ifdef TEST1
        // String-type not supported by elrsv3.lua (Issue 2859)
        addNode(p, Param_t{0, PType::Str, "Name", nullptr, nullptr, 0, 0, nullptr, 0, 8, 0, &mNameTest[0]}); // maxlen 8
        addNode(p, Param_t{0, PType(PType::U8), "Address", nullptr, &eeprom.address, 0, 255, setAddress});
        addNode(p, Param_t{0, PType(PType::Str | PType::Hidden), "N0", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &mNameTest[0]});
        addNode(p, Param_t{0, PType(PType::Str | PType::Hidden), "N2", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &mNameTest[0]});
        addNode(p, Param_t{0, PType(PType::Str | PType::Hidden), "N3", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &mNameTest[0]});
        addNode(p, Param_t{0, PType(PType::U8 | PType::Hidden), "HiddenInfo", nullptr, nullptr, 0, 255});
#endif
#ifndef TEST1
        auto parent = addParent(p, Param_t{0, PType::Folder, "Global"});
        addNode(p, Param_t{parent, PType::U8, "Address", nullptr, &eeprom.address, 0, 255, setAddress});
        addNode(p, Param_t{parent, PType::U8, "PWM Freq G1 (O3,2)[100Hz]", nullptr, &eeprom.pwm1, 1, 200, [](const uint8_t v){Meta::nth_element<0, pwms>::freqCenties(v); return true;}});
        addNode(p, Param_t{parent, PType::U8, "PWM Freq G2 (O1,4,5,6)[100Hz]", nullptr, &eeprom.pwm2, 1, 200, [](const uint8_t v){Meta::nth_element<1, pwms>::freqCenties(v); return true;}});
        addNode(p, Param_t{parent, PType::U8, "PWM Freq G3 (O7)[100Hz]", nullptr, &eeprom.pwm3, 1, 200, [](const uint8_t v){Meta::nth_element<2, pwms>::freqCenties(v); return true;}});
        addNode(p, Param_t{parent, PType::U8, "PWM Freq G4 (O0)[100Hz]", nullptr, &eeprom.pwm4, 1, 200, [](const uint8_t v){Meta::nth_element<3, pwms>::freqCenties(v); return true;}});
        addNode(p, Param_t{parent, PType::U8, "CRSF Address", nullptr, &eeprom.crsf_address, 0xc0, 0xcf, [](const uint8_t v){
                               if (!mEepromMode) {
                                   const uint8_t slot = 2 * (v - 0xc0);
                                   eeprom.response_slot = slot;
                                   responder::telemetrySlot(slot);
                               }
                               responder::address(std::byte{v});
                               return true;
                           }});
        addNode(p, Param_t{parent, PType::U8, "Response Slot", nullptr, &eeprom.response_slot, 0, 15, [](const uint8_t v){responder::telemetrySlot(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Config resp.", "Button;Allways on", &eeprom.telemetry, 0, 1});

        parent = addParent(p, Param_t{0, PType::Folder, &eeprom.outputs[0].name[0]});
        // parent = addParent(p, Param_t{0, PType::Folder, "Output 0"});
        addNode(p, Param_t{parent, PType::Info, "Output 0 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Str, "Alias", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &eeprom.outputs[0].name[0]}); // not supported by elrsv3.lua?
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On;Remote", &eeprom.outputs[0].pwm, 0, 2, [](const uint8_t v){Meta::nth_element<0, bsws>::pwm(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, &eeprom.outputs[0].pwmDuty, 1, 99, [](const uint8_t v){Meta::nth_element<0, bsws>::duty((eeprom.outputs[0].pwm == 2)?0:v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, &eeprom.outputs[0].pwmScale, 0, 100, [](const uint8_t v){Meta::nth_element<0, bsws>::expo(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", &eeprom.outputs[0].blink, 0, 1, [](const uint8_t v){Meta::nth_element<0, bsws>::blink(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, &eeprom.outputs[0].blinkOnTime, 1, 255, [](const uint8_t v){Meta::nth_element<0, bsws>::on_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, &eeprom.outputs[0].blinkOffTime, 1, 255, [](const uint8_t v){Meta::nth_element<0, bsws>::off_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t v){Meta::nth_element<0, bsws>::on(v); return false;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 1"});
        addNode(p, Param_t{parent, PType::Info, "Output 1 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On;Remote", &eeprom.outputs[1].pwm, 0, 2, [](const uint8_t v){Meta::nth_element<1, bsws>::pwm(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, &eeprom.outputs[1].pwmDuty, 1, 99, [](const uint8_t v){Meta::nth_element<1, bsws>::duty((eeprom.outputs[1].pwm == 2)?0:v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, &eeprom.outputs[1].pwmScale, 0, 100, [](const uint8_t v){Meta::nth_element<1, bsws>::expo(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", &eeprom.outputs[1].blink, 0, 1, [](const uint8_t v){Meta::nth_element<1, bsws>::blink(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, &eeprom.outputs[1].blinkOnTime, 1, 255, [](const uint8_t v){Meta::nth_element<1, bsws>::on_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, &eeprom.outputs[1].blinkOffTime, 1, 255, [](const uint8_t v){Meta::nth_element<1, bsws>::off_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t v){Meta::nth_element<1, bsws>::on(v); return false;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 2"});
        addNode(p, Param_t{parent, PType::Info, "Output 2 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On;Remote", &eeprom.outputs[2].pwm, 0, 2, [](const uint8_t v){Meta::nth_element<2, bsws>::pwm(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, &eeprom.outputs[2].pwmDuty, 1, 99, [](const uint8_t v){Meta::nth_element<2, bsws>::duty((eeprom.outputs[2].pwm == 2)?0:v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, &eeprom.outputs[2].pwmScale, 0, 100, [](const uint8_t v){Meta::nth_element<2, bsws>::expo(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", &eeprom.outputs[2].blink, 0, 1, [](const uint8_t v){Meta::nth_element<2, bsws>::blink(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, &eeprom.outputs[2].blinkOnTime, 1, 255, [](const uint8_t v){Meta::nth_element<2, bsws>::on_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, &eeprom.outputs[2].blinkOffTime, 1, 255, [](const uint8_t v){Meta::nth_element<2, bsws>::off_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t v){Meta::nth_element<2, bsws>::on(v); return false;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 3"});
        addNode(p, Param_t{parent, PType::Info, "Output 3 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On;Remote", &eeprom.outputs[3].pwm, 0, 2, [](const uint8_t v){Meta::nth_element<3, bsws>::pwm(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, &eeprom.outputs[3].pwmDuty, 1, 99, [](const uint8_t v){Meta::nth_element<3, bsws>::duty((eeprom.outputs[3].pwm == 2)?0:v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, &eeprom.outputs[3].pwmScale, 0, 100, [](const uint8_t v){Meta::nth_element<3, bsws>::expo(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", &eeprom.outputs[3].blink, 0, 1, [](const uint8_t v){Meta::nth_element<3, bsws>::blink(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, &eeprom.outputs[3].blinkOnTime, 1, 255, [](const uint8_t v){Meta::nth_element<3, bsws>::on_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, &eeprom.outputs[3].blinkOffTime, 1, 255, [](const uint8_t v){Meta::nth_element<3, bsws>::off_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t v){Meta::nth_element<3, bsws>::on(v); return false;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 4"});
        addNode(p, Param_t{parent, PType::Info, "Output 4 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On;Remote", &eeprom.outputs[4].pwm, 0, 2, [](const uint8_t v){Meta::nth_element<4, bsws>::pwm(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, &eeprom.outputs[4].pwmDuty, 1, 99, [](const uint8_t v){Meta::nth_element<4, bsws>::duty((eeprom.outputs[4].pwm == 2)?0:v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, &eeprom.outputs[4].pwmScale, 0, 100, [](const uint8_t v){Meta::nth_element<4, bsws>::expo(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", &eeprom.outputs[4].blink, 0, 1, [](const uint8_t v){Meta::nth_element<4, bsws>::blink(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, &eeprom.outputs[4].blinkOnTime, 1, 255, [](const uint8_t v){Meta::nth_element<4, bsws>::on_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, &eeprom.outputs[4].blinkOffTime, 1, 255, [](const uint8_t v){Meta::nth_element<4, bsws>::off_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 4, [](const uint8_t v){Meta::nth_element<4, bsws>::on(v); return false;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 5"});
        addNode(p, Param_t{parent, PType::Info, "Output 5 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On;Remote", &eeprom.outputs[5].pwm, 0, 2, [](const uint8_t v){Meta::nth_element<5, bsws>::pwm(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, &eeprom.outputs[5].pwmDuty, 1, 99, [](const uint8_t v){Meta::nth_element<5, bsws>::duty((eeprom.outputs[5].pwm == 2)?0:v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, &eeprom.outputs[5].pwmScale, 0, 100, [](const uint8_t v){Meta::nth_element<5, bsws>::expo(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", &eeprom.outputs[5].blink, 0, 1, [](const uint8_t v){Meta::nth_element<5, bsws>::blink(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, &eeprom.outputs[5].blinkOnTime, 1, 255, [](const uint8_t v){Meta::nth_element<5, bsws>::on_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, &eeprom.outputs[5].blinkOffTime, 1, 255, [](const uint8_t v){Meta::nth_element<5, bsws>::off_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t v){Meta::nth_element<5, bsws>::on(v); return false;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 6"});
        addNode(p, Param_t{parent, PType::Info, "Output 6 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On;Remote", &eeprom.outputs[6].pwm, 0, 2, [](const uint8_t v){Meta::nth_element<6, bsws>::pwm(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, &eeprom.outputs[6].pwmDuty, 1, 99, [](const uint8_t v){Meta::nth_element<6, bsws>::duty((eeprom.outputs[6].pwm == 2)?0:v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, &eeprom.outputs[6].pwmScale, 0, 100, [](const uint8_t v){Meta::nth_element<6, bsws>::expo(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", &eeprom.outputs[6].blink, 0, 1, [](const uint8_t v){Meta::nth_element<6, bsws>::blink(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, &eeprom.outputs[6].blinkOnTime, 1, 255, [](const uint8_t v){Meta::nth_element<6, bsws>::on_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, &eeprom.outputs[6].blinkOffTime, 1, 255, [](const uint8_t v){Meta::nth_element<6, bsws>::off_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t v){Meta::nth_element<6, bsws>::on(v); return false;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 7"});
        addNode(p, Param_t{parent, PType::Info, "Output 7 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On;Remote", &eeprom.outputs[7].pwm, 0, 2, [](const uint8_t v){Meta::nth_element<7, bsws>::pwm(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, &eeprom.outputs[7].pwmDuty, 1, 99, [](const uint8_t v){Meta::nth_element<7, bsws>::duty((eeprom.outputs[7].pwm == 2)?0:v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, &eeprom.outputs[7].pwmScale, 0, 100, [](const uint8_t v){Meta::nth_element<7, bsws>::expo(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", &eeprom.outputs[7].blink, 0, 1, [](const uint8_t v){Meta::nth_element<7, bsws>::blink(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, &eeprom.outputs[7].blinkOnTime, 1, 255, [](const uint8_t v){Meta::nth_element<7, bsws>::on_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, &eeprom.outputs[7].blinkOffTime, 1, 255, [](const uint8_t v){Meta::nth_element<7, bsws>::off_dezi(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t v){Meta::nth_element<7, bsws>::on(v); return false;}});

        parent = addParent(p, Param_t{0, PType::Folder, "Operate"});
        addNode(p, Param_t{parent, PType::Sel, "Output 0", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<0, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 1", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<1, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 2", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<2, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 3", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<3, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 4", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<4, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 5", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<5, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 6", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<6, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 7", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<7, bsws>::on(v); return false;}});
#endif
        return p;
    }();
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using crsf = devs::crsf;
    using crsf_out = devs::crsf_out;
    using crsf_pa = devs::crsf_pa;
    using led = devs::ledBlinker;
    using btn = devs::btn;
    using debug = devs::debug;
    using debugtx = devs::debugtx;
    using crsfCallback = devs::crsfCallback;

    using bsw0 = devs::bsw0;
    using bsw1 = devs::bsw1;
    using bsw2 = devs::bsw2;
    using bsw3 = devs::bsw3;
    using bsw4 = devs::bsw4;
    using bsw5 = devs::bsw5;
    using bsw6 = devs::bsw6;
    using bsw7 = devs::bsw7;
    using bsws = devs::bsws;

    enum class State : uint8_t {Undefined, Init,
                                RunNoTelemetry, RunWithTelemetry};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};

    static inline void update(const bool eepromMode = true) {
        crsfCallback::callbacks(eepromMode);
    }
    static inline void prop(const uint8_t channel, const uint8_t duty) {
        IO::outl<debug>("# prop: ", channel, " duty: ", duty);
        if (channel < 8) {
            Meta::visitAt<bsws>(channel, [&]<typename SW>(Meta::Wrapper<SW>){
                                    SW::duty(duty);
                                });
        }
    }
    static inline void set(const uint8_t sw) {
        // IO::outl<debug>("# set: ", sw);
        for(uint8_t i = 0; i < 8; ++i) {
            const uint8_t mask = (0x01 << i);
            Meta::visitAt<bsws>(i, [&]<typename SW>(Meta::Wrapper<SW>){
                if (sw & mask) {
                    // IO::outl<debug>("# on: ", i);
                    SW::event(SW::Event::On);
                }
                else {
                    // IO::outl<debug>("# off: ", i);
                    SW::event(SW::Event::Off);
                }
            });
        }
    }

    static inline void init() {
        devs::init();
        if constexpr(!std::is_same_v<debug, void>) {
            debug::init();
            debug::template rxEnable<false>();
            debug::baud(115'200);
            debugtx::afunction(0);
        }
    }
    static inline void periodic() {
        crsf::periodic();
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
    }
    static inline void ratePeriodic() {
        led::ratePeriodic();
        btn::ratePeriodic();

        crsf_out::ratePeriodic();

        if (eeprom.telemetry) {
            crsfCallback::ratePeriodic();
        }

        crsf_pa::ratePeriodic([](const bool connected){
            static bool mLast = false;
            if (!etl::equalStore(mLast, connected)) {
                if (connected) {
                    led::event(led::Event::Slow);
                }
                else {
                    led::event(led::Event::Fast);
                }
            }
        });

        bsw0::ratePeriodic();
        bsw1::ratePeriodic();
        bsw2::ratePeriodic();
        bsw3::ratePeriodic();
        bsw4::ratePeriodic();
        bsw5::ratePeriodic();
        bsw6::ratePeriodic();
        bsw7::ratePeriodic();

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
                if (eeprom.telemetry) {
                    mState = State::RunWithTelemetry;
                }
                else {
                    mState = State::RunNoTelemetry;
                }
            });
            break;
        case State::RunNoTelemetry:
            if (const auto e = btn::event(); e == btn::Press::Long) {
                mState = State::RunWithTelemetry;
            }
            break;
        case State::RunWithTelemetry:
            if (const auto e = btn::event(); e == btn::Press::Long) {
                if (eeprom.telemetry) {
                    led::count(2);
                    led::event(led::Event::Slow);
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
                IO::outl<debug>("# Init eep magic: ", eeprom.magic, " mism: ", (uint8_t)mismatch);
                break;
            case State::RunNoTelemetry:
                IO::outl<debug>("# Run NT");
                IO::outl<debug>("# adr: ", eeprom.address);
                crsf_pa::enableReply(false);
                led::event(led::Event::Slow);
                break;
            case State::RunWithTelemetry:
                IO::outl<debug>("# Run WT");
                IO::outl<debug>("# adr: ", eeprom.address);
                crsf_pa::enableReply(true);
                if (eeprom.telemetry) {
                    led::count(2);
                    led::event(led::Event::Slow);
                }
                else {
                    led::event(led::Event::Steady);
                }
            }
        }
    }
    private:
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState = State::Undefined;
};

struct Setter;

template<typename L, typename T>
using CrsfCallback_WithSetter = CrsfCallback<L, Setter, T>;

using devs = Devices2<SW12, CrsfCallback_WithSetter>;
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
    if (eeprom_flash.magic == EEPROM_MAGIC) {
        std::memcpy(&eeprom, &eeprom_flash, sizeof(EEProm));
        mismatch = false;
    }
    else {
        mismatch = true;
    }

    gfsm::init();
    gfsm::update(true);

    NVIC_EnableIRQ(TIM3_IRQn);
    __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

extern "C" {

void TIM3_IRQHandler() {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR = ~TIM_SR_UIF;
        devs::bsw1::set();
        devs::bsw6::set();
    }
    if (TIM3->SR & TIM_SR_CC3IF) {
        TIM3->SR = ~TIM_SR_CC3IF;
        devs::bsw1::reset();
    }
    if (TIM3->SR & TIM_SR_CC4IF) {
        TIM3->SR = ~TIM_SR_CC4IF;
        devs::bsw6::reset();
    }
}
}

void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}
