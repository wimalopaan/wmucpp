#define USE_MCU_STM_V3
#define USE_CRSF_V2

#define NDEBUG

#include <cstdint>

#include "devices.h"

using namespace std::literals::chrono_literals;

template<typename CbList, typename SwitchCallback, typename Trace = void>
struct CrsfCallback {
    using trace = Trace;
    using Param_t = RC::Protokoll::Crsf::Parameter;
    using PType = RC::Protokoll::Crsf::Parameter::Type;

    static inline constexpr const char* const title = "MultiSwitch-E@";

    static inline constexpr uint8_t addressIndex = 1;

    static inline constexpr void updateName(auto& n) {
        strncpy(&n[0], title, n.size());
        std::to_chars(std::begin(n) + strlen(title), std::end(n), params[addressIndex].mValue);
    }

    static inline constexpr void update() {
        updateName(mName);
    }

    static inline void setParameter(const uint8_t index, const uint8_t value) {
        IO::outl<trace>("SetP adr: ", index, " v: ", value);
        if ((index >= 1) && (index <= params.size())) {
            params[index - 1].mValue = value;
            mLastChangedParameter = index;
            if (params[index - 1].cb) {
                params[index - 1].cb(value);
            }
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
                    IO::outl<trace>("Command: ", address, " sw: ", sw);
                    SwitchCallback::set(sw);
                }
            }
        }
    }
private:
    static inline uint8_t mLastChangedParameter{};
    static inline constexpr uint32_t mSerialNumber{1234};
    static inline constexpr uint32_t mHWVersion{1};
    static inline constexpr uint32_t mSWVersion{1};
    static inline constexpr auto mVersionString = [](){
        std::array<char, 16> s{};
        auto [ptr, e] = std::to_chars(std::begin(s), std::end(s), mHWVersion);
        *ptr++ = ':';
        std::to_chars(ptr, std::end(s), mSWVersion);
        return s;
    }();

    using name_t = std::array<char, 32>;
    static inline name_t mName = []{
        name_t name{};
        updateName(name);
        return name;
    }();

    // static inline uint8_t addParent(auto& c, const Param_t& p) {

    // }
    // static inline uint8_t addNode(auto& c, const Param_t& p) {

    // }
#if 0
    static inline auto params = []{
        etl::FixedVector<Param_t, 64> p;
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::U8, "Address", nullptr, 0, 0, 255});
        auto parent = addParent(p, Param_t{0, PType::Folder, "Output 0"});
        addNode(p, Param_t{parent, PType::Info, "Output 0 : ", &mName[0]});

            uint8_t index = 1; // attention
            {
                p.push_back(Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
                ++index;
                p.push_back(Param_t{0, PType::U8, "Address", nullptr, 0, 0, 255}); // addressIndex
                ++index;
            }
            {
                p.push_back(Param_t{0, PType::Folder, "Output 0"});
                const uint8_t parent = index;
                ++index;
                p.push_back(Param_t{parent, PType::Info, "Output 0 : ", &mName[0]});
                Param_t{3, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<0, CbList>::pwm},
                Param_t{3, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<0, CbList>::duty},
                Param_t{3, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<0, CbList>::blink},
                Param_t{3, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<0, CbList>::on_dezi},
                Param_t{3, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<0, CbList>::off_dezi},
            }

        Param_t{0, PType::Folder, "Output 1"}, // 10
        Param_t{10, PType::Info, "Output 1 : ", &mName[0]},
        Param_t{10, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<1, CbList>::pwm},
        Param_t{10, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<1, CbList>::duty},
        Param_t{10, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<1, CbList>::blink},
        Param_t{10, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<1, CbList>::on_dezi},
        Param_t{10, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<1, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 2"}, // 17
        Param_t{17, PType::Info, "Output 2 : ", &mName[0]},
        Param_t{17, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<2, CbList>::pwm},
        Param_t{17, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<2, CbList>::duty},
        Param_t{17, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<2, CbList>::blink},
        Param_t{17, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<2, CbList>::on_dezi},
        Param_t{17, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<2, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 3"}, // 24
        Param_t{24, PType::Info, "Output 3 : ", &mName[0]},
        Param_t{24, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<3, CbList>::pwm},
        Param_t{24, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<3, CbList>::duty},
        Param_t{24, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<3, CbList>::blink},
        Param_t{24, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<3, CbList>::on_dezi},
        Param_t{24, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<3, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 4"}, // 31
        Param_t{31, PType::Info, "Output 4 : ", &mName[0]},
        Param_t{31, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<4, CbList>::pwm},
        Param_t{31, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<4, CbList>::duty},
        Param_t{31, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<4, CbList>::blink},
        Param_t{31, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<4, CbList>::on_dezi},
        Param_t{31, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<4, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 5"}, // 38
        Param_t{38, PType::Info, "Output 5 : ", &mName[0]},
        Param_t{38, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<5, CbList>::pwm},
        Param_t{38, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<5, CbList>::duty},
        Param_t{38, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<5, CbList>::blink},
        Param_t{38, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<5, CbList>::on_dezi},
        Param_t{38, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<5, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 6"}, // 45
        Param_t{45, PType::Info, "Output 6 : ", &mName[0]},
        Param_t{45, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<6, CbList>::pwm},
        Param_t{45, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<6, CbList>::duty},
        Param_t{45, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<6, CbList>::blink},
        Param_t{45, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<6, CbList>::on_dezi},
        Param_t{45, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<6, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 7"}, // 52
        Param_t{52, PType::Info, "Output 7 : ", &mName[0]},
        Param_t{52, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<7, CbList>::pwm},
        Param_t{52, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<7, CbList>::duty},
        Param_t{52, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<7, CbList>::blink},
        Param_t{52, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<7, CbList>::on_dezi},
        Param_t{52, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<7, CbList>::off_dezi};

        return p;
    }();

#endif
#if 1
    static inline etl::FixedVector<Param_t, 64> params {
        Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]},
        Param_t{0, PType::U8, "Address", nullptr, 0, 0, 255}, // addressIndex

        Param_t{0, PType::Folder, "Output 0"}, // 3
        Param_t{3, PType::Info, "Output 0 : ", &mName[0]},
        Param_t{3, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<0, CbList>::pwm},
        Param_t{3, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<0, CbList>::duty},
        Param_t{3, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<0, CbList>::blink},
        Param_t{3, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<0, CbList>::on_dezi},
        Param_t{3, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<0, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 1"}, // 10
        Param_t{10, PType::Info, "Output 1 : ", &mName[0]},
        Param_t{10, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<1, CbList>::pwm},
        Param_t{10, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<1, CbList>::duty},
        Param_t{10, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<1, CbList>::blink},
        Param_t{10, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<1, CbList>::on_dezi},
        Param_t{10, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<1, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 2"}, // 17
        Param_t{17, PType::Info, "Output 2 : ", &mName[0]},
        Param_t{17, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<2, CbList>::pwm},
        Param_t{17, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<2, CbList>::duty},
        Param_t{17, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<2, CbList>::blink},
        Param_t{17, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<2, CbList>::on_dezi},
        Param_t{17, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<2, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 3"}, // 24
        Param_t{24, PType::Info, "Output 3 : ", &mName[0]},
        Param_t{24, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<3, CbList>::pwm},
        Param_t{24, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<3, CbList>::duty},
        Param_t{24, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<3, CbList>::blink},
        Param_t{24, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<3, CbList>::on_dezi},
        Param_t{24, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<3, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 4"}, // 31
        Param_t{31, PType::Info, "Output 4 : ", &mName[0]},
        Param_t{31, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<4, CbList>::pwm},
        Param_t{31, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<4, CbList>::duty},
        Param_t{31, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<4, CbList>::blink},
        Param_t{31, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<4, CbList>::on_dezi},
        Param_t{31, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<4, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 5"}, // 38
        Param_t{38, PType::Info, "Output 5 : ", &mName[0]},
        Param_t{38, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<5, CbList>::pwm},
        Param_t{38, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<5, CbList>::duty},
        Param_t{38, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<5, CbList>::blink},
        Param_t{38, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<5, CbList>::on_dezi},
        Param_t{38, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<5, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 6"}, // 45
        Param_t{45, PType::Info, "Output 6 : ", &mName[0]},
        Param_t{45, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<6, CbList>::pwm},
        Param_t{45, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<6, CbList>::duty},
        Param_t{45, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<6, CbList>::blink},
        Param_t{45, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<6, CbList>::on_dezi},
        Param_t{45, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<6, CbList>::off_dezi},

        Param_t{0, PType::Folder, "Output 7"}, // 52
        Param_t{52, PType::Info, "Output 7 : ", &mName[0]},
        Param_t{52, PType::Sel, "PWM", "Off;On", 0, 0, 1, Meta::nth_element<7, CbList>::pwm},
        Param_t{52, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<7, CbList>::duty},
        Param_t{52, PType::Sel, "Intervall", "Off;On", 0, 0, 1, Meta::nth_element<7, CbList>::blink},
        Param_t{52, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<7, CbList>::on_dezi},
        Param_t{52, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<7, CbList>::off_dezi},
    };
#endif
#if 0
    static inline etl::FixedVector<Param_t, 64> params {
        Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]},
        Param_t{0, PType::U8, "Address", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 0"}, // 3
        // todo: Info als Überschrift
        Param_t{3, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{3, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{3, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{3, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{3, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 1"}, // 9
        Param_t{9, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{9, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{9, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{9, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{9, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 2"}, // 15
        Param_t{15, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{15, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{15, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{15, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{15, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 3"}, // 21
        Param_t{21, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{21, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{21, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{21, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{21, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 4"}, // 27
        Param_t{27, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{27, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{27, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{27, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{27, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 5"}, // 33
        Param_t{33, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{33, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{33, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{33, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{33, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 6"}, // 39
        Param_t{39, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{39, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{39, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{39, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{39, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},

        Param_t{0, PType::Folder, "Output 7"}, // 45
        Param_t{45, PType::Sel, "PWM", "Off;On", 0, 0, 1},
        Param_t{45, PType::U8,  "PWM Duty", nullptr, 0, 0, 255},
        Param_t{45, PType::Sel, "Intervall", "Off;On", 0, 0, 1},
        Param_t{45, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255},
        Param_t{45, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255},
    };
#endif
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using crsf = devs::crsf;
    using crsf_pa = devs::crsf_pa;
    using crsf_out = devs::crsf_out;
    using led = devs::led;
    // using pb4 = devs::pb4;
    // using mco = devs::mco;
    using debug = devs::debug;

    using bsw0 = devs::bsw0;
    using bsw1 = devs::bsw1;
    using bsw2 = devs::bsw2;
    using bsw3 = devs::bsw3;
    using bsw4 = devs::bsw4;
    using bsw5 = devs::bsw5;
    using bsw6 = devs::bsw6;
    using bsw7 = devs::bsw7;
    using bsws = devs::bsws;

    enum class State : uint8_t {Undefined, Init, Run};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};

    static inline void set(const uint8_t sw) {
        IO::outl<debug>("set: ", sw);
        for(uint8_t i = 0; i < 8; ++i) {
            const uint8_t mask = (0x01 << i);
            Meta::visitAt<bsws>(i, [&]<typename SW>(Meta::Wrapper<SW>){
                if (sw & mask) {
                    IO::outl<debug>("on: ", i);
                    SW::event(SW::Event::On);
                }
                else {
                    IO::outl<debug>("off: ", i);
                    SW::event(SW::Event::Off);
                }
            });
        }
    }

    static inline void init() {
        devs::init();
    }
    static inline void periodic() {
        crsf::periodic();
        if constexpr(!std::is_same_v<debug, void>) {
            debug::periodic();
        }
        // mco::toggle();
    }
    static inline void ratePeriodic() {
        // pb4::toggle();
        crsf_out::ratePeriodic();

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
                mState = State::Run;
            });
            break;
        case State::Run:
            (++mDebugTick).on(debugTicks, []{
                led::toggle();
                // bsw0::event(bsw0::Event::On);
                // bsw1::event(bsw1::Event::On);
                // bsw2::event(bsw2::Event::On);
                // bsw3::event(bsw3::Event::On);
                // bsw4::event(bsw4::Event::On);
                // bsw5::event(bsw5::Event::On);
                // bsw6::event(bsw6::Event::On);
                // bsw7::event(bsw7::Event::On);
                IO::outl<debug>("bc: ", crsf_pa::mBytesCounter);
            });
            // crsfcallback::whenParameterChanged([](const uint8_t p){
            // });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                IO::outl<debug>("Init");
                break;
            case State::Run:
                IO::outl<debug>("Run");
                break;
            }
        }
    }
    private:
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState = State::Undefined;
};

struct S;
template<typename L, typename T>
using CrsfCallback_A = CrsfCallback<L, S, T>;

using devs = Devices2<SW10, CrsfCallback_A>;
using gfsm = GFSM<devs>;

struct S {
    static inline void set(const uint8_t sw) {
        gfsm::set(sw);
    }
};

int main() {
    gfsm::init();

    // NVIC_EnableIRQ(TIM3_IRQn);
    // __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}
void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}
