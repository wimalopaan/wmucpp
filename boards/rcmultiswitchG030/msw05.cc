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

    static inline constexpr const char* const title = "MultiSwitch-E @ ";

    static inline constexpr uint8_t addressIndex = 1;

    using name_t = std::array<char, 32>;

    static inline constexpr void updateName(name_t& n) {
        strncpy(&n[0], title, n.size());
        std::to_chars(std::begin(n) + strlen(title), std::end(n), params[addressIndex].mValue);
    }
    static inline void update() {
        updateName(mName);
    }

    static inline void setParameter(const uint8_t index, const uint8_t value) {
        IO::outl<trace>("SetP i: ", index, " v: ", value);
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

    using params_t = etl::FixedVector<Param_t, 128>;
    static inline params_t params = []{
        params_t p;
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        addNode(p, Param_t{0, PType::U8, "Address", nullptr, 0, 0, 255});

        auto parent = addParent(p, Param_t{0, PType::Folder, "Output 0"});
        addNode(p, Param_t{parent, PType::Info, "Output 0 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On", 0, 0, 1, Meta::nth_element<0, CbList>::pwm});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<0, CbList>::duty});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, 0, 0, 100, Meta::nth_element<0, CbList>::expo});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", 0, 0, 1, Meta::nth_element<0, CbList>::blink});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<0, CbList>::on_dezi});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<0, CbList>::off_dezi});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", 0, 0, 1, Meta::nth_element<0, CbList>::on});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 1"});
        addNode(p, Param_t{parent, PType::Info, "Output 1 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On", 0, 0, 1, Meta::nth_element<1, CbList>::pwm});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<1, CbList>::duty});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, 0, 0, 100, Meta::nth_element<1, CbList>::expo});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", 0, 0, 1, Meta::nth_element<1, CbList>::blink});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<1, CbList>::on_dezi});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<1, CbList>::off_dezi});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", 0, 0, 1, Meta::nth_element<1, CbList>::on});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 2"});
        addNode(p, Param_t{parent, PType::Info, "Output 2 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On", 0, 0, 1, Meta::nth_element<2, CbList>::pwm});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<2, CbList>::duty});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, 0, 0, 100, Meta::nth_element<2, CbList>::expo});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", 0, 0, 1, Meta::nth_element<2, CbList>::blink});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<2, CbList>::on_dezi});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<2, CbList>::off_dezi});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", 0, 0, 1, Meta::nth_element<2, CbList>::on});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 3"});
        addNode(p, Param_t{parent, PType::Info, "Output 3 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On", 0, 0, 1, Meta::nth_element<3, CbList>::pwm});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<3, CbList>::duty});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, 0, 0, 100, Meta::nth_element<3, CbList>::expo});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", 0, 0, 1, Meta::nth_element<3, CbList>::blink});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<3, CbList>::on_dezi});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<3, CbList>::off_dezi});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", 0, 0, 1, Meta::nth_element<3, CbList>::on});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 4"});
        addNode(p, Param_t{parent, PType::Info, "Output 4 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On", 0, 0, 1, Meta::nth_element<4, CbList>::pwm});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<4, CbList>::duty});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, 0, 0, 100, Meta::nth_element<4, CbList>::expo});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", 0, 0, 1, Meta::nth_element<4, CbList>::blink});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<4, CbList>::on_dezi});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<4, CbList>::off_dezi});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", 0, 0, 4, Meta::nth_element<4, CbList>::on});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 5"});
        addNode(p, Param_t{parent, PType::Info, "Output 5 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On", 0, 0, 1, Meta::nth_element<5, CbList>::pwm});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<5, CbList>::duty});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, 0, 0, 100, Meta::nth_element<5, CbList>::expo});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", 0, 0, 1, Meta::nth_element<5, CbList>::blink});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<5, CbList>::on_dezi});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<5, CbList>::off_dezi});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", 0, 0, 1, Meta::nth_element<5, CbList>::on});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 6"});
        addNode(p, Param_t{parent, PType::Info, "Output 6 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On", 0, 0, 1, Meta::nth_element<6, CbList>::pwm});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<6, CbList>::duty});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, 0, 0, 100, Meta::nth_element<6, CbList>::expo});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", 0, 0, 1, Meta::nth_element<6, CbList>::blink});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<6, CbList>::on_dezi});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<6, CbList>::off_dezi});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", 0, 0, 1, Meta::nth_element<6, CbList>::on});

        parent = addParent(p, Param_t{0, PType::Folder, "Output 7"});
        addNode(p, Param_t{parent, PType::Info, "Output 7 : ", &mName[0]});
        addNode(p, Param_t{parent, PType::Sel, "PWM Mode", "Off;On", 0, 0, 1, Meta::nth_element<7, CbList>::pwm});
        addNode(p, Param_t{parent, PType::U8,  "PWM Duty", nullptr, 0, 0, 100, Meta::nth_element<7, CbList>::duty});
        addNode(p, Param_t{parent, PType::U8,  "PWM Expo", nullptr, 0, 0, 100, Meta::nth_element<7, CbList>::expo});
        addNode(p, Param_t{parent, PType::Sel, "Intervall Mode", "Off;On", 0, 0, 1, Meta::nth_element<7, CbList>::blink});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(on)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<7, CbList>::on_dezi});
        addNode(p, Param_t{parent, PType::U8,  "Intervall(off)[0.1s]", nullptr, 0, 0, 255, Meta::nth_element<7, CbList>::off_dezi});
        addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", 0, 0, 1, Meta::nth_element<7, CbList>::on});

        return p;
    }();
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using crsf = devs::crsf;
    using crsf_out = devs::crsf_out;
    using led = devs::led;
    using debug = devs::debug;
    using debugtx = devs::debugtx;

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
        if constexpr(!std::is_same_v<debug, void>) {
            debug::init();
            debug::baud(420'000);
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

struct Setter;
template<typename L, typename T>
using CrsfCallback_WithSetter = CrsfCallback<L, Setter, T>;

using devs = Devices2<SW10, CrsfCallback_WithSetter>;
using gfsm = GFSM<devs>;

struct Setter {
    static inline void set(const uint8_t sw) {
        gfsm::set(sw);
    }
};

int main() {
    gfsm::init();

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
    if (TIM3->SR & TIM_SR_CC1IF) {
        TIM3->SR &= ~TIM_SR_CC1IF;
    }
    if (TIM3->SR & TIM_SR_CC2IF) {
        TIM3->SR &= ~TIM_SR_CC2IF;
    }
    if (TIM3->SR & TIM_SR_CC3IF) {
        TIM3->SR &= ~TIM_SR_CC3IF;
        devs::bsw1::set();
    }
    if (TIM3->SR & TIM_SR_CC4IF) {
        TIM3->SR &= ~TIM_SR_CC4IF;
        devs::bsw6::set();
    }
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;
        devs::bsw1::reset();
        devs::bsw6::reset();
    }
}
}

void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}
