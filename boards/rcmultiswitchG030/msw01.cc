#define USE_MCU_STM_V3
#define USE_CRSF_V2

#define NDEBUG

#include <cstdint>

#include "devices.h"

using namespace std::literals::chrono_literals;

struct McuConfig {
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<64_MHz, 2'000_Hz, Mcu::Stm::HSI>>;
};

template<typename Trace>
struct CrsfCallback {
    using trace = Trace;
    using Param_t = RC::Protokoll::Crsf::Parameter;
    using PType = RC::Protokoll::Crsf::Parameter::Type;

    static inline void setParameter(const uint8_t index, const uint8_t value) {
        // IO::outl<trace>("SetP adr: ", index, " v: ", value);
        if ((index >= 1) && (index <= params.size())) {
            params[index - 1].mValue = value;
            mLastChangedParameter = index;
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
        return mName;
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
            if (payload[2] == (uint8_t)RC::Protokoll::Crsf::CommandType::CC) {
                if (payload[3] == (uint8_t)RC::Protokoll::Crsf::CcCommand::SetAltData) {
                    const uint8_t index = (uint8_t)payload[4];
                    // if (index < Data::switches.size()) {
                    //     Data::switches[index] = (uint8_t)payload[5];
                    // }
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
    static inline constexpr const char* const mName = "VarioProp";
    static inline etl::FixedVector<Param_t, 64> params {
        Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]},
        Param_t{0, PType::Sel, "RF", "Off;On", 0, 0, 1},
        Param_t{0, PType::Sel, "Override", "Off;On", 0, 0, 1},
        Param_t{0, PType::Sel, "Bluetooth", "Off;On", 0, 0, 1},
    };
};

struct Config {
    using mcuconfig = McuConfig;
    using crsfcallback = CrsfCallback<void>;
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;
    using crsf = devs::crsf;
    using led = devs::led;
    using pb4 = devs::pb4;
    using debug = devs::debug;

    enum class State : uint8_t {Undefined, Init, Run};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};

    static inline void init() {
        devs::init();
    }
    static inline void periodic() {
        crsf::periodic();
        debug::periodic();
    }
    static inline void ratePeriodic() {
        // pb4::toggle();
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

using devs = Devices<Nucleo, Config>;
using gfsm = GFSM<devs>;

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
