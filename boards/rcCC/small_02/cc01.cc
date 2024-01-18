#define USE_MCU_STM_V2
#define NDEBUG

// Usart 3
// #define USE_ESC
 #define USE_SPORT
// #define USE_VESC

// USART 2
// #define USE_DMA_SBUS

#include "devices.h"

#include <chrono>
#include <cassert>

using namespace std::literals::chrono_literals;

struct Config {
    Config() = delete;
};

struct Data {
    static inline std::array<uint16_t, 64> mChannels{}; // sbus [172, 1812], center = 992
    static inline std::array<uint8_t, 256> mAltData{};
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;

    using pwm1 = devs::pwm1;
    using crsf = devs::crsf;
    using crsf_pa = devs::crsf_pa;
    using crsf_out = devs::crsf_out;
    using crsfTelemetry = devs::crsfTelemetry;

#ifdef USE_SPORT
    using sport = devs::sport;
    using sport_pa = devs::sport_pa;
    using sport_uart = devs::sport_uart;
#endif
#ifdef USE_ESC
    using esc = devs::esc;
    using esc_pa = devs::esc_pa;
    using esc_uart = devs::esc_uart;
#endif
#ifdef USE_VESC
    using vesc = devs::vesc;
    using vesc_pa = devs::vesc_pa;
    using vesc_uart = devs::vesc_uart;
#endif

    using sbus = devs::sbus;

#ifndef USE_DMA_SBUS
    using sbus_pa = devs::sbus_pa;
#endif

    using bt_uart = devs::bt_uart;
    using robo_pa = devs::robo_pa;

    using led1 = devs::led1;
    using led2 = devs::led2;

    using btpwr = devs::btpwr;
    using bten = devs::bten;

    using tp1 = devs::tp1;
    using tp2 = devs::tp2;

    using i2c_2 = devs::i2c_2;
    using tlc_01 = devs::tlc_01;

    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> initTicks{500ms};

    enum class State : uint8_t {Undefined, Init, Info, DevsInit, Run};

    static inline void init() {
        devs::init();
        devs::led1::set();

        for(auto& v : Data::mChannels) {
            v = 992;
        }
    }
    static inline void periodic() {
        tp1::set();
        trace::periodic();

        i2c_2::periodic();

        crsf::periodic();
#ifdef USE_SPORT
        sport::periodic();
        sport_uart::periodic();
#endif
#ifdef USE_ESC
        esc::periodic();
        esc_uart::periodic();
#endif
#ifdef USE_VESC
        vesc::periodic();
        vesc_uart::periodic();
#endif

#ifndef USE_DMA_SBUS
        sbus::periodic();
#endif

        bt_uart::periodic();

        tp1::reset();
    }
    static inline void ratePeriodic() {
        static uint8_t c{0};

        tp2::set();

        i2c_2::ratePeriodic();

        robo_pa::ratePeriodic();

        crsf_pa::copyChangedChannels(Data::mChannels);

        robo_pa::whenTargetChanged([&](const robo_pa::Target t, const robo_pa::index_type::nan_type i){
            IO::outl<trace>("Robo");
            switch(t) {
            case robo_pa::Target::Prop:
                if (i < Data::mChannels.size()) {
                    Data::mChannels[i] = robo_pa::propValues[i];
                }
                break;
            case robo_pa::Target::Switch:
                if (i < Data::mAltData.size()) {
                    Data::mAltData[i] = robo_pa::switchValues[i];
                }
                break;
            case robo_pa::Target::Toggle:
                if (i < Data::mAltData.size()) {
                    Data::mAltData[i] = robo_pa::toggleValues[i];
                }
                break;
            default:
                break;
            }
        });

        sbus::set(std::span{&Data::mChannels[0], 16});

#ifdef USE_SPORT
        sport::ratePeriodic();
#endif
#ifdef USE_ESC
        esc::throttle(Data::mChannels[0]);
        esc::ratePeriodic();
#endif
#ifdef USE_VESC
        vesc::throttle(Data::mChannels[0]);
        vesc::ratePeriodic();
#endif

        crsf_out::ratePeriodic();
        crsfTelemetry::ratePeriodic();

#ifndef USE_DMA_SBUS
        sbus::ratePeriodic();
#endif

        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Init;
                btpwr::reset(); // on
            });
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                if (i2c_2::isIdle()) {
                    mState = State::Info;
                }
                else {
                    mStateTick.reset();
                }
            });
            break;
        case State::Info:
            for(uint8_t a{0}; a < 0x7f; ++a) {
                if (i2c_2::isPresent(I2C::Address{a})) {
                    IO::outl<trace>("I2C2: ", a);
                }
            }
            mState = State::DevsInit;
            break;
        case State::DevsInit:
            mStateTick.on(initTicks, []{
                if (i2c_2::isIdle()) {
                    mState = State::Run;
                }
                else {
                    mStateTick.reset();
                }
            });
            break;
        case State::Run:
            (++mDebugTick).on(debugTicks, []{
                led1::toggle();

                if (++c > 1) c = 0;

                if (c == 0) {
                    tlc_01::on(0);
                }
                else {
                    tlc_01::off(0);
                }


                // IO::outl<trace>(
                //             "b: ", crsf_pa::mBytesCounter
                //             );
                //             " p: ", crsf_pa::mPackagesCounter,
                //             " l: ", crsf_pa::mLinkPackagesCounter,
                //             " ch: ", crsf_pa::mChannelsPackagesCounter,
                //             " pg: ", crsf_pa::mPingPackagesCounter,
                //             " pe: ", crsf_pa::mParameterEntryPackagesCounter,
                //             " pr: ", crsf_pa::mParameterReadPackagesCounter,
                //             " pw: ", crsf_pa::mParameterWritePackagesCounter,
                //             " c: ", crsf_pa::mCommandPackagesCounter,
                //             " d: ", crsf_pa::mDataPackagesCounter);
                // IO::outl<trace>(
                //     "ch0: ", Data::mChannels[0],
                //     " ch1: ", Data::mChannels[1],
                //     " ch2: ", Data::mChannels[2],
                //     " ch3: ", Data::mChannels[3],
                //     " rpm: ", (uint8_t)sbus_pa::mSlots[0][1],
                //     " rpm: ", (uint8_t)sbus_pa::mSlots[0][2]
                //     );
#ifdef USE_VESC
                IO::outl<trace>(
                    "ch0: ", Data::mChannels[0],
                    " ch1: ", Data::mChannels[1],
                    " ch2: ", Data::mChannels[2],
                    " ch3: ", Data::mChannels[3],
                    " v b: ", vesc_pa::mBytes,
                    " v p: ", vesc_pa::mPackages,
                    " v t: ", (uint8_t)vesc_pa::mType,
                    " v l: ", vesc_pa::mLength
                    );
#endif
                IO::outl<trace>(
                            " ro0: ", robo_pa::propValues[0],
                            " ro1: ", robo_pa::propValues[1],
                            " ro2: ", robo_pa::propValues[2],
                            " ro3: ", robo_pa::propValues[3]
                        );

            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                i2c_2::scan(i2c_scan_callback);
                break;
            case State::Info:
                break;
            case State::DevsInit:
                tlc_01::init();
                break;
            case State::Run:
                led2::set();
                break;
            }
        }
        tp2::reset();
    }
    static inline void i2c_scan_callback(uint8_t) {

    }
private:
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState{State::Undefined};
};

using devs = Devices<CC03, Config, Mcu::Stm::Stm32G473>;

int main() {
    using gfsm = GFSM<devs>;
    gfsm::init();

    // NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
        devs::tp1::set();
        devs::tp1::reset();
    }
}

extern "C" {

void DMA1_Channel1_IRQHandler() {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}

}
