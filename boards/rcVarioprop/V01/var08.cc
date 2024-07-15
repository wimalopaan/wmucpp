#define USE_MCU_STM_V3
#define NDEBUG

#include "devices.h"

#include <chrono>
#include <cassert>

using namespace std::literals::chrono_literals;

struct Config {
};

struct Data {
    static inline std::array<uint16_t, 64> crsfChannels{}; // sbus [172, 1812], center = 992

    static inline std::array<uint16_t, 16> channels{}; // [0,1000]
    static inline std::array<uint8_t, 32> switches{}; // [0, 1, 2]
    static inline bool rfOn{false};
    static inline uint16_t voltage{}; // [0, 1600]: 10mV
    static inline uint16_t temperatur{}; // [0, 100]: 1°C
    static inline uint16_t rfChannel{58};
    static inline bool crsfOverride{false};
};

struct BtCallback {
    static inline void buttonPress(const uint8_t b) {
        if (b == 0) {
            --Data::rfChannel;
        }
        else if (b == 1) {
            ++Data::rfChannel;
        }
    }
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;

    using led = devs::led;
    using btPwr = devs::btPwr;
    using hfPwr = devs::hfPwr;

    using btUsart = devs::btUsart;
    using btPa = devs::btPa;

    using crsf = devs::crsf;
    using crsf_out = devs::crsf_out;
    using crsf_pa = devs::crsf_pa;
    using crsfTelemetry = devs::crsfTelemetry;

    using pca0 = devs::pca0;
    // using pca1 = devs::pca1;

    using adc1 = devs::adc1;
    using adc2 = devs::adc2;

    using i2c1 = devs::i2c1;
    using si = devs::si;

    using cppm = devs::cppm;

    static inline void init() {
        devs::init();
        led::set();
        btPwr::reset();
    }
    static inline void periodic() {
        trace::periodic();
        si::periodic();
        i2c1::periodic();
        pca0::periodic();

        btUsart::periodic();
        crsf::periodic();
    }

    enum class State : uint8_t {Undefined, Init, Run, Reset, Test, RfOn, RfOff,
                                Set};

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> changeTicks{2000ms};
    static inline constexpr External::Tick<systemTimer> monitorTicks{200ms};

    static inline uint8_t txChannel = 56; // C8
    // static inline uint8_t txChannel = 58;
    // static inline uint8_t txChannel = 64; // Scan RX
    // static inline uint8_t txChannel = 65;

    static inline void ratePeriodic() {
        btPa::ratePeriodic();

        crsf_pa::copyChangedChannels(Data::crsfChannels);
        crsf_out::ratePeriodic();
        crsfTelemetry::ratePeriodic();

        const auto oldState = mState;

        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Reset;
            });
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                si::setOutput(0);
                mState = State::Run;
            });
            break;
        case State::Set:
            if (si::setChannel(txChannel)) {
                mState = State::Run;
            }
            break;
        case State::Test:
            (++mChangeTick).on(changeTicks, []{
                cppm::setNormalized(4, mToggle ? 100 : 900);
                mToggle = !mToggle;
            });
            break;
        case State::Run:
            pca0::ratePeriodic();
            updateAnalogs();
            devs::CrsfCallback::whenParameterChanged([](const uint8_t p){
                IO::outl<trace>("crsf p: ", p);
                const auto pv = devs::CrsfCallback::parameter(p);
                if (p == 2) { // HF
                    if (pv.mValue > 0) {
                        Data::rfOn = true;
                        hfPwr::set();
                        mState = State::Set;
                    }
                    else {
                        Data::rfOn = false;
                        hfPwr::reset();
                    }
                }
                if (p == 3) { // Override
                    if (pv.mValue > 0) {
                        Data::crsfOverride = true;
                    }
                    else {
                        Data::crsfOverride = false;
                    }
                }
            });
            btPa::whenTargetChanged([](btPa::Target t, uint8_t n){
                if (t == btPa::Target::Toggle) {
                    if (n == 0) {
                        IO::outl<btUsart>("$v00 ");
                        Data::rfOn = !Data::rfOn;
                        if (Data::rfOn) {
                            hfPwr::set();
                            mState = State::Set;
                        }
                        else {
                            hfPwr::reset();
                        }
                    }
                }
                else if (t == btPa::Target::Switch) {
                    IO::outl<trace>("s: ", n, " ", btPa::switchValues[n]);
                    Data::switches[n - 1] = btPa::switchValues[n];
                }
            });
            for(uint8_t i = 0; i < 8; ++i) {
                if (const uint8_t v = pca0::switchValue(i); v != s0[i]) {
                    Data::switches[i] = v;
                    s0[i] = v;
                    IO::outl<trace>("x: ", v);
                }
            }
            (++mMonitorTick).on(monitorTicks, []{
                sendToMonitor();
                pca0::startRead();
            });
            (++mDebugTick).on(debugTicks, []{
                // IO::outl<trace>("a0: ", adc1::mData[0], " a1: ", adc1::mData[1], " a2: ", adc1::mData[2], " a3: ", adc1::mData[3], " a4: ", adc1::mData[4], " a5: ", adc1::mData[5]);
                // IO::outl<trace>("t: ", adc1::mData[6], " v: ", adc2::mData[0]);
                // IO::outl<btUsart>("$g00 ", adc2::mData[0]);
                // IO::outl<trace>("e ", i2c1::mErrors, " s ", (uint8_t)i2c1::mState);
                // IO::outl<trace>("pca0 ", (uint8_t)pca0::mValuePU, " ", (uint8_t)pca0::mValuePD);
                // IO::outl<trace>("pca0 s0", pca0::switchValue(0));
                IO::outl<trace>("crsf ", Data::crsfChannels[0]);
            });
            break;
        case State::Reset:
            mState = State::Init;
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                break;
            case State::Set:
                break;
            case State::Run:
                adc1::start();
                adc2::start();
                // hfPwr::set();
                // led::reset();
                break;
            case State::Reset:
                break;
            }
        }
    }
    static inline void sync() {
        updateMulti();
        cppm::setNormalized(4, multi[multiIndex]);
        cppm::setNormalized(7, multi[multiIndex]);
        if (++multiIndex == multi.size()) multiIndex = 0;
    }
    private:
    static inline uint8_t multiIndex{0};
    static inline std::array<uint16_t, 10> multi{1100, 1100};
    // static inline std::array<uint16_t, 10> multi{1600, 1600};

    static inline void sendToMonitor() {
        for(uint8_t i = 0; i < 4; ++i) {
            IO::outl<btUsart>("$p", i + 1, " ", Data::channels[i]);
        }
        IO::outl<btUsart>("$ch ", Data::rfChannel);
        IO::outl<btUsart>("$t00 ", Data::rfOn ? 1 : 0);
    }

    static inline void updateAnalogs() {
        for(uint8_t i = 0; i < 4; ++i) {
            if (Data::crsfOverride) {
                const uint16_t v = (1000.0f * (Data::crsfChannels[i] - 172.0f)) / 1640.0f;
                Data::channels[i] = v;
                cppm::setNormalized(i, v);
            }
            else {
                const uint16_t v = 999.9f * adc1::mData[i] / 4095.0f;
                Data::channels[i] = v;
                cppm::setNormalized(i, v);
            }
        }
    }
    static inline void updateMulti() {
        for(uint8_t i = 2; i < multi.size(); ++i) {
            const uint8_t k = i - 2;

            if (Data::switches[k] == 1) {
                multi[i] = 500;
            }
            else if (Data::switches[k] == 0) {
                multi[i] = 100;
                // multi[i] = 0;
            }
            else if (Data::switches[k] == 2) {
                multi[i] = 900;
                // multi[i] = 1000;
            }
        }
    }
    static inline std::array<uint8_t, 8> s0;
    static inline bool mToggle{false};
    static inline State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline External::Tick<systemTimer> mChangeTick;
    static inline External::Tick<systemTimer> mMonitorTick;
};

using devs = Devices<Var01, Config, Mcu::Stm::Stm32G431>;
using gfsm = GFSM<devs>;

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
void __assert_func (const char *, int, const char *, const char *){
    while(true) {
    }
}

extern "C" {

void TIM3_IRQHandler() {
    const uint32_t sr = TIM3->SR;
    if (sr & TIM_SR_CC2IF) {
         TIM3->SR = ~TIM_SR_CC2IF;
        if (!(sr & TIM_SR_UIF)) {
            gfsm::sync();
        }
    }
    if (sr & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;
        devs::pulse::startRollon();
    }
    if (sr & TIM_SR_CC1IF) {
        TIM3->SR = ~TIM_SR_CC1IF;
        devs::pulse::startRolloff();
    }
}



}
