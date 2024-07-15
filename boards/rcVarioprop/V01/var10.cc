#define USE_MCU_STM_V3
#define NDEBUG

#include "devices.h"

#include <chrono>
#include <cassert>

#include "buzzer.h"
#include "switches.h"
#include "analog_gauge.h"

using namespace std::literals::chrono_literals;

struct McuConfig {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;
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

template<typename Trace>
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
                    if (index < Data::switches.size()) {
                        Data::switches[index] = (uint8_t)payload[5];
                    }
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
    using data = Data;
    using btcallback = BtCallback<McuConfig::trace>;
    using crsfcallback = CrsfCallback<McuConfig::trace>;
};

template<typename Out, typename SystemTimer>
struct CrsfTelemetry {
    using out = Out;
    static inline constexpr External::Tick<SystemTimer> telemTicks{500ms};

    enum class State : uint8_t {Batt, Temp};

    static inline void ratePeriodic() {
        (++mTelemTick).on(telemTicks, []{
            switch(mState) {
            case State::Batt:
                out::data(RC::Protokoll::Crsf::Type::Battery, mBatt);
                mState = State::Temp;
                break;
            case State::Temp:
                out::data(RC::Protokoll::Crsf::Type::Temp1, mTemp);
                mState = State::Batt;
                break;
            }
        });
    }
    private:
    static inline State mState{State::Batt};
    static inline std::array<std::byte, 8> mBatt; //volts amps mAh percent
    static inline std::array<std::byte, 2> mTemp;
    static inline External::Tick<SystemTimer> mTelemTick;
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
    using crsfTelemetry = CrsfTelemetry<crsf_out, systemTimer>;
    using crsfcallback = devs::crsfcallback;

    using pca0 = devs::pca0;
    using pca1 = devs::pca1;

    using adc1 = devs::adc1;
    using adc2 = devs::adc2;

    using i2c1 = devs::i2c1;
    using si = devs::si;

    using cppm = devs::cppm;

    // gauge.h
    using ds = devs::ds;
    using buzzer = External::Buzzer<typename devs::pwm, 0, typename devs::clock>;

    static inline void i2c1ScanCallback(const I2C::Address adr) {
        IO::outl<trace>("Scan adr: ", adr.value);
        i2c1Adresses.push_back(adr);
    }

    static inline void init() {
        devs::init();
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

    enum class State : uint8_t {Undefined, Init,
                                I2CScan1,
                                Run, Reset, Test, RfOn, RfOff,
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
        i2c1::ratePeriodic();
        crsf_pa::copyChangedChannels(Data::crsfChannels);
        crsf_out::ratePeriodic();
        crsfTelemetry::ratePeriodic();
        buzzer::ratePeriodic();

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
        case State::RfOff:
        case State::RfOn:
        case State::Run:
            if (switchesBoard0Present) {
                pca0::ratePeriodic();
            }
            updateAnalogs();
            crsfcallback::whenParameterChanged([](const uint8_t p){
                IO::outl<trace>("crsf p: ", p);
                const auto pv = crsfcallback::parameter(p);
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

            // switches.h
            if (switchesBoard0Present) {
                for(uint8_t i = 0; i < 8; ++i) {
                    if (const uint8_t v = pca0::switchValue(i); v != s0[i]) {
                        Data::switches[i] = v;
                        s0[i] = v;
                        IO::outl<trace>("x: ", v);
                    }
                }
            }
            if (switchesBoard1Present) {

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
            mState = State::I2CScan1;
            break;
        case State::I2CScan1:
            if (i2c1::isIdle()) {
                checkForSwitchesBoards();
                mState = State::Init;
            }
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                IO::outl<trace>("Undefined");
                break;
            case State::Init:
                IO::outl<trace>("Init");
                led::set();
                break;
            case State::Set:
                IO::outl<trace>("Set");
                break;
            case State::Test:
                IO::outl<trace>("Test");
                break;
            case State::RfOn:
                IO::outl<trace>("RfOn");
                buzzer::off();
                buzzer::oneShotLong();
                break;
            case State::RfOff:
                IO::outl<trace>("RfOff");
                buzzer::shortPeriodic();
                break;
            case State::Run:
                IO::outl<trace>("Run");
                adc1::start();
                adc2::start();
                break;
            case State::Reset:
                IO::outl<trace>("Reset");
                break;
            case State::I2CScan1:
                IO::outl<trace>("I2CScan1");
                if (i2c1::scan(i2c1ScanCallback)) {
                    IO::outl<trace>("start");
                }
                else {
                    IO::outl<trace>("failed");
                }
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

    static inline void checkForSwitchesBoards() {
        for(const auto& a : i2c1Adresses) {
            if (a == devs::pcaAdr0) {
                switchesBoard0Present = true;
            }
            if (a == devs::pcaAdr1) {
                switchesBoard1Present = true;
            }
        }
    }

    static inline bool switchesBoard0Present{};
    static inline bool switchesBoard1Present{};
    static inline etl::FixedVector<I2C::Address, 10> i2c1Adresses;

    static inline std::array<uint8_t, 8> s0;
    static inline bool mToggle{false};
    static inline State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline External::Tick<systemTimer> mChangeTick;
    static inline External::Tick<systemTimer> mMonitorTick;
};

using devs = Devices<Var02, Config, Mcu::Stm::Stm32G431>;
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
