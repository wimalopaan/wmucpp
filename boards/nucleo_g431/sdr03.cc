#define USE_MCU_STM_V2

#include "devices.h"

#include "fsk.h"

#include <chrono>
#include <cassert>

using namespace std::literals::chrono_literals;

struct Config1 {
//    Config() = delete;
    static inline constexpr uint8_t down = 1;
    static inline constexpr float fs = 48000.0 / down;
        static inline constexpr float fb = 2048.0f;
//    static inline constexpr float fb = 2 * 2048.0f;
    static inline constexpr float bitTicks = fs / fb;
//    static inline constexpr uint16_t firLength = 23;
    static inline constexpr uint16_t firLength = bitTicks + 0.5;
    static inline constexpr float halfBitTicks = fs / (2.0f * fb);
    static inline constexpr float syncTicks = bitTicks * 10.0f - halfBitTicks;
    static inline constexpr float zf = 5'000.0f;
    static inline constexpr float fSymbolLow = 0.0f;
    static inline constexpr float fSymbolHigh = 5'000.0f;
    static inline constexpr float fLow  = zf + fSymbolLow;
    static inline constexpr float fHigh = zf + fSymbolHigh;
    static inline constexpr float bandwidth = 0.1f;
    static inline constexpr float halfBandwidthLow = (bandwidth * fLow) / 2.0f;
    static inline constexpr float halfBandwidthHigh = (bandwidth * fHigh) / 2.0f;
    static inline constexpr float bpLow_fl = fLow - halfBandwidthLow;
    static inline constexpr float bpLow_fh = fLow + halfBandwidthLow;
    static inline constexpr float bpHigh_fl = fHigh - halfBandwidthLow;
    static inline constexpr float bpHigh_fh = fHigh + halfBandwidthLow;
    static inline constexpr uint16_t lobeLow = 0.5 * fs / fLow + 0.5;
    static inline constexpr uint16_t lobeHigh = 0.5 * fs / fHigh + 0.5 + 1;

    static inline constexpr uint16_t bytesInFrame = 10;
//    static inline constexpr uint16_t bitsInFrame = bytesInFrame * (8 + 1); // ohne erstes 0-Bit
};




template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;
    
    using pwm1 = devs::pwm1;
    
    struct Callback {
        template<auto N>
        static inline void process(const std::array<uint8_t, N>& data) {
            const uint16_t ch1 = data[1] + 256;
            pwm1::duty(ch1);
        }
    };
    
//    using demod = Demodulation<trace>;
    using demod = Dsp::FSK::Demodulation<Config1, void, Callback, void, typename devs::dac1>;
    
//    using adc1 = devs::adc1;

    using adc_i = devs::adc2;
    using dac_ref = devs::dac3;
    
    using i2c2 = devs::i2c2;
    using si = devs::si;
 
    using serial1 = devs::serial1;
    
    enum class State : uint8_t {Undefined, Init, Setup, Conv, Wait};
    
//    static inline constexpr External::Tick<systemTimer> mInitTicks{500ms};
    
    static inline void init() {
        devs::init();
        devs::led::set();
    }
    static inline void periodic() {
        trace::periodic();
        si::periodic();
        i2c2::periodic();
        serial1::periodic();
        
        if (!adc_i::busy()) {
            const auto aValue = adc_i::value();
            devs::pina11::reset();
//            devs::dac1::set(aValue);
            devs::pinb5::set();
            iq_bit = demod::process(Dsp::IQ(aValue, aValue));
            if (iq_bit.i) {
                devs::pinb0::set();
            }
            else {
                devs::pinb0::reset();
            }
            devs::pinb5::reset();
        }
        
//        const auto oldState = mState;
//        switch(mState) {
//        case State::Undefined:
//        break;
//        case State::Init:
//        break;
//        case State::Setup:
//        break;
//        case State::Conv:
//        break;
//        case State::Wait:
//            if (!adc_i::busy()) {
//                aValue = adc_i::value();
//                devs::pina11::reset();
//                devs::dac1::set(aValue);
//                devs::pinb5::set();
//                iq_bit = demod::process({aValue, aValue});
//                if (iq_bit.i) {
//                    devs::pinb0::set();
//                }
//                else {
//                    devs::pinb0::reset();
//                }
//                devs::pinb5::reset();
//            }
//        break;
//        }
//        if (oldState != mState) {
//            switch(mState) {
//            case State::Undefined:
//            break;
//            case State::Init:
//            break;
//            case State::Setup:
//            break;
//            case State::Conv:
//            break;
//            case State::Wait:
//            break;
//            }
//        }
    }
    static inline void ratePeriodic() {
        
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
        break;
        case State::Init:
            if (adc_i::ready()) {
                dac_ref::set(0x0fff / 2);
                si::setOutput(1);
//                si::off();    
                mState = State::Setup;
            }
        break;
        case State::Setup:
            if (si::setFrequency(Units::hertz{6'995'000 * 4})) { // zf = 5 KHz
                mState = State::Conv;
            }
        break;
        case State::Conv:
            devs::pina11::set();
            adc_i::start();
            mState = State::Wait;
        break;
        case State::Wait:
        break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
            break;
            case State::Init:
            break;
            case State::Setup:
            break;
            case State::Conv:
            break;
            case State::Wait:
            break;
            }
        }

        if (++c == 48000) {
            c = 0;
//            IO::outl<trace>("State: ", (uint8_t)mState, " adc: ", aValue, " ac: ", ac);
            IO::outl<trace>("max i: ", Dsp::Stats::max(), " e: ", demod::proto::ByteStuff::errors(), " p: ", demod::proto::ByteStuff::packages());
            IO::outl<serial1>("max i: ", Dsp::Stats::max(), " e: ", demod::proto::ByteStuff::errors(), " p: ", demod::proto::ByteStuff::packages());
        }
    }
private:
    static inline Dsp::IQ_Bit iq_bit;
    
    static inline uint32_t g{0};
    
    static inline uint32_t c;
    static inline uint16_t a;
    
    static inline State mState{State::Undefined};
//    static inline uint16_t aValue{42};
};

//extern "C" void SysTick_Handler()  {                               
//    asm("# systickA");
//    using devs = Devices<SDR03, Mcu::Stm::Stm32G431>;
//    using gfsm = GFSM<devs>;
//    devs::systemTimer::periodic([]{
//        gfsm::ratePeriodic();
//    });
//}

int main() {
    using devs = Devices<SDR03, Mcu::Stm::Stm32G431>;
    using gfsm = GFSM<devs>;
    gfsm::init();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

