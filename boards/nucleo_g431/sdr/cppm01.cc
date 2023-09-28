#define USE_MCU_STM_V2

#include "devices.h"
#include "cppm.h"

#include <chrono>
#include <cassert>

// Für das Freq.-Tuning:
// - max Ablage +/- 5KHz
// Algorithmus:
// - Intervall [-5KHz, 5KHz] in 11 Stützstellen , Abstand 1KHz -> Faltung mit 2ms ZF-Sinus / Max / Mean
// - größten Wert f0
// - um f0 mit 500Hz Schritten 11 Stützstellen
// - größten Wert f1
// - um f1 mit 100Hz 
// - ... 50Hz
// Danach ggf. noch die Pulse-Frequenz fp optimieren:
// - um 2500Hz 11 Stützstellen mit 100Hz Abstand -> höchste Korrelation


using namespace std::literals::chrono_literals;

struct Config {
    Config() = delete;
    static inline constexpr float fs = 96000.0;
    
    static inline constexpr float zf = 5'000.0f + 4'670.0f; // Abweichung -> Tuning (s.o.)
    static inline constexpr float ft = 40'915'000.0f;
    static inline constexpr float fp = 2'400.0f; // Pulse-Frequenz
    
    static inline constexpr float tPulse = 0.0005; // 0.5 ms
    static inline constexpr size_t nPulse = std::round(fs * tPulse);    
    static inline constexpr float fMaxFilter = 0.005; // Max Extimator
    static inline constexpr float alphaHighPass = 0.001; // Highpass
    static inline constexpr float tauPulse = 6; // Chirp
    static inline constexpr float tMaxDistance = 0.0022; // 2.2 ms
    static inline constexpr size_t nMaxDistance = std::round(tMaxDistance * fs);
    static inline constexpr float fMaxPeakFind = 0.0005;
    static inline constexpr float fThreshPeakFind = 0.2;

    static inline constexpr float tMaxPulse = tMaxDistance;
    static inline constexpr size_t nMaxPulse = std::round(tMaxPulse * fs);
    static inline constexpr float tMinPulse = 0.0009;
    static inline constexpr size_t nMinPulse = std::round(tMinPulse * fs);

    static inline constexpr uint16_t maxImpulseDelta = 4;
    
    static inline constexpr float fImpulseMean = 0.333;
    
    static inline constexpr float fb = 2048.0f;
    static inline constexpr float bitTicks = fs / fb;
    static inline constexpr uint16_t firLength = bitTicks + 0.5;
    static inline constexpr float halfBitTicks = fs / (2.0f * fb);
    static inline constexpr float syncTicks = bitTicks * 10.0f - halfBitTicks;
//    static inline constexpr float zf = 5'000.0f;
//    static inline constexpr float ft = 7'000'000.0f;
//    static inline constexpr float ft = 40'915'000.0f;
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
    
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;
    
    using pwm1 = devs::pwm1;
    
    struct Callback {
        static void process(const std::array<uint16_t, 8>& intervalls) {
            for(size_t k{}; const auto& iv : intervalls) {
                const uint16_t l = limiter[k].process(iv);
                const uint16_t channel = 256 + 256.0f * (peakEstimators[k].process(l) - Config::nMinPulse) / (Config::nMaxPulse - Config::nMinPulse);
                if (k == 0) {
                    pwm1::duty(channel);
                }
                ++k;
            }
        }
    private:        
        inline static std::array<Dsp::ExpMean<Config>, 8> peakEstimators{[]{
            std::array<Dsp::ExpMean<Config>, 8> data;
            for(auto& d: data) {
                d = Dsp::ExpMean<Config>{Config::fImpulseMean};
                d.set((Config::nMaxPulse + Config::nMinPulse) / 2);
            }
            return data;                
        }()};

        inline static std::array<Dsp::StepLimiter<Config>, 8> limiter{};
        
        
//        template<auto N>
//        static inline void process(const std::array<uint8_t, N>& data) {
//            const uint16_t ch1 = data[1] + 256;
//            pwm1::duty(ch1);
//        }
    };
    
    using demod = Dsp::Cppm::Demodulation<Config, Callback>;
    
    using adc_q = devs::adc1;
    using adc_i = devs::adc2;
    
    using dac_ref = devs::dac3;
    
    using i2c2 = devs::i2c2;
    using si = devs::si;
 
    using serial1 = devs::serial1;
    
    enum class State : uint8_t {Undefined, Init, Setup, StartConv, Run};
    
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
            devs::pinb5::set(); 
            demod::process(Dsp::IQ(aValue, aValue));
//            devs::pinb0::set(iq_bit.i);
            devs::pinb5::reset();
        }
    }
    static inline void ratePeriodic() {
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
        break;
        case State::Init:
            if (adc_i::ready()) {
                dac_ref::set(0x0fff / 2);
                mState = State::Setup;
            }
        break;
        case State::Setup:
            if (si::setFrequency(Units::hertz{uint32_t(Config::ft - Config::zf)})) { 
                mState = State::StartConv;
            }
        break;
        case State::StartConv:
            devs::pina11::set();
            adc_i::start();
            mState = State::Run;
        break;
        case State::Run:
        break;
        }

        if (++c == 48000) {
//            demod::debugSwitch();
            c = 0;
//            IO::outl<trace>("State: ", (uint8_t)mState, " adc: ", aValue, " ac: ", ac);
//            IO::outl<trace>("max i: ", Dsp::Stats::max(), " e: ", demod::proto::ByteStuff::errors(), " p: ", demod::proto::ByteStuff::packages());
//            IO::outl<serial1>("max i: ", Dsp::Stats::max(), " e: ", demod::proto::ByteStuff::errors(), " p: ", demod::proto::ByteStuff::packages());
        }
    }
private:
    static inline uint32_t g{0};
    static inline uint32_t c{};
    static inline State mState{State::Undefined};
};

int main() {
    using devs = Devices<CPPM01, Config, Mcu::Stm::Stm32G431>;
    using gfsm = GFSM<devs>;
    gfsm::init();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

