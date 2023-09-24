#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>

#include "mcu/cppm.h"

struct Config {
    Config() = delete;
//    static inline constexpr uint8_t down = 1;
//    static inline constexpr float fs = 48000.0 / down;
    static inline constexpr uint8_t down = 8;
    static inline constexpr float fs = 781250.0 / down; // 96xxx Hz
    
    static inline constexpr float zf = 5'000.0f + 4'670.0f; // Abweichung
    static inline constexpr float ft = 40'000'000.0f;
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
    static inline constexpr float tMinPulse = 0.0009; // 0.9 ms
    static inline constexpr size_t nMinPulse = std::round(tMinPulse * fs);

    static inline constexpr uint16_t maxImpulseDelta = 4;
    
    static inline constexpr float fImpulseMean = 0.333;
    
    static inline constexpr float fb = 2048.0f;
    static inline constexpr float bitTicks = fs / fb;
    static inline constexpr uint16_t firLength = bitTicks + 0.5;
    static inline constexpr float halfBitTicks = fs / (2.0f * fb);
    static inline constexpr float syncTicks = bitTicks * 10.0f - halfBitTicks;
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

constexpr void normalize(std::vector<Dsp::IQ>& values) {
    float min{std::numeric_limits<float>::max()};
    float max{std::numeric_limits<float>::min()};
    for(const auto v : values) {
        min = std::min(min, v.i);
        min = std::min(min, v.q);
        max = std::max(max, v.i);
        max = std::max(max, v.q);
    }
    float s = float{0xff00} / 0x8000; 
    for(auto& v : values) {
        v.i = s * (v.i - min) / (max - min);
        v.q = s * (v.q - min) / (max - min);
    }
}

struct Stats {
    Stats() = delete;
    static inline constexpr void process(const Dsp::IQ v) {
        ++numberOfSamples;
        min = std::min(min, v.i);
        min = std::min(min, v.q);
        max = std::max(max, v.i);
        max = std::max(max, v.q);
    }
    static inline float min{std::numeric_limits<float>::max()};
    static inline float max{std::numeric_limits<float>::min()};
    static inline uint32_t numberOfSamples{};
};

struct Export {
    static void input(const float v) {
        in << v << '\n';
    }
    static void max(const float v) {
        ma << v << '\n';
    }
    static void normMax(const float v) {
        nm << v << '\n';
    }
    static void highpass(const float v) {
        hp << v << '\n';
    }
    static void chirp(const float v) {
        ch << v << '\n';
    }
    static void chirpS(const float v) {
        chs << v << '\n';
    }
    static void peak(const float v) {
        pk << v << '\n';
    }
    static void peakMax(const float v) {
        pm << v << '\n';
    }
    static void peakPos(const Dsp::Cppm::Peak v) {
        pp << v.pos << ',' << v.value << '\n';
    }
    static inline size_t globalCounter{0};
private:    
    static inline std::ofstream in{"cppm_in.csv"};
    static inline std::ofstream ma{"cppm_ma.csv"};
    static inline std::ofstream nm{"cppm_nm.csv"};
    static inline std::ofstream hp{"cppm_hp.csv"};
    static inline std::ofstream ch{"cppm_chirp.csv"};
    static inline std::ofstream chs{"cppm_chS.csv"};
    static inline std::ofstream pk{"cppm_pk.csv"};
    static inline std::ofstream pm{"cppm_pm.csv"};
    static inline std::ofstream pp{"cppm_pp.csv"};
    
};

struct CallBack {
    static void process(const std::array<uint16_t, 8>& intervalls) {
        for(size_t k{}; const auto& iv : intervalls) {
            const uint16_t l = limiter[k].process(iv);
            peakEstimators[k].process(l);
            ++k;
        }
        
        std::cout << "Intervalls: ";
        for(size_t k{}; const auto& i : intervalls) {
            std::cout << i << "(" << limiter[k].value() << ", " << peakEstimators[k].value() << ")" << " ";
            ++k;
        }
        std::cout << '\n';
    }
    
    inline static std::array<Dsp::ExpMean<Config>, 8> peakEstimators{[]{
        std::array<Dsp::ExpMean<Config>, 8> data;
        for(auto& d: data) {
            d = Dsp::ExpMean<Config>{Config::fImpulseMean};
            d.set((Config::nMaxPulse + Config::nMinPulse) / 2);
        }
        return data;                
    }()};
    
    inline static std::array<Dsp::StepLimiter<Config>, 8> limiter{};
};

using demod = Dsp::Cppm::Demodulation<Config, CallBack, Export>;

int main(const int argc, const char* const* const argv) {
    std::vector<std::string> args{argv, argv + argc};    
    std::cout << "ppm01\n";
    if (args.size() <= 1) {
        std::cerr << "filename?\n";
        return -1;
    }
    std::cout << "file: " << args[1] << "\n";
    
    std::vector<Dsp::IQ> adcValues;
    
    std::ifstream dataFile{args[1]};
    
    if (!dataFile.is_open()) {
        std::cerr << "open?\n";
        return -1;
    }
    std::string line;
    while(std::getline(dataFile, line)) {
        std::stringstream lineStream{line};
        std::string token;
        Dsp::IQ iq;
        if (std::getline(lineStream, token, ',')) {
            if (!std::isdigit(token[0])) {
                continue;
            }            
        }
        if (std::getline(lineStream, token, ',')) {
            iq.i = std::stof(token);
        }
//        if (std::getline(lineStream, token, ',')) {
//            iq.q = std::stof(token);
//        }
        adcValues.push_back(iq);
    }   
    
    normalize(adcValues);
    
//    std::vector<Dsp::IQ_Bit> bits;
    
//    std::ofstream ch{"chirp.csv"};
//    for(const auto v : demod::chirpCorrelator.chirp) {
//        ch << v << '\n';
//    }
    
    uint8_t c = 0;
    for(const auto& v : adcValues) {
        if (c == 0) {        
            Stats::process(v);
            demod::process(v);
        }
        c += 1;
        if (c == Config::down) c = 0;
    }
    
    std::cout << "nsamples: " << Stats::numberOfSamples << '\n';
    std::cout << "min: " << Stats::min << '\n';
    std::cout << "max: " << Stats::max << '\n';
    
    std::cout << "nMaxPulse: " << Config::nMaxPulse << '\n';
    std::cout << "nMinPulse: " << Config::nMinPulse << '\n';
    std::cout << "nPulse: " << Config::nPulse << '\n';
}
