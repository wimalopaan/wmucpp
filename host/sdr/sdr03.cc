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

#include "mcu/algorithm.h"
#include "mcu/dsp.h"
#include "mcu/fsk.h"
#include "mcu/output.h"

struct Config {
    Config() = delete;
//    static inline constexpr uint8_t down = 1;
//    static inline constexpr float fs = 48000.0 / down;
    static inline constexpr uint8_t down = 16;
    static inline constexpr float fs = 781250.0 / down; // 48828
    
    static inline constexpr float fb = 2048.0f;
    static inline constexpr float bitTicks = fs / fb;
    static inline constexpr uint16_t firLength = bitTicks + 0.5;
    static inline constexpr float halfBitTicks = fs / (2.0f * fb);
    static inline constexpr float syncTicks = bitTicks * 10.0f - halfBitTicks;
    static inline constexpr float zf = 5'000.0f;
    static inline constexpr float ft = 7'000'000.0f;
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

using demod = Dsp::FSK::Demodulation<Config>;

int main(const int argc, const char* const* const argv) {
    std::vector<std::string> args{argv, argv + argc};    
    std::cout << "demod_fsk_01\n";
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
        if (std::getline(lineStream, token, ',')) {
            iq.q = std::stof(token);
        }
        adcValues.push_back(iq);
    }   
    
    normalize(adcValues);
    
    std::vector<Dsp::IQ_Bit> bits;
    
    uint8_t c = 0;
    for(const auto& v : adcValues) {
        if (c == 0) {        
            Stats::process(v);
            const auto r = demod::process(v);
            bits.push_back(r);
        }
        c += 1;
        if (c == Config::down) c = 0;
    }
    
    std::cout << "nsamples: " << Stats::numberOfSamples << '\n';
    std::cout << "min: " << Stats::min << '\n';
    std::cout << "max: " << Stats::max << '\n';
    
//    std::cout << "g low: " << '\n';
    
//    for(const auto& c : Bandpass::fir_bp_low_i.coeff) {
//        std::cout << c << '\n';
//    }

//    std::cout << "g high: " << '\n';
    
//    for(const auto& c : Bandpass::fir_bp_high_i.coeff) {
//        std::cout << c << '\n';
//    }
    
    
    std::cout << "bit ticks: " << Config::bitTicks << '\n';
    std::cout << "lobe low: " << Config::lobeLow << '\n';
    std::cout << "lobe high: " << Config::lobeHigh << '\n';
    std::cout << "fir length: " << Config::firLength << '\n';
    
}
