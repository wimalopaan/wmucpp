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

#include "Iir.h"

#include "mcu/dsp.h"

struct Config {
    Config() = delete;
//    static inline constexpr uint8_t down = 1;
//    static inline constexpr float fs = 48000.0 / down;
    static inline constexpr uint8_t down = 8;
    static inline constexpr float fs = 781250.0 / down; // 96xxx Hz

};

constexpr void normalize(std::vector<float>& values) {
    float min{std::numeric_limits<float>::max()};
    float max{std::numeric_limits<float>::min()};
    for(const auto v : values) {
        min = std::min(min, v);
        max = std::max(max, v);
    }
    float s = float{0xff00} / 0x8000; 
    for(auto& v : values) {
        v = s * (v - min) / (max - min);
    }
}

struct Stats {
    Stats() = delete;
    static inline constexpr void process(const float v) {
        ++numberOfSamples;
        min = std::min(min, v);
        max = std::max(max, v);
    }
    static inline float min{std::numeric_limits<float>::max()};
    static inline float max{std::numeric_limits<float>::min()};
    static inline uint32_t numberOfSamples{};
};

struct Export {
    static void input(const float v) {
        in << v << '\n';
    }
    static void f1(const float v) {
        filter << v << '\n';
    }
    static void mean(const float v) {
        m0 << v << '\n';
    }
    static void max(const float v) {
        ma << v << '\n';
    }
    static void min(const float v) {
        mi << v << '\n';
    }
    static void res(const bool v) {
        r << v << '\n';
    }
    static inline size_t globalCounter{0};
private:    
    static inline std::ofstream in{"in.csv"};
    static inline std::ofstream m0{"mean.csv"};
    static inline std::ofstream ma{"max.csv"};
    static inline std::ofstream mi{"min.csv"};
    static inline std::ofstream filter{"f1.csv"};
    static inline std::ofstream r{"res.csv"};
};

template<typename C>
struct Estimator {
    static inline Dsp::HystereseThreshold<void> th{0.0001};

    // static inline auto expMean = []{
    //     Dsp::ExpMean<void> filter{0.0001};
    //     return filter;
    // }();

    static inline auto iirFilter = []{
        Dsp::Butterworth::LowPass<6> filter;
        filter.setup(Config::fs, 1000);
        // Iir::Butterworth::LowPass<6> filter;
        // // filter.setup(Config::fs, 200, 5);
        // filter.setup(Config::fs, 200);
        return filter;
    }();

    static inline void process(const float v) {

        // median-filter

        const auto x = iirFilter.process(v);
        const bool t = th.process(x);

        Export::input(v);
        Export::f1(th.v0);
        Export::mean(th.mMean.value());
        Export::max(th.mMax.value());
        Export::min(th.mMin.value());
        Export::res(th.state);
    }
};

using estimator = Estimator<Config>;

int main(const int argc, const char* const* const argv) {
    std::vector<std::string> args{argv, argv + argc};
    std::cout << "test00\n";
    if (args.size() <= 1) {
        std::cerr << "filename?\n";
        return -1;
    }
    std::cout << "file: " << args[1] << "\n";
    
    std::vector<float> adcValues;
    
    std::ifstream dataFile{args[1]};
    
    if (!dataFile.is_open()) {
        std::cerr << "open?\n";
        return -1;
    }
    std::string line;
    while(std::getline(dataFile, line)) {
        std::stringstream lineStream{line};
        std::string token;
        float v;
        if (std::getline(lineStream, token, ',')) {
            if (!std::isdigit(token[0])) {
                continue;
            }            
        }
        // if (std::getline(lineStream, token, ',')) {
        //     if (!std::isdigit(token[0])) {
        //         continue;
        //     }
        // }
        if (std::getline(lineStream, token, ',')) {
            v = std::stof(token);
        }
        // std::cout << v << '\n';
        adcValues.push_back(v);
    }   
    
    normalize(adcValues);
    
    uint8_t c = 0;
    for(const auto& v : adcValues) {
        if (c == 0) {
            Stats::process(v);
            estimator::process(v);
        }
        c += 1;
        if (c == Config::down) c = 0;
    }
    
    std::cout << "nsamples: " << Stats::numberOfSamples << '\n';
    std::cout << "min: " << Stats::min << '\n';
    std::cout << "max: " << Stats::max << '\n';
}
