#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <array>

#include "filter.h"

enum class State {High, Low};

struct IQ {
    float i{};
    float q{};
};

struct IQ_Sign {
    State i{};
    State q{};
};
struct IQ_F {
    uint16_t i{};
    uint16_t q{};
};
struct IQ_Bit {
    bool i{};
    bool q{};
};

struct IQ_Pass {
    IQ low{};
    IQ high{};
};

std::ostream& operator<<(std::ostream& s, const IQ_Pass v) {
    return s << v.low.i << ',' << v.low.q << ',' << v.high.i << ',' << v.high.q << '\n';
}

struct BiQuadCoeff {
    std::array<float, 2> a{};
    std::array<float, 3> b{};    
    float g{1.0};
};

//octave:21> [bp, ap] = butter(3, [2 * 4000/31250, 2 * 11000/31250], "pass");
//octave:25> [x, y] = tf2sos(bp, ap)
//x =

//1.0000e+00   2.0000e+00   9.9999e-01   1.0000e+00   8.8099e-01   5.6595e-01
//1.0000e+00  -2.0000e+00   9.9999e-01   1.0000e+00  -8.9096e-02   8.1864e-02
//1.0000e+00  -4.5765e-07  -1.0000e+00   1.0000e+00  -1.0636e+00   5.9949e-01

//y = 0.1287

constexpr std::array<BiQuadCoeff, 3> butter {
    BiQuadCoeff{{8.8099e-01,  5.6595e-01}, {1.0000e+00, 2.0000e+00,  9.9999e-01}, 0.1287},
    BiQuadCoeff{{-8.9096e-02, 8.1864e-02}, {1.0000e+00, -2.0000e+00, 9.9999e-01}},
    BiQuadCoeff{{-1.0636e+00, 5.9949e-01}, {1.0000e+00, -4.5765e-07, -1.0000e+00}}
};


//octave:24> [bp, ap] = ellip(3, 3, 60, [2 * 4000/31250, 2 * 11000/31250], "pass");
//octave:25> tf2sos(bp, ap)
//ans =

//7.1510e-02   1.3788e-01   7.1510e-02   1.0000e+00   1.0341e+00   8.4948e-01
//1.0000e+00  -1.9481e+00   1.0000e+00   1.0000e+00  -1.3095e-01   5.9005e-01
//1.0000e+00   1.6653e-15  -1.0000e+00   1.0000e+00  -1.2336e+00   8.6296e-01

constexpr std::array<BiQuadCoeff, 3> elliptic {
    BiQuadCoeff{{1.0341e+00, 8.4948e-01}, {7.1510e-02, 1.3788e-01, 7.1510e-02}},
    BiQuadCoeff{{-1.3095e-01, 5.9005e-01}, {1.0000e+00, -1.9481e+00, 1.0000e+00}},
    BiQuadCoeff{{-1.2336e+00, 8.6296e-01}, {1.0000e+00, 1.6653e-15, -1.0000e+00}}
}; 

struct BiQuad {
    BiQuad() {}
    explicit BiQuad(const BiQuadCoeff& c) : coeff{c} {}
    float process(const float v) {
        w[0] = v * coeff.g - (coeff.a[0] * w[1]) - (coeff.a[1] * w[2]);
        const float y = (coeff.b[0] * w[0]) + (coeff.b[1] * w[1]) + (coeff.b[2] * w[2]);
        w[2] = w[1];
        w[1] = w[0]; 
        return y;
    }
    BiQuadCoeff coeff;
    std::array<float, 3> w{};
};

template<uint8_t NSOS>
struct IIR {
    explicit IIR(const std::array<BiQuadCoeff, NSOS>& cc) {
        for(uint8_t i{0}; const auto& c : cc) {
            stages[i++] = BiQuad{c};
        }
    }
    float process(float v) {
        float r{0};
        for(BiQuad& s : stages) {
            r = s.process(v);
            v = r;
        }
        return r;
    }        
    std::array<BiQuad, NSOS> stages;
};

struct IIR2 {
    IIR2() {
        filter = create_che_band_pass_filter(4, 0.1, 31250, 4000, 11000);
    }
    ~IIR2() {
        free_che_band_pass(filter);
    }
    float process(const float v) {
        return che_band_pass(filter, v);
    }
    CHEBandPass* filter{};
};


constexpr std::array<float, 22> fir_bp_5k = {
    1.1781e-02, 1.6254e-02, 1.7846e-02, 3.8526e-03, -3.3222e-02, -8.0878e-02, -1.0768e-01, -8.3444e-02, -4.6129e-03, 9.6081e-02, 1.6610e-01,
    1.6610e-01, 9.6081e-02, -4.6129e-03, -8.3444e-02, -1.0768e-01, -8.0878e-02, -3.3222e-02, 3.8526e-03, 1.7846e-02, 1.6254e-02, 1.1781e-02  
};
constexpr std::array<float, 22> fir_bp_10k = {
    6.5069e-03, 1.3253e-02, -2.0038e-03, -3.8900e-02, -2.8620e-02, 6.0968e-02, 9.7768e-02, -2.9822e-02, -1.5842e-01, -6.0689e-02, 1.4666e-01,
    1.4666e-01, -6.0689e-02, -1.5842e-01, -2.9822e-02, 9.7768e-02, 6.0968e-02, -2.8620e-02, -3.8900e-02, -2.0038e-03, 1.3253e-02, 6.5069e-03   
};

template<uint8_t L>
struct FIR {
    explicit FIR(const std::array<float, L>& c) : coeff{c} {}
    
    float process(const float v) {
        buffer[in] = v;
        float result{0};
        for(uint8_t i{0}, k = in; i < L; ++i) {
            result += coeff[i] * buffer[k];
//            k = (k + 1) % L;
            if (k == 0) {
                k = L - 1;
            }
            else {
                --k;
            }
        }
        in = (in + 1) % L;
        return result;
    }
    uint8_t in{0};
    std::array<float, L> buffer{};    
    std::array<float, L> coeff{};
};

struct IQ2Pass {
    static inline IQ_Pass process(const IQ v) {
        IQ_Pass r{};
        r.low.i = fir_bp_low_i.process(v.i);        
        r.low.q = fir_bp_low_q.process(v.q);        
        r.high.i = fir_bp_high_i.process(v.i);        
        r.high.q = fir_bp_high_q.process(v.q);        
        file << r;
        return r;
    }
    static inline FIR<22> fir_bp_low_i{fir_bp_5k};
    static inline FIR<22> fir_bp_high_i{fir_bp_10k};
    static inline FIR<22> fir_bp_low_q{fir_bp_5k};
    static inline FIR<22> fir_bp_high_q{fir_bp_10k};
    static inline std::ofstream file{"filt.csv"};
};

struct Square {
    static inline IQ process(const IQ v) {
        return {v.i * v.i, v.q * v.q};
    }        
};

template<uint8_t L>
struct Max {
    float process(const float v) {
        buffer[in] = v;
        float max = v;
        for(const float& x : buffer) {
            if (x > max) max = x;
        }
        in = (in + 1) % L;
        return max;
    }            
    std::array<float, L> buffer;
    uint8_t in{0};
};

struct Thresh {
    static inline bool process(const float l, const float h) {
        return h > l;
    }            
};

struct Filter {
    static inline IQ process(const IQ v) {
        IQ r{iir_i.process(v.i), iir_q.process(v.q)};
        file << r.i << ',' << r.q << '\n';
        return r;
    }
//    static inline IIR2 iir_i;
//    static inline IIR2 iir_q;
    static inline IIR<3> iir_i{butter};
    static inline IIR<3> iir_q{butter};
    static inline std::ofstream file{"filt.csv"};
};

struct Comparator {
    explicit Comparator(const float high, const float low) :
        ic{high, low}, qc{high, low} {}

    struct SingleComparator {
        explicit SingleComparator(const float high, const float low) :
            mThreshHigh{high}, mThreshLow{low} {}
        State process(const float v) {
            switch(mState) {
            case State::Low:
                if (v > mThreshHigh) {
                    mState = State::High;
                }
            break;
            case State::High:
                if (v < mThreshLow) {
                    mState = State::Low;
                }
            break;
            }            
            return mState;
        }
        State mState{State::Low};
        float mThreshHigh{};
        float mThreshLow{};
    };
    
    IQ_Sign process(const IQ v) {
        IQ_Sign s;
        s.i = ic.process(v.i);
        s.q = qc.process(v.q);

        file << (uint16_t)s.i << ',' << (uint16_t)s.q << '\n';
        
        return s;
    }
    SingleComparator ic;
    SingleComparator qc;

    static inline std::ofstream file{"comp.csv"};
};

struct RunLength {
    struct SingleRunLength {
        enum class FState {Off, On};

        static inline uint16_t process(const State v) {
            switch(mState) {
            case FState::Off:
                if (v == State::High) {
                    mState = FState::On;
                    mActual = l;
                    l = 1;
                }
                else {
                    ++l;
                }
            break;
            case FState::On:
                if (v == State::Low) {
                    mState = FState::Off;
                    mActual = l;
                    l = 1;
                }
                else {
                    ++l;
                }
            break;
            }
            return mActual;
        }
        static inline uint16_t l{0};
        static inline uint16_t mActual{0};
        static inline FState mState{FState::Off};
    };

    static inline IQ_F process(const IQ_Sign v) {
        IQ_F f;
        f.i = rli.process(v.i);
        f.q = rlq.process(v.q);

        file << f.i << ',' << f.q << '\n';
        
        return f;
    }
    
    static inline SingleRunLength rli;
    static inline SingleRunLength rlq;
    static inline std::ofstream file{"rl.csv"};
};

struct Demodulation {
    static inline IQ_Bit process(const IQ v) {
        ++numberOfSamples;
        min = std::min(min, v.i);
        min = std::min(min, v.q);
        max = std::max(max, v.i);
        max = std::max(max, v.q);

        const auto iq_p = IQ2Pass::process(v);        

        IQ_Pass iq_s = {Square::process(iq_p.low), Square::process(iq_p.high)};
        sq << iq_s;   
        
        float i_low_max = iLowMax.process(iq_s.low.i);
        float q_low_max = qLowMax.process(iq_s.low.q);
        float i_high_max = iHighMax.process(iq_s.high.i);
        float q_high_max = qHighMax.process(iq_s.high.q);

        file << i_low_max << ',' << q_low_max << ',' << i_high_max << ',' << q_high_max << '\n';        
        IQ_Bit b;
        b.i = Thresh::process(i_low_max, i_high_max);
        b.q = Thresh::process(q_low_max, q_high_max);
        
        bits << b.i << ',' << b.q << '\n';
        
//        IQ v1 = Filter::process(v);
//        IQ_Sign v2 = comp.process(v1);
//        IQ_F v3 = RunLength::process(v2);
        
        return b;
    }

    static inline std::ofstream bits{"bits.csv"};
    static inline std::ofstream file{"max.csv"};
    static inline std::ofstream sq{"sq.csv"};
    
    static inline Max<10> iLowMax;
    static inline Max<10> qLowMax;
    static inline Max<5> iHighMax;
    static inline Max<5> qHighMax;
    
    static inline Comparator comp{0.01, -0.01};
    static inline float min{std::numeric_limits<float>::max()};
    static inline float max{std::numeric_limits<float>::min()};
    static inline uint32_t numberOfSamples{};
    
};

void normalize(std::vector<IQ>& values) {
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

int main(const int argc, const char* const* const argv) {
    std::vector<std::string> args{argv, argv + argc};    
    std::cout << "demod_fsk_01\n";
    if (args.size() <= 1) {
        std::cerr << "filename?\n";
        return -1;
    }
    std::cout << "file: " << args[1] << "\n";
    
    std::vector<IQ> adcValues;
    
    std::ifstream dataFile{args[1]};
    
    if (!dataFile.is_open()) {
        std::cerr << "open?\n";
        return -1;
    }
    std::string line;
    while(std::getline(dataFile, line)) {
        std::stringstream lineStream{line};
        std::string token;
        IQ iq;
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
    
    std::vector<IQ_Bit> bits;
    
    const uint8_t down = 16;
    uint8_t c = 0;
    for(const auto& v : adcValues) {
        if (c++ == down) {
            c = 0;
            const auto r = Demodulation::process(v);
            bits.push_back(r);
        }
    }
    
    std::cout << "nsamples: " << Demodulation::numberOfSamples << '\n';
    std::cout << "min: " << Demodulation::min << '\n';
    std::cout << "max: " << Demodulation::max << '\n';
}
