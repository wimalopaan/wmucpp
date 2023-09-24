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

struct IQ_Bands {
    IQ low{};
    IQ high{};
};

std::ostream& operator<<(std::ostream& s, const IQ_Bands v) {
    return s << v.low.i << ',' << v.low.q << ',' << v.high.i << ',' << v.high.q << '\n';
}

constexpr double sinc(const double x) {
    if (x == 0.0) return 1.0;
    return sin(M_PI * x) / (M_PI * x);
}

constexpr inline void crc16(uint16_t& crc, const uint8_t value) {
    constexpr uint16_t crc_polynome = 0x1021;
    crc = crc ^ (((uint16_t)value) << 8);
    for(uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ crc_polynome;
        }
        else {
            crc = (crc << 1);
        }
    }
}


struct Config {
    Config() = delete;
    static inline constexpr uint8_t down = 16;
    static inline constexpr float fs = 781250.0 / down;
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


struct BandPass;
template<uint8_t Length, typename Type = BandPass, float fs = 48000, float f1 = 4500, float f2 = 5500>
struct FirFilter;

template<uint8_t L, float fs, float f1, float f2>
struct FirFilter<L, BandPass, fs, f1, f2> {
    constexpr float process(const float v) {
        buffer[in] = v;
        const float result = [&]{
            float sum{};
            for(uint8_t i{0}, k = in; i < L; ++i) {
                sum += coeff[i] * buffer[k];
                if (k == 0) {
                    k = L - 1;
                }
                else {
                    --k;
                }
            }
            return sum;
        }();
        in += 1;
        if (in == L) in = 0;
        return result;
    }
private:
    uint8_t in{0};
    std::array<float, L> buffer{};    
public:
    static inline constexpr auto coeff = []{
        std::array<double, L> window;
        constexpr double alpha   = 0.54;
        constexpr double beta    = 0.46;
        for(int i = 0; i < L; i++) {
            window[i] = alpha - beta * cos(2.0 * M_PI * i / (L - 1));
        }
        std::array<float, L> cc;
        constexpr double df1 = f1 / fs;
        constexpr double df2 = f2 / fs;
        for(int i = 0; i < L; i++) {
            double n = i - ((L - 1) / 2.0); 
            cc[i] = 2.0 * df1 * sinc(2.0 * df1 * n) - 2.0 * df2 * sinc(2.0 * df2 * n);
            cc[i] *= window[i];
        }
        
        constexpr double fc = (f1 + f2) / 2.0;
        double gain_c = 0.0;
        double gain_r = 0.0;
        double gain_i = 0.0;
        for(int i = 0; i < L; i++) {
            gain_r += cos(2 * M_PI * (fc / fs) * i) * cc[i];
            gain_i += sin(2 * M_PI * (fc / fs) * i) * cc[i];
        }
        gain_c = sqrt(gain_r * gain_r + gain_i * gain_i);
        for(int i = 0; i < L; i++) {
            cc[i] /= gain_c;
        }
        return cc;
    }();
};


struct Bandpass {
    Bandpass() = delete;
    static inline constexpr IQ_Bands process(const IQ v) {
        IQ low{fir_bp_low_i.process(v.i), fir_bp_low_q.process(v.q)};
        IQ high{fir_bp_high_i.process(v.i), fir_bp_high_q.process(v.q)};
        return {low, high};
    }
private:
    static inline FirFilter<Config::firLength, BandPass, Config::fs, Config::bpLow_fl, Config::bpLow_fh> fir_bp_low_i;
    static inline FirFilter<Config::firLength, BandPass, Config::fs, Config::bpHigh_fl, Config::bpHigh_fh> fir_bp_high_i;
    static inline FirFilter<Config::firLength, BandPass, Config::fs, Config::bpLow_fl, Config::bpLow_fh> fir_bp_low_q;
    static inline FirFilter<Config::firLength, BandPass, Config::fs, Config::bpHigh_fl, Config::bpHigh_fh> fir_bp_high_q;
};

struct Square {
    Square() = delete;
    static inline constexpr IQ process(const IQ v) {
        return {v.i * v.i, v.q * v.q};
    }        
    static inline constexpr IQ_Bands process(const IQ_Bands v) {
        return {process(v.low), process(v.high)};
    }        
};

template<uint8_t L>
struct Max {
    constexpr float process(const float v) {
        buffer[in] = v;
        const float max = [&]{
            float max{0};
            for(const float& x : buffer) {
                if (x > max) max = x;
            }
            return max;
        }();
        in += 1;
        if (in == L) in = 0;
        return max;
    }            
private:
    std::array<float, L> buffer{};
    uint8_t in{0};
};

template<uint8_t Low, uint8_t High>
struct Maximum {
    Maximum() = delete;
    static inline constexpr IQ_Bands process(const IQ_Bands v) {
        IQ low{iLowMax.process(v.low.i), qLowMax.process(v.low.q)};
        IQ high{iHighMax.process(v.high.i), qHighMax.process(v.high.q)};
        return {low, high};
    }
private:
    static inline Max<Low> iLowMax;
    static inline Max<Low> qLowMax;
    static inline Max<High> iHighMax;
    static inline Max<High> qHighMax;
};

struct Thresh {
    Thresh() = delete;
    static inline constexpr bool process(const float l, const float h) {
        return h > l;
    }            
    static inline constexpr IQ_Bit process(const IQ_Bands v) {
        IQ_Bit b;
        b.i = Thresh::process(v.low.i, v.high.i);
        b.q = Thresh::process(v.low.q, v.high.q);
        return b;
    }
};

//struct Comparator {
//    explicit Comparator(const float high, const float low) :
//        ic{high, low}, qc{high, low} {}

//    struct SingleComparator {
//        explicit SingleComparator(const float high, const float low) :
//            mThreshHigh{high}, mThreshLow{low} {}
//        State process(const float v) {
//            switch(mState) {
//            case State::Low:
//                if (v > mThreshHigh) {
//                    mState = State::High;
//                }
//            break;
//            case State::High:
//                if (v < mThreshLow) {
//                    mState = State::Low;
//                }
//            break;
//            }            
//            return mState;
//        }
//        State mState{State::Low};
//        float mThreshHigh{};
//        float mThreshLow{};
//    };

//    IQ_Sign process(const IQ v) {
//        IQ_Sign s;
//        s.i = ic.process(v.i);
//        s.q = qc.process(v.q);

//        file << (uint16_t)s.i << ',' << (uint16_t)s.q << '\n';

//        return s;
//    }
//    SingleComparator ic;
//    SingleComparator qc;

//    static inline std::ofstream file{"comp.csv"};
//};

struct Stats {
    Stats() = delete;
    static inline constexpr void process(const IQ v) {
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

struct Protocoll {
    Protocoll() = delete;
    struct ByteStuff {
        ByteStuff() = delete;
        static inline constexpr bool process(const bool b) {
            if (mBitInByteCounter++ < 8) {
                mActual >>= 1;
                if (b) {
                    mActual |= 0x80;    
                }
            }
            else {
                if (b) {
                    return true; // error
                }
                else {
                    mData[mByteCounter++] = mActual;
                    mActual = 0;
                    mBitInByteCounter = 0;
                }
            }
            if (mByteCounter == Config::bytesInFrame) {
                const uint16_t cs = []{
                    uint16_t v{0};
                    for(uint8_t i{0}; i < 8; ++i) {
                        crc16(v, mData[i]);
                    }
                    return v;
                }();
                reset();
                for(const auto& v : mData) {
                    if (&v == &mData[0]) {
                        data << (uint16_t)v;
                    }
                    else {
                        data << ',' << (uint16_t)v;
                    }
                }
                const uint16_t cs_p = mData[8] + (mData[9] << 8);
                if (cs_p == cs) {
                    data << " +";
                }
                else {
                    data << " !";
                }
                data << '\n';
                return true; // complete
            }
            return false; // not complete
        }
        static inline void reset() {
            mActual = 0;
            mBitInByteCounter = 0;
            mByteCounter = 0;
        }
    private:
        static inline uint8_t mActual = 0;
        static inline uint8_t mBitInByteCounter = 0;
        static inline uint32_t mByteCounter{0};
        static inline std::ofstream data{"data.csv"};
    };
    
    enum class State {Undefined, WaitForSync, Sync, Start, Bit};
    
    static inline constexpr void process(const IQ_Bit b) {
        bool bit = b.i; // was ist mit q?
        const State oldState = mState;
        ++mStateCounter;
        switch(mState) {
        case State::Undefined:
            if (!bit) {
                mState = State::WaitForSync;
            }
        break;
        case State::WaitForSync:
            if (bit) {
                mState = State::Sync;
                ++mBitTickCounter;
            }
        break;
        case State::Sync:
            if (bit) {
                ++mBitTickCounter;
            }
            else {
                if (mBitTickCounter >= Config::syncTicks) {
                    mState = State::Start;
                }
                else {
                    mState = State::WaitForSync;
                }
            }
        break;
        case State::Start:
            if (++mBitTickCounter >= mNextBitTick) {
                mNextBitTick += Config::bitTicks;
                if (bit) {
                    mState = State::Undefined;
                }
                else {
                    mState = State::Bit;
                }
            }
        break;
        case State::Bit:
            if (++mBitTickCounter >= mNextBitTick) {
                mNextBitTick += Config::bitTicks;
                if (ByteStuff::process(bit)) {
                    mState = State::Undefined;
                }
//                if (mBitCounter > (bitsInFrame + 1)) {
//                    mState = State::Undefined;
//                }
                ++mBitCounter;
            }
        break;
        }
        if (oldState != mState) {
            mStateCounter = 0;
            switch(mState) {
            case State::Undefined:
            break;
            case State::WaitForSync:
                mBitTickCounter = 0;
            break;
            case State::Sync:
            break;
            case State::Start:
                mBitCounter = 0;
                mBitTickCounter = 0;
                mNextBitTick = Config::halfBitTicks;
                ++mSyncs;
            break;
            case State::Bit:
                mBitCounter = 1;
                ByteStuff::reset();
            break;
            }                
        }
        frame << mBitCounter << ',' << mSyncs << '\n';
    }
private:    
    static inline uint32_t mBitTickCounter{};
    static inline float mNextBitTick{Config::halfBitTicks};

    static inline uint32_t mSyncs{0};

    static inline uint32_t mBitCounter{};

    static inline uint32_t mStateCounter{};
    static inline State mState{State::Undefined};
    
    static inline std::array<uint8_t, Config::bytesInFrame> mData;
    
    static inline std::ofstream frame{"frame.csv"};
};

struct Demodulation {
    Demodulation() = delete;
    static inline constexpr IQ_Bit process(const IQ v) {
        Stats::process(v);
        
        const IQ_Bands iq_p = Bandpass::process(v);        
        filt << iq_p;
        
        IQ_Bands iq_s = Square::process(iq_p);
        sq << iq_s;   

        IQ_Bands iq_m = Maximum<Config::lobeLow, Config::lobeHigh>::process(iq_s);
        maxs << iq_m;
        
        // ggf. kann man auch max_i und max_q addieren vor der Thresh
        
        IQ_Bit b = Thresh::process(iq_m);
        bits << b.i << ',' << b.q << '\n';

        Protocoll::process(b);
        
        return b;
    }
private:
    static inline std::ofstream filt{"filt.csv"};
    static inline std::ofstream sq{"sq.csv"};
    static inline std::ofstream maxs{"max.csv"};
    static inline std::ofstream bits{"bits.csv"};
};

constexpr void normalize(std::vector<IQ>& values) {
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
    
    uint8_t c = 0;
    for(const auto& v : adcValues) {
        if (c == 0) {
            const auto r = Demodulation::process(v);
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
