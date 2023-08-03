#pragma once

#include <cstdint>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>

#include "dsp.h"

// TODO:
// - oversampling der Bits: drei Werte je Bit
// - i/q kombinieren

//struct Config_XXX {
////    Config() = delete;
//    static inline constexpr uint8_t down = 1;
//    static inline constexpr float fs = 48000.0 / down;
//        static inline constexpr float fb = 2048.0f;
////    static inline constexpr float fb = 2 * 2048.0f;
//    static inline constexpr float bitTicks = fs / fb;
////    static inline constexpr uint16_t firLength = 23;
//    static inline constexpr uint16_t firLength = bitTicks + 0.5;
//    static inline constexpr float halfBitTicks = fs / (2.0f * fb);
//    static inline constexpr float syncTicks = bitTicks * 10.0f - halfBitTicks;
//    static inline constexpr float zf = 5'000.0f;
//    static inline constexpr float fSymbolLow = 0.0f;
//    static inline constexpr float fSymbolHigh = 5'000.0f;
//    static inline constexpr float fLow  = zf + fSymbolLow;
//    static inline constexpr float fHigh = zf + fSymbolHigh;
//    static inline constexpr float bandwidth = 0.1f;
//    static inline constexpr float halfBandwidthLow = (bandwidth * fLow) / 2.0f;
//    static inline constexpr float halfBandwidthHigh = (bandwidth * fHigh) / 2.0f;
//    static inline constexpr float bpLow_fl = fLow - halfBandwidthLow;
//    static inline constexpr float bpLow_fh = fLow + halfBandwidthLow;
//    static inline constexpr float bpHigh_fl = fHigh - halfBandwidthLow;
//    static inline constexpr float bpHigh_fh = fHigh + halfBandwidthLow;
//    static inline constexpr uint16_t lobeLow = 0.5 * fs / fLow + 0.5;
//    static inline constexpr uint16_t lobeHigh = 0.5 * fs / fHigh + 0.5 + 1;

//    static inline constexpr uint16_t bytesInFrame = 10;
////    static inline constexpr uint16_t bitsInFrame = bytesInFrame * (8 + 1); // ohne erstes 0-Bit
//};


namespace Dsp {
    enum class State {High, Low};
    
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
    
    struct Square {
        Square() = delete;
        static inline constexpr IQ process(const IQ v) {
            return {v.i * v.i, v.q * v.q};
        }        
        static inline constexpr IQ_Bands process(const IQ_Bands v) {
            return {process(v.low), process(v.high)};
        }        
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
    
    struct Stats {
        Stats() = delete;
        static inline constexpr void process(const IQ v) {
            ++numberOfSamples;
            mMaxValue = fExp * mMaxValue;
            if (v.i > mMaxValue) mMaxValue = v.i;
            if (v.q > mMaxValue) mMaxValue = v.q;
        }
        static inline constexpr uint16_t max() {
            return mMaxValue;
        }
    private:
        static inline float fExp{0.9999f};
        static inline float mMaxValue{};
        static inline uint32_t numberOfSamples{};
    };
    
    template<typename Config, typename Term = void, typename Callback = void, typename ErrorPin = void>
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
                    const uint16_t cs_p = mData[8] + (mData[9] << 8);
                    if (cs != cs_p) {
                        ++mErrors;
                        if constexpr (!std::is_same_v<ErrorPin, void>) {
                            ErrorPin::set();
                        }
                    }
                    else { // ok
                        if constexpr (!std::is_same_v<ErrorPin, void>) {
                            ErrorPin::reset();
                        }
                        if constexpr(!std::is_same_v<Callback, void>) {
                            Callback::process(mData);
                        }
                    }
                    if constexpr (!std::is_same_v<Term, void>) {
                        IO::outl<Term>(mData[0], ',', mData[1], ',', mData[2], ',', mData[3], ',', mData[4], ',', mData[5], ',', mData[6], ',', mData[7], ',', (cs_p == cs) ? '+' : '-');
                    }
                    ++mPackages;
                    return true; // complete
                }
                return false; // not complete
            }
            static inline void reset() {
                mActual = 0;
                mBitInByteCounter = 0;
                mByteCounter = 0;
            }
            static inline uint16_t errors() {
                return mErrors;
            }
            static inline uint16_t packages() {
                return mPackages;
            }
        private:
            static inline uint_fast16_t mPackages = 0;
            static inline uint_fast16_t mErrors = 0;
            static inline uint_fast8_t mActual = 0;
            static inline uint_fast8_t mBitInByteCounter = 0;
            static inline uint_fast32_t mByteCounter{0};
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
        }
    private:    
        static inline uint_fast16_t mBitTickCounter{};
        static inline float mNextBitTick{Config::halfBitTicks};
        static inline uint_fast16_t mSyncs{0};
        static inline uint_fast16_t mBitCounter{};
        static inline uint_fast16_t mStateCounter{};
        static inline State mState{State::Undefined};
        static inline std::array<uint8_t, Config::bytesInFrame> mData;
    };
    
    namespace FSK {
        template<typename Config>
        struct Bandpass {
            Bandpass() = delete;
            static inline constexpr IQ_Bands process(const IQ v) {
                return fir_bp.process(v);
            }
        private:
            static inline FirFilterMulti<Config::firLength, BandPass, Config::fs, 
                          FCut{Config::bpLow_fl, Config::bpLow_fh}, FCut{Config::bpHigh_fl, Config::bpHigh_fh}> fir_bp;

//            static inline FirFilterMulti<Config::firLength, BandPass, Config::fs, Config::bpLow_fl, Config::bpLow_fh> fir_bp_low;
//            static inline FirFilterMulti<Config::firLength, BandPass, Config::fs, Config::bpHigh_fl, Config::bpHigh_fh> fir_bp_high;

//            static inline FirFilter<Config::firLength, BandPass, Config::fs, Config::bpLow_fl, Config::bpLow_fh> fir_bp_low_i;
//            static inline FirFilter<Config::firLength, BandPass, Config::fs, Config::bpHigh_fl, Config::bpHigh_fh> fir_bp_high_i;
//            static inline FirFilter<Config::firLength, BandPass, Config::fs, Config::bpLow_fl, Config::bpLow_fh> fir_bp_low_q;
//            static inline FirFilter<Config::firLength, BandPass, Config::fs, Config::bpHigh_fl, Config::bpHigh_fh> fir_bp_high_q;
        };
        
        template<typename Config, typename Term = void, typename CallBack = void, typename ErrorPin = void, typename Dac = void>
        struct Demodulation {
            Demodulation() = delete;
            using proto = Protocoll<Config, Term, CallBack, ErrorPin>;
            static inline constexpr IQ_Bit process(const IQ v) {
                Stats::process(v);
                /*volatile */const IQ_Bands iq_bandpassed = Bandpass<Config>::process(v);        
                
                if constexpr(!std::is_same_v<Dac, void>) {
                    Dac::set(v.i + 2048);
//                    if (out) {
//                        Dac::set(iq_bandpassed.low.i + 2048);
//                    }
//                    else {
//                        Dac::set(iq_bandpassed.high.i + 2048);
//                    }
                }
                
                IQ_Bands iq_squared = Square::process(iq_bandpassed);
        
                IQ_Bands iq_maximum = Maximum<Config::lobeLow, Config::lobeHigh>::process(iq_squared);
                
                // - ggf. kann man auch max_i und max_q addieren vor der Thresh
                // - oversampling 
                
                IQ_Bit iq_bits = Thresh::process(iq_maximum);
        
                proto::process(iq_bits);
                
                return iq_bits;
                
//                return {};
            }
            static inline constexpr void debugSwitch() {
                out = !out;
            }
        private:
            static inline bool out{};
        };
    }
}


