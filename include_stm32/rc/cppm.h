/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <cmath>

#include "dsp.h"

namespace Dsp {
    namespace Cppm {
        
        struct Peak {
            size_t pos{};
            float  value{};
        };
        
        template<typename Config>
        struct MatchedFilter {
            float process(const float v) {
                return correlate.process(v);
            }
        private:
            const std::array<float, Config::nPulse> chirp{[]{
                std::array<float, Config::nPulse> frequencies{};
                for(size_t i{}; auto& f : frequencies) {
                    const float q = 1 - exp((-1.0 * Config::tauPulse * i) / Config::nPulse);
                    f = Config::zf + Config::fp * q;
                    ++i;
                }    
                std::array<float, Config::nPulse> sumFreqs;
                sumFreqs[0] = frequencies[0];
                for(size_t i = 1; i < frequencies.size(); ++i) {
                    sumFreqs[i] = sumFreqs[i - 1] + frequencies[i];
                }
                std::array<float, Config::nPulse> ch;
                for(size_t i = 0; i < sumFreqs.size(); ++i) {
                    ch[Config::nPulse - 1 - i] = sin(2.0 * std::numbers::pi * sumFreqs[i] / Config::fs); // reversed
                }
                return ch;
            }()};
            Dsp::Convolve<Config::nPulse> correlate{chirp};
        };
        
        template<typename Config>
        struct Hamming {
            float process(const float v) {
                return convolve.process(v);
            }
        private:
            const std::array<float, Config::nPulse> window{[]{
                    std::array<float, Config::nPulse> data{};
                    for(size_t i{}; auto& d : data) {
                        d = 0.54 -0.46 * cos((2 * std::numbers::pi * i++) / (Config::nPulse - 1));
                    }
                    return data;
            }()};            
            Dsp::Convolve<Config::nPulse> convolve{window};
        };

        template<typename Config>
        struct PeakMax {
            float process(const float v) {
                if (v > max) {
                    max = v;
                    last = Config::nMaxDistance;
                }
                else {
                    if (--last == 0) {
                        max = v;
                    }
                }
                return max;
            }
        private:
            float max{};
            size_t last{};
        };

        template<typename Config, typename CallBack = void, typename Global = void>
        struct PeakFind {
            enum class State {Low, TransUp, Above, TransDown, Found, Sync};
            using return_type = std::conditional_t<std::is_same_v<Global, void>, void, Peak>;
            
            return_type process(const float v) {
                if constexpr(!std::is_same_v<Global, void>) {
                    ++Global::globalCounter; // sample in Frame?
                }
                Peak p;
                const float thresh = expMax.process(v) * Config::fThreshPeakFind;
                const State oldState = state;
                ++stateCounter;
                if (lastPeakCounter) {
                    *lastPeakCounter += 1;
                }
                ++frameCounter;
                switch(state) {
                case State::Sync:
                    if (v < thresh) {
                        for(size_t i = 0; i < peakIntervalls.size(); ++i) {
                            const size_t d = peakPositions[i + 1] - peakPositions[i];
                            if ((d <= Config::nMaxPulse) && (d >= Config::nMinPulse)) {
                                peakIntervalls[i] = d;
                            }
                        }
                        if constexpr(!std::is_same_v<CallBack, void>) {
                            CallBack::process(peakIntervalls);
                        }
                        lastPeakCounter = std::nullopt;
                        numberOfPeaksFound = 0;
                        frameCounter = 0;
                        state = State::Low;
                    }
                break;
                case State::Low:
                    if (v > thresh) {
                        state = State::TransUp;
                        last = v;
                    }
                    else {
                        if (numberOfPeaksFound >= 1) {
                            if (lastPeakCounter && (*lastPeakCounter > Config::nMaxDistance)) {
                                state = State::Sync;
                            }
                        }
                    }
                break;
                case State::TransUp:
                    if (v < thresh) {
                        state = State::Low;
                    }
                    else {
                        if (stateCounter > (Config::nPulse / 4)) {
                            state = State::Above;
//                            startAbove = v;
                        }
                    }
                    last = v;
                break;
                case State::Above:
                    if (v < last) {
                        max = last;
                        state = State::TransDown;
                    }
                    last = v;
                break;
                case State::TransDown:
                    if (v > max) {
                        max = v;
                        state = State::Above;
                    }
                    else {
                        if (stateCounter > (Config::nPulse / 4)) {
                            state = State::Found;
                            if constexpr(!std::is_same_v<Global, void>) {
                                p.pos = Global::globalCounter;
                                p.value = max;
                            }
                            if (numberOfPeaksFound < 9) {
                                peakPositions[numberOfPeaksFound++] = frameCounter;
//                                peakPositions[numberOfPeaksFound++] = globalCounter;
                            }
                            lastPeakCounter = 0;
                        }
                    }
                    last = v;
                break;
                case State::Found:
                    if (v < thresh) {
                        state = State::Low;
                    }
                break;
                }
                if (oldState != state) {
                    stateCounter = 0;
                }
                if constexpr(!std::is_same_v<Global, void>) {
                    return p;
                }
            }
        private:
//            size_t globalCounter{};
            State state{State::Low};
            ExpMax<Config> expMax{Config::fMaxPeakFind};
            size_t stateCounter{};
            size_t frameCounter{};
//            float startAbove{};
            float last{};
            float max{};
            size_t numberOfPeaksFound{};
            std::optional<size_t> lastPeakCounter;
            std::array<size_t, 9> peakPositions{};
            
            std::array<uint16_t, 8> peakIntervalls{[]{
                    std::array<uint16_t, 8> data;
                    for(auto& d: data) {
                        d = (Config::nMaxPulse + Config::nMinPulse) / 2;
                    }
                    return data;
            }()};
        };
        
        template<typename Config, typename Callback, typename Export = void>
        struct Demodulation {
            static void process(const Dsp::IQ v) {
                if constexpr (!std::is_same_v<Export, void>) {
                    Export::input(v.i); 
                }
                const float vHigh = highPass.process(v.i);
                if constexpr (!std::is_same_v<Export, void>) {
                    Export::highpass(vHigh); 
                }
                const float vMax = expMax.process(vHigh);
                if constexpr (!std::is_same_v<Export, void>) {
                    Export::max(vMax); 
                }
                const float vNormMax = vHigh / (0.001 + vMax);
                if constexpr (!std::is_same_v<Export, void>) {
                    Export::normMax(vMax); 
                }
                const float vChirp = chirpCorrelator.process(vNormMax);  
                if constexpr (!std::is_same_v<Export, void>) {
                    Export::chirp(vChirp); 
                }
                const float vChirpS = vChirp * vChirp;
                if constexpr (!std::is_same_v<Export, void>) {
                    Export::chirpS(vChirpS); 
                }
                const float vPeak = window.process(vChirpS); 
                if constexpr (!std::is_same_v<Export, void>) {
                    Export::peak(vPeak); 
                }
                const float vPeakMax = peakMax.process(vPeak); 
                if constexpr (!std::is_same_v<Export, void>) {
                    Export::peakMax(vPeakMax); 
                }
                if constexpr (!std::is_same_v<Export, void>) {
                    const Peak vPeakPos = peakFind.process(vPeak); 
                    Export::peakPos(vPeakPos); 
                }
                else {
                    peakFind.process(vPeak); 
                }
            }  
        private:
            inline static ExpMax<Config> expMax{Config::fMaxFilter};
            inline static HighPass<Config> highPass;
            inline static MatchedFilter<Config> chirpCorrelator;
            inline static Hamming<Config> window;
            inline static PeakMax<Config> peakMax;
            inline static PeakFind<Config, Callback, Export> peakFind;
        };
    }
}
