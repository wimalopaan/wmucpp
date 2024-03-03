#pragma once

#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"
#include "rf.h"
#include "units.h"

#include <type_traits>
#include <concepts>
#include <cstddef>
#include <array>

namespace External {
    namespace SI5351 {
        static inline constexpr uint8_t OutputEnableReg{3};
        static inline constexpr uint8_t Clk0Control{16};
        static inline constexpr uint8_t Clk1Control{17};
        static inline constexpr uint8_t Clk2Control{18};
        static inline constexpr uint8_t PllResetReg{177};
        static inline constexpr uint8_t Clk0Phase{165};
        static inline constexpr uint8_t Clk1Phase{166};
        static inline constexpr uint8_t Clk2Phase{167};
        
        static inline constexpr uint8_t PllARegStart{26};
        static inline constexpr uint8_t MSynth0RegStart{42};
        static inline constexpr uint8_t MSynth1RegStart{50};
        static inline constexpr uint8_t MSynth2RegStart{58};

       static inline constexpr uint32_t maxPllf{900'000'000UL};
        // static inline constexpr uint32_t maxPllf{700'000'000UL};
        static inline constexpr uint32_t xtalfreq{25'000'000UL};
        
        struct SetupData {
            using data_type = std::array<std::byte, 8>;
            data_type pll{};
            data_type msynth{};
            uint32_t a{};
            uint32_t b{};
            uint32_t c{};
            uint32_t div{};
        };

        static inline constexpr auto calculateSetupData(const uint32_t ff) {
            SetupData data;
            uint32_t divider = maxPllf / ff;
            if (divider % 2) --divider;
            
            const uint32_t pllFreq = divider * ff;
            const uint32_t mult = pllFreq / xtalfreq;
            const uint32_t l = pllFreq % xtalfreq;
            double f = l;
            const uint32_t denom = 1'048'575UL;
            f *= denom;
            f /= xtalfreq;
            const uint32_t num = f;
            
            data.a = mult;
            data.b = num;
            data.c = denom;
            data.div = divider;
            
            uint32_t P1;					// PLL config register P1
            uint32_t P2;					// PLL config register P2
            uint32_t P3;					// PLL config register P3

            P1 = (uint32_t)(128 * ((double)num / (double)denom));
            P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
            P2 = (uint32_t)(128 * ((double)num / (double)denom));
            P2 = (uint32_t)(128 * num - denom * P2);
            P3 = denom;
             
            data.pll[0] = std::byte((P3 & 0x0000FF00) >> 8);
            data.pll[1] = std::byte((P3 & 0x000000FF));
            data.pll[2] = std::byte((P1 & 0x00030000) >> 16);
            data.pll[3] = std::byte((P1 & 0x0000FF00) >> 8);
            data.pll[4] = std::byte((P1 & 0x000000FF));
            data.pll[5] = std::byte(((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
            data.pll[6] = std::byte((P2 & 0x0000FF00) >> 8);
            data.pll[7] = std::byte((P2 & 0x000000FF));
            
            P1 = 128ULL * divider - 512;
            P2 = 0;							// P2 = 0, P3 = 1 forces an integer value for the divider
            P3 = 1;
        
            uint8_t rDiv = 0;
            
            data.msynth[0] = std::byte((P3 & 0x0000FF00) >> 8);
            data.msynth[1] = std::byte((P3 & 0x000000FF));
            data.msynth[2] = std::byte(((P1 & 0x00030000) >> 16) | rDiv);
            data.msynth[3] = std::byte((P1 & 0x0000FF00) >> 8);
            data.msynth[4] = std::byte((P1 & 0x000000FF));
            data.msynth[5] = std::byte(((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
            data.msynth[6] = std::byte((P2 & 0x0000FF00) >> 8);
            data.msynth[7] = std::byte((P2 & 0x000000FF));
            
            return data;
        }

        template<uint8_t N>
        requires (N <= 2) 
        using IOutput = std::integral_constant<uint8_t, N>;

        template<uint8_t N>
        requires (N <= 2) 
        using QOutput = std::integral_constant<uint8_t, N>;
        
        template<typename Bus, auto Adr, typename IO, typename QO, int32_t FreqOffset = 0>
        struct IQClock { // 90 degree between outputs
            using bus = Bus;
            
//            static_assert(bus::size() > 8);
//            std::integral_constant<uint16_t, External::RC::channels.size()>::_;
            
            enum class State : uint8_t {Idle, 
                                        ReadWait, 
                                        SetupChannelStart, SetupPLLA, 
                                        SetupMSynth0, SetupMSynth1, SetupMSynth2,
                                        SetupPhase,
                                        SetupChannelReset, 
                                        SetupChannelOutput0, SetupChannelOutput1, SetupChannelOutput2, 
                                        SetupChannelOutputEnable, 
                                        SetupChannelComplete
                                       };

            static inline void periodic() {
                switch(mState) {
                case State::Idle:
                    break;
                case State::ReadWait:
                    break;
                case State::SetupChannelStart:
                    mState = State::SetupPLLA;
                    break;
                case State::SetupPLLA:
                    if (bus::write(Adr, std::byte{PllARegStart}, actual.pll)) {
                        mState = State::SetupMSynth0;
                    }
                    break;
                case State::SetupMSynth0:
                    if (bus::write(Adr, std::byte{MSynth0RegStart}, actual.msynth)) {
                        mState = State::SetupMSynth1;
                    }
                    break;
                case State::SetupMSynth1:
                    if (bus::write(Adr, std::byte{MSynth1RegStart}, actual.msynth)) {
                        mState = State::SetupMSynth2;
                    }
                    break;
                case State::SetupMSynth2:
                    if (bus::write(Adr, std::byte{MSynth2RegStart}, actual.msynth)) {
                        mState = State::SetupPhase;
                    }
                    break;
                case State::SetupPhase:
                {
                    uint8_t phase = actual.div & 0x7f;
                    if constexpr(QO::value == 0) {
                        if (bus::write(Adr, {std::byte{Clk0Phase}, std::byte{phase}})) {
                            mState = State::SetupChannelReset;
                        }
                    }
                    else if constexpr(QO::value == 1) {
                        if (bus::write(Adr, {std::byte{Clk1Phase}, std::byte{phase}})) {
                            mState = State::SetupChannelReset;
                        }
                    }
                    else {
                        if (bus::write(Adr, {std::byte{Clk2Phase}, std::byte{phase}})) {
                            mState = State::SetupChannelReset;
                        }
                    }
                }
                break;
                case State::SetupChannelReset:
                    if (bus::write(Adr, {std::byte{PllResetReg}, std::byte{0xA0}})) {
                        mState = State::SetupChannelOutput0;
                    }
                    break;
                case State::SetupChannelOutput0:
                    if (bus::write(Adr, {std::byte{Clk0Control}, std::byte{0x4f}})) {
                        mState = State::SetupChannelOutput1;
                    }
                    break;
                case State::SetupChannelOutput1:
                    if (bus::write(Adr, {std::byte{Clk1Control}, std::byte{0x4f}})) {
                        mState = State::SetupChannelOutput2;
                    }
                    break;
                case State::SetupChannelOutput2:
                    if (bus::write(Adr, {std::byte{Clk2Control}, std::byte{0x4f}})) {
                        mState = State::SetupChannelOutputEnable;
                    }
                    break;
                case State::SetupChannelOutputEnable:
                    if (bus::write(Adr, {std::byte{OutputEnableReg}, std::byte{0x00}})) {
                        mState = State::SetupChannelComplete;
                    }
                    break;
                case State::SetupChannelComplete:
                    break;
                }
            }
            static inline bool setFrequency(const Units::hertz& f) {
                if (mState == State::SetupChannelComplete) {
                    mState = State::Idle;
                    return true;
                }
                else {
                    if (mState == State::Idle) {
                        actual = calculateSetupData(f.value + FreqOffset);
                        mState = State::SetupChannelStart;
                    }
                    return false;
                }
            }
            
        private:
            static inline SetupData actual;
            static inline State mState{State::Idle};
        };
        
        template<typename Bus, auto Adr, int32_t FreqOffset = 0>
        struct Clock {
            using bus = Bus;
            
//            static_assert(bus::size() > 8);
//            std::integral_constant<uint16_t, External::RC::channels.size()>::_;
            
            static inline constexpr auto setupChannels = []{
                std::array<SetupData, External::RC::channels.size()> data;
                
                for(uint8_t i{0}; const External::RC::Channel c1 : External::RC::channels) {
                    data[i] = calculateSetupData(c1.mFreq + FreqOffset);
                    i++;
                }
                return data;
            }();
    
            static inline constexpr auto setupChannelsU = []{
                std::array<SetupData, External::RC::channels.size()> data;
                
                for(uint8_t i{0}; const External::RC::Channel c1 : External::RC::channels) {
                    data[i] = calculateSetupData(c1.mFreq + FreqOffset + 2'000);
                    i++;
                }
                return data;
            }();

            
            enum class State : uint8_t {Idle, ReadWait, 
                                        SetupChannelStart, SetupChannelPLL, SetupChannelMSynth, SetupChannelReset, SetupChannelOutput, SetupChannelOutput2, SetupChannelComplete
                                       };
            
            
            static inline uint8_t iindex{};
            static inline uint8_t ichannel{};

            static inline uint8_t ipll{26};
            static inline uint8_t imsynth{42};
            static inline uint8_t ireg{16};
            static inline uint8_t iregValue{0x4f};
            static inline uint8_t iclk0PhaseOffset{165};
            
            static inline void periodic() {
                switch(mState) {
                case State::Idle:
                    break;
                case State::ReadWait:
                    break;
                case State::SetupChannelStart:
                    iindex = 0;
                    mState = State::SetupChannelPLL;
                    break;
                case State::SetupChannelPLL:
                    if (bus::write(Adr, {std::byte(iindex + ipll), actual.pll[iindex]})) {
                        ++iindex;
                        if (iindex >= std::tuple_size<SetupData::data_type>::value) {
                            mState = State::SetupChannelMSynth;
                            iindex = 0;
                        }
                    }
                    break;
                case State::SetupChannelMSynth:
                    if (bus::write(Adr, {std::byte(iindex + imsynth), actual.msynth[iindex]})) {
                        ++iindex;
                        if (iindex >= std::tuple_size<SetupData::data_type>::value) {
                            mState = State::SetupChannelReset;
                            iindex = 0;
                        }
                    }
                    break;
                case State::SetupChannelReset:
                    if (bus::write(Adr, {std::byte{PllResetReg}, std::byte{0xA0}})) {
                        mState = State::SetupChannelOutput;
                    }
                    break;
                case State::SetupChannelOutput:
                    if (bus::write(Adr, {ireg, iregValue})) {
                        mState = State::SetupChannelOutput2;
                    }
                    break;
                case State::SetupChannelOutput2:
                    if (bus::write(Adr, {std::byte{OutputEnableReg}, std::byte{0x00}})) {
                        mState = State::SetupChannelComplete;
                    }
                    break;
                case State::SetupChannelComplete:
                    break;
                }
            }

            static inline bool off() {
                return bus::write(Adr, {OutputEnableReg, std::byte{0xff}});
            }
            
            static inline bool on() {
                return bus::write(Adr, {OutputEnableReg, std::byte{0x00}});
            }

            static inline bool phase(const uint8_t output, uint8_t phase) {
                phase &= 0x7f;
                return bus::write(Adr, {iclk0PhaseOffset + output, phase});
            }

            static inline bool setupWithClockBuilderData() {
                if (mState == State::DefaultSetupComplete) {
                    mState = State::Idle;
                    return true;
                }
                else {
                    if (mState == State::Idle) {
                        mState = State::DefaultSetup1;
                    }
                    return false;
                }
            }
            
            static inline void setOutput(const uint8_t o) {
                if (o == 2) {
                    ipll = 34; // PLLB
                    imsynth = 58;
                    ireg = 18;
                    iregValue = 0x6f; // use PLLB                    
                }
                else if (o == 1) {
                    ipll = 34; // PLLB
                    imsynth = 50;
                    ireg = 17;
                    iregValue = 0x6f; // use PLLB
                }
                else {
                    ipll = 26; // PLLA
                    imsynth = 42;
                    ireg = 16;
                    iregValue = 0x4f; // use PLLA
                }
            }

            static inline void setOutputSamePll(const uint8_t o, bool invert = false) {
                if (o == 2) {
                    ipll = 26; // PLLA
                    imsynth = 58;
                    ireg = 18;
                    iregValue = 0x4f; // use PLLA                    
                }
                else if (o == 1) {
                    ipll = 26; // PLLA
                    imsynth = 50;
                    ireg = 17;
                    iregValue = 0x4f; // use PLLA
                }
                else {
                    ipll = 26; // PLLA
                    imsynth = 42;
                    ireg = 16;
                    iregValue = 0x4f; // use PLLA
                }
                if (invert) {
                    iregValue |= 0x10;
                }
            }
            
            static inline bool setFrequency(const Units::hertz& f, uint32_t* div = nullptr) {
                if (mState == State::SetupChannelComplete) {
                    mState = State::Idle;
                    return true;
                }
                else {
                    if (mState == State::Idle) {
                        actual = calculateSetupData(f.value + FreqOffset);
                        if (div) {
                            *div = actual.div;
                        }
                        mState = State::SetupChannelStart;
                    }
                    return false;
                }
            }
            
            static inline bool setChannel(const uint16_t c) {
                if (mState == State::SetupChannelComplete) {
                    mState = State::Idle;
                    return true;
                }
                else {
                    if (mState == State::Idle) {
                        ichannel = c;
                        actual = setupChannels[c];
                        mState = State::SetupChannelStart;
                    }
                    return false;
                }
                
            }
            static inline bool setChannelUpperFreq(const uint16_t c) {
                if (mState == State::SetupChannelComplete) {
                    mState = State::Idle;
                    return true;
                }
                else {
                    if (mState == State::Idle) {
                        ichannel = c;
                        actual = setupChannelsU[c];
                        mState = State::SetupChannelStart;
                    }
                    return false;
                }
                
            }
        private:
            static inline SetupData actual;
            static inline uint8_t nextRegNr{0};
            static inline State mState{State::Idle};
        };
    }
}
