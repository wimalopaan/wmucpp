#pragma once

namespace External {
    namespace SI5351 {

        static inline constexpr std::array<std::pair<uint8_t, uint8_t>, 100> m_si5351_regs_15to92_149to170 = {{ 
            {15, 0x00}, /* Input source = crystal for PLLA and PLLB */
            {16, 0x4F}, /* CLK0 Control: 8mA drive, Multisynth 0 as CLK0 source, Clock
                           not inverted, Source = PLLA, Multisynth 0 in integer mode,
                           clock powered up */
            {17, 0x4F}, /* CLK1 Control: 8mA drive, Multisynth 1 as CLK1 source, Clock
                           not inverted, Source = PLLA, Multisynth 1 in integer mode,
                           clock powered up */
            {18, 0x6F}, /* CLK2 Control: 8mA drive, Multisynth 2 as CLK2 source, Clock
                           not inverted, Source = PLLB, Multisynth 2 in integer mode,
                           clock powered up */
            {19, 0x80}, /* CLK3 Control: Not used ... clock powered down */
            {20, 0x80}, /* CLK4 Control: Not used ... clock powered down */
            {21, 0x80}, /* CLK5 Control: Not used ... clock powered down */
            {22, 0x80}, /* CLK6 Control: Not used ... clock powered down */
            {23, 0x80}, /* CLK7 Control: Not used ... clock powered down */
            {24, 0x00}, /* Clock disable state 0..3 (low when disabled) */
            {25, 0x00}, /* Clock disable state 4..7 (low when disabled) */
            /* PLL_A Setup */
            {26, 0x00},
            {27, 0x05},
            {28, 0x00},
            {29, 0x0C},
            {30, 0x66},
            {31, 0x00},
            {32, 0x00},
            {33, 0x02},
            /* PLL_B Setup */
            {34, 0x02},
            {35, 0x71},
            {36, 0x00},
            {37, 0x0C},
            {38, 0x1A},
            {39, 0x00},
            {40, 0x00},
            {41, 0x86},
            /* Multisynth Setup */
            {42, 0x00},
            {43, 0x01},
            {44, 0x00},
            {45, 0x01},
            {46, 0x00},
            {47, 0x00},
            {48, 0x00},
            {49, 0x00},
            {50, 0x00},
            {51, 0x01},
            {52, 0x00},
            {53, 0x1C},
            {54, 0x00},
            {55, 0x00},
            {56, 0x00},
            {57, 0x00},
            {58, 0x00},
            {59, 0x01},
            {60, 0x00},
            {61, 0x18},
            {62, 0x00},
            {63, 0x00},
            {64, 0x00},
            {65, 0x00},
            {66, 0x00},
            {67, 0x00},
            {68, 0x00},
            {69, 0x00},
            {70, 0x00},
            {71, 0x00},
            {72, 0x00},
            {73, 0x00},
            {74, 0x00},
            {75, 0x00},
            {76, 0x00},
            {77, 0x00},
            {78, 0x00},
            {79, 0x00},
            {80, 0x00},
            {81, 0x00},
            {82, 0x00},
            {83, 0x00},
            {84, 0x00},
            {85, 0x00},
            {86, 0x00},
            {87, 0x00},
            {88, 0x00},
            {89, 0x00},
            {90, 0x00},
            {91, 0x00},
            {92, 0x00},
            /* Misc Config Register */
            {149, 0x00},
            {150, 0x00},
            {151, 0x00},
            {152, 0x00},
            {153, 0x00},
            {154, 0x00},
            {155, 0x00},
            {156, 0x00},
            {157, 0x00},
            {158, 0x00},
            {159, 0x00},
            {160, 0x00},
            {161, 0x00},
            {162, 0x00},
            {163, 0x00},
            {164, 0x00},
            {165, 0x00},
            {166, 0x00},
            {167, 0x00},
            {168, 0x00},
            {169, 0x00},
            {170, 0x00}
        }};
        
        enum {
          SI5351_REGISTER_0_DEVICE_STATUS = 0,
          SI5351_REGISTER_1_INTERRUPT_STATUS_STICKY = 1,
          SI5351_REGISTER_2_INTERRUPT_STATUS_MASK = 2,
          SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL = 3,
          SI5351_REGISTER_9_OEB_PIN_ENABLE_CONTROL = 9,
          SI5351_REGISTER_15_PLL_INPUT_SOURCE = 15,
          SI5351_REGISTER_16_CLK0_CONTROL = 16,
          SI5351_REGISTER_17_CLK1_CONTROL = 17,
          SI5351_REGISTER_18_CLK2_CONTROL = 18,
          SI5351_REGISTER_19_CLK3_CONTROL = 19,
          SI5351_REGISTER_20_CLK4_CONTROL = 20,
          SI5351_REGISTER_21_CLK5_CONTROL = 21,
          SI5351_REGISTER_22_CLK6_CONTROL = 22,
          SI5351_REGISTER_23_CLK7_CONTROL = 23,
          SI5351_REGISTER_24_CLK3_0_DISABLE_STATE = 24,
          SI5351_REGISTER_25_CLK7_4_DISABLE_STATE = 25,
          SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1 = 42,
          SI5351_REGISTER_43_MULTISYNTH0_PARAMETERS_2 = 43,
          SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3 = 44,
          SI5351_REGISTER_45_MULTISYNTH0_PARAMETERS_4 = 45,
          SI5351_REGISTER_46_MULTISYNTH0_PARAMETERS_5 = 46,
          SI5351_REGISTER_47_MULTISYNTH0_PARAMETERS_6 = 47,
          SI5351_REGISTER_48_MULTISYNTH0_PARAMETERS_7 = 48,
          SI5351_REGISTER_49_MULTISYNTH0_PARAMETERS_8 = 49,
          SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1 = 50,
          SI5351_REGISTER_51_MULTISYNTH1_PARAMETERS_2 = 51,
          SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3 = 52,
          SI5351_REGISTER_53_MULTISYNTH1_PARAMETERS_4 = 53,
          SI5351_REGISTER_54_MULTISYNTH1_PARAMETERS_5 = 54,
          SI5351_REGISTER_55_MULTISYNTH1_PARAMETERS_6 = 55,
          SI5351_REGISTER_56_MULTISYNTH1_PARAMETERS_7 = 56,
          SI5351_REGISTER_57_MULTISYNTH1_PARAMETERS_8 = 57,
          SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1 = 58,
          SI5351_REGISTER_59_MULTISYNTH2_PARAMETERS_2 = 59,
          SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3 = 60,
          SI5351_REGISTER_61_MULTISYNTH2_PARAMETERS_4 = 61,
          SI5351_REGISTER_62_MULTISYNTH2_PARAMETERS_5 = 62,
          SI5351_REGISTER_63_MULTISYNTH2_PARAMETERS_6 = 63,
          SI5351_REGISTER_64_MULTISYNTH2_PARAMETERS_7 = 64,
          SI5351_REGISTER_65_MULTISYNTH2_PARAMETERS_8 = 65,
          SI5351_REGISTER_66_MULTISYNTH3_PARAMETERS_1 = 66,
          SI5351_REGISTER_67_MULTISYNTH3_PARAMETERS_2 = 67,
          SI5351_REGISTER_68_MULTISYNTH3_PARAMETERS_3 = 68,
          SI5351_REGISTER_69_MULTISYNTH3_PARAMETERS_4 = 69,
          SI5351_REGISTER_70_MULTISYNTH3_PARAMETERS_5 = 70,
          SI5351_REGISTER_71_MULTISYNTH3_PARAMETERS_6 = 71,
          SI5351_REGISTER_72_MULTISYNTH3_PARAMETERS_7 = 72,
          SI5351_REGISTER_73_MULTISYNTH3_PARAMETERS_8 = 73,
          SI5351_REGISTER_74_MULTISYNTH4_PARAMETERS_1 = 74,
          SI5351_REGISTER_75_MULTISYNTH4_PARAMETERS_2 = 75,
          SI5351_REGISTER_76_MULTISYNTH4_PARAMETERS_3 = 76,
          SI5351_REGISTER_77_MULTISYNTH4_PARAMETERS_4 = 77,
          SI5351_REGISTER_78_MULTISYNTH4_PARAMETERS_5 = 78,
          SI5351_REGISTER_79_MULTISYNTH4_PARAMETERS_6 = 79,
          SI5351_REGISTER_80_MULTISYNTH4_PARAMETERS_7 = 80,
          SI5351_REGISTER_81_MULTISYNTH4_PARAMETERS_8 = 81,
          SI5351_REGISTER_82_MULTISYNTH5_PARAMETERS_1 = 82,
          SI5351_REGISTER_83_MULTISYNTH5_PARAMETERS_2 = 83,
          SI5351_REGISTER_84_MULTISYNTH5_PARAMETERS_3 = 84,
          SI5351_REGISTER_85_MULTISYNTH5_PARAMETERS_4 = 85,
          SI5351_REGISTER_86_MULTISYNTH5_PARAMETERS_5 = 86,
          SI5351_REGISTER_87_MULTISYNTH5_PARAMETERS_6 = 87,
          SI5351_REGISTER_88_MULTISYNTH5_PARAMETERS_7 = 88,
          SI5351_REGISTER_89_MULTISYNTH5_PARAMETERS_8 = 89,
          SI5351_REGISTER_90_MULTISYNTH6_PARAMETERS = 90,
          SI5351_REGISTER_91_MULTISYNTH7_PARAMETERS = 91,
          SI5351_REGISTER_092_CLOCK_6_7_OUTPUT_DIVIDER = 92,
          SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET = 165,
          SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET = 166,
          SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET = 167,
          SI5351_REGISTER_168_CLK3_INITIAL_PHASE_OFFSET = 168,
          SI5351_REGISTER_169_CLK4_INITIAL_PHASE_OFFSET = 169,
          SI5351_REGISTER_170_CLK5_INITIAL_PHASE_OFFSET = 170,
          SI5351_REGISTER_177_PLL_RESET = 177,
          SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE = 183
        };
        
        struct SetupData {
            using data_type = std::array<std::byte, 8>;
            data_type pll;
            data_type msynth;
            uint32_t a;
            uint32_t b;
            uint32_t c;
            uint32_t div;
        };
        
        static inline constexpr auto setupChannels = []{
            std::array<SetupData, External::RC::channels.size()> data;
            
            const uint32_t maxPllf{900'000'000UL};
            const uint32_t xtalfreq{25'000'000UL};
            
            for(uint8_t i{0}; const External::RC::Channel c1 : External::RC::channels) {
                const uint32_t ff = c1.mFreq;
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
                
                data[i].a = mult;
                data[i].b = num;
                data[i].c = denom;
                data[i].div = divider;
                
                uint32_t P1;					// PLL config register P1
                uint32_t P2;					// PLL config register P2
                uint32_t P3;					// PLL config register P3

                P1 = (uint32_t)(128 * ((double)num / (double)denom));
                P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
                P2 = (uint32_t)(128 * ((double)num / (double)denom));
                P2 = (uint32_t)(128 * num - denom * P2);
                P3 = denom;
                 
                data[i].pll[0] = std::byte((P3 & 0x0000FF00) >> 8);
                data[i].pll[1] = std::byte((P3 & 0x000000FF));
                data[i].pll[2] = std::byte((P1 & 0x00030000) >> 16);
                data[i].pll[3] = std::byte((P1 & 0x0000FF00) >> 8);
                data[i].pll[4] = std::byte((P1 & 0x000000FF));
                data[i].pll[5] = std::byte(((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
                data[i].pll[6] = std::byte((P2 & 0x0000FF00) >> 8);
                data[i].pll[7] = std::byte((P2 & 0x000000FF));
                
                
                P1 = 128ULL * divider - 512;
                P2 = 0;							// P2 = 0, P3 = 1 forces an integer value for the divider
                P3 = 1;
            
                uint8_t rDiv = 0;
                
                data[i].msynth[0] = std::byte((P3 & 0x0000FF00) >> 8);
                data[i].msynth[1] = std::byte((P3 & 0x000000FF));
                data[i].msynth[2] = std::byte(((P1 & 0x00030000) >> 16) | rDiv);
                data[i].msynth[3] = std::byte((P1 & 0x0000FF00) >> 8);
                data[i].msynth[4] = std::byte((P1 & 0x000000FF));
                data[i].msynth[5] = std::byte(((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
                data[i].msynth[6] = std::byte((P2 & 0x0000FF00) >> 8);
                data[i].msynth[7] = std::byte((P2 & 0x000000FF));
                
                i++;
            }
            return data;
        }();

        static inline constexpr auto setupChannelsU = []{
            std::array<SetupData, External::RC::channels.size()> data;
            
            const uint32_t maxPllf{900'000'000UL};
            const uint32_t xtalfreq{25'000'000UL};
            
            for(uint8_t i{0}; const External::RC::Channel c1 : External::RC::channels) {
                const uint32_t ff = c1.mFreq + 5'000;
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
                
                data[i].a = mult;
                data[i].b = num;
                data[i].c = denom;
                data[i].div = divider;
                
                uint32_t P1;					// PLL config register P1
                uint32_t P2;					// PLL config register P2
                uint32_t P3;					// PLL config register P3

                P1 = (uint32_t)(128 * ((double)num / (double)denom));
                P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
                P2 = (uint32_t)(128 * ((double)num / (double)denom));
                P2 = (uint32_t)(128 * num - denom * P2);
                P3 = denom;
                 
                data[i].pll[0] = std::byte((P3 & 0x0000FF00) >> 8);
                data[i].pll[1] = std::byte((P3 & 0x000000FF));
                data[i].pll[2] = std::byte((P1 & 0x00030000) >> 16);
                data[i].pll[3] = std::byte((P1 & 0x0000FF00) >> 8);
                data[i].pll[4] = std::byte((P1 & 0x000000FF));
                data[i].pll[5] = std::byte(((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
                data[i].pll[6] = std::byte((P2 & 0x0000FF00) >> 8);
                data[i].pll[7] = std::byte((P2 & 0x000000FF));
                
                
                P1 = 128ULL * divider - 512;
                P2 = 0;							// P2 = 0, P3 = 1 forces an integer value for the divider
                P3 = 1;
            
                uint8_t rDiv = 0;
                
                data[i].msynth[0] = std::byte((P3 & 0x0000FF00) >> 8);
                data[i].msynth[1] = std::byte((P3 & 0x000000FF));
                data[i].msynth[2] = std::byte(((P1 & 0x00030000) >> 16) | rDiv);
                data[i].msynth[3] = std::byte((P1 & 0x0000FF00) >> 8);
                data[i].msynth[4] = std::byte((P1 & 0x000000FF));
                data[i].msynth[5] = std::byte(((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
                data[i].msynth[6] = std::byte((P2 & 0x0000FF00) >> 8);
                data[i].msynth[7] = std::byte((P2 & 0x000000FF));
                
                i++;
            }
            return data;
        }();
        
                
        template<typename Bus, auto Adr>
        struct Clock {
            using bus = Bus;
            
            static_assert(bus::size() > 8);
            
            enum class State : uint8_t {Idle, ReadWait, 
                                        DefaultSetup1, DefaultSetup2, DefaultSetup3, DefaultSetup4, DefaultSetupComplete,
                                        SetupChannelStart, SetupChannelPLL, SetupChannelMSynth, SetupChannelReset, SetupChannelOutput, SetupChannelOutput2, SetupChannelComplete
                                       };
            
            
            static inline uint8_t iindex{};
            static inline uint8_t ichannel{};

            static inline uint8_t ipll{26};
            static inline uint8_t imsynth{42};
            static inline uint8_t ireg{16};
            static inline uint8_t iregValue{0x4f};
            
            static inline void periodic() {
                const auto oldState = mState;
                switch(mState) {
                case State::Idle:
                    break;
                case State::ReadWait:
                    break;
                case State::DefaultSetup1:
                    if (bus::write(Adr, {SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF})) {
                        mState = State::DefaultSetup2;
                        iindex = 0;
                    }
                    break;
                case State::DefaultSetup2:
                      if (bus::write(Adr, m_si5351_regs_15to92_149to170[iindex])) {
                          ++iindex;
                          if (iindex >= (sizeof(m_si5351_regs_15to92_149to170) / 2)) {
                              mState = State::DefaultSetup3;
                          }
                      }
                    break;
                case State::DefaultSetup3:
                    if (bus::write(Adr, {SI5351_REGISTER_177_PLL_RESET, 0xAC})) {
                        mState = State::DefaultSetup4;
                    }
                    break;
                case State::DefaultSetup4:
                    if (bus::write(Adr, {SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0x00})) {
                        mState = State::DefaultSetupComplete;
                    }
                    break;
                case State::DefaultSetupComplete:
                    break;
                case State::SetupChannelStart:
                    iindex = 0;
                    mState = State::SetupChannelPLL;
                    break;
                case State::SetupChannelPLL:
//                    if (bus::write(Adr, {std::byte(iindex + ipll), setupChannels[ichannel].pll[iindex]})) {
                    if (bus::write(Adr, {std::byte(iindex + ipll), actual.pll[iindex]})) {
                        ++iindex;
                        if (iindex >= SetupData::data_type::size()) {
                            mState = State::SetupChannelMSynth;
                            iindex = 0;
                        }
                    }
                    break;
                case State::SetupChannelMSynth:
//                    if (bus::write(Adr, {std::byte(iindex + imsynth), setupChannels[ichannel].msynth[iindex]})) {
                    if (bus::write(Adr, {std::byte(iindex + imsynth), actual.msynth[iindex]})) {
                        ++iindex;
                        if (iindex >= SetupData::data_type::size()) {
                            mState = State::SetupChannelReset;
                            iindex = 0;
                        }
                    }
                    break;
                case State::SetupChannelReset:
                    if (bus::write(Adr, {SI5351_REGISTER_177_PLL_RESET, 0xA0})) {
                        mState = State::SetupChannelOutput;
                    }
                    break;
                case State::SetupChannelOutput:
                    if (bus::write(Adr, {ireg, iregValue})) {
                        mState = State::SetupChannelOutput2;
                    }
                    break;
                case State::SetupChannelOutput2:
                    if (bus::write(Adr, {SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0x00})) {
                        mState = State::SetupChannelComplete;
                    }
                    break;
                case State::SetupChannelComplete:
                    break;
                }
            }

            static inline bool off() {
                return bus::write(Adr, {SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xff});
            }
            
            static inline bool on() {
                return bus::write(Adr, {SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0x00});
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
                if (o == 1) {
                    ipll = 34;
                    imsynth = 50;
                    ireg = 17;
                    iregValue = 0x6f;
                }
                else {
                    ipll = 26;
                    imsynth = 42;
                    ireg = 16;
                    iregValue = 0x4f;
                }
            }
            
            static inline bool setChannel(const uint8_t c) {
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
            static inline bool setChannelUpperFreq(const uint8_t c) {
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
