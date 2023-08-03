#pragma once

#include <cstdint>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>

#include "algorithm.h"

namespace RC {
    namespace Protokoll {
        namespace Null {
            template<typename Buf = void>
            class Adapter final {
            public:
                using buffer_t = Buf;
                using value_type = void;
                Adapter() = delete;
                static inline constexpr bool process(const std::byte) {
                    return false;
                }
                static inline void ratePeriodic() {}
            };
        }
        namespace IBus {
            struct CheckSum final {
                inline void reset() {
                    mSum = std::numeric_limits<uint16_t>::max();
                }
                inline std::byte operator+=(const std::byte b) {
                    mSum -= static_cast<uint8_t>(b);
                    return b;
                }
                inline std::byte highByte() const {
                    return std::byte(mSum >> 8);
                }
                inline std::byte lowByte() const {
                    return std::byte(mSum);
                }
                inline void highByte(const std::byte hb) {
                    mH = hb;
                }
                inline void lowByte(const std::byte lb){
                    mL = lb;
                }
                inline explicit operator bool() const {
                    return (etl::nth_byte<0>(mSum) == mL) && (etl::nth_byte<1>(mSum) == mH);
                }
            private:
                std::byte mH{};
                std::byte mL{};
                uint16_t mSum = std::numeric_limits<uint16_t>::max();
            };

            template<auto N = 0, typename Dbg = void>
            struct Adapter {
                // AFHDS2A: 14 channels
                // there should be an update to 16 / 18 channels
                enum class State : uint8_t {Undefined, GotStart20, Data, CheckL, CheckH};
                
                using value_type = uint16_t;            
                using channel_t = uint8_t;
//                using value_type = etl::uint_ranged_NaN<uint16_t, 988, 2011>;            
//                using channel_t = etl::uint_ranged_NaN<uint8_t, 0, 17>;
                
                static inline value_type value(const channel_t chi) {
//                    if (const uint8_t chi = ch.toInt(); ch) {
                        if (chi < 14) {
                            const std::byte h = (*inactive)[2 * chi + 1] & std::byte{0x0f};
                            const std::byte l = (*inactive)[2 * chi];
                            const uint16_t  v = (uint16_t(h) << 8) + uint8_t(l);
                            if ((v >= 988) && (v <= 2011)) {
//                            if ((v >= value_type::Lower) && (v <= value_type::Upper)) {
                                return value_type{v};
                            }
                            else {
                                return {};
                            }
                        }
                        else if (chi < 18) {
                            const std::byte h1 = (*inactive)[6 * (chi - 14) + 1] & std::byte{0xf0};
                            const std::byte h2 = (*inactive)[6 * (chi - 14) + 3] & std::byte{0xf0};
                            const std::byte h3 = (*inactive)[6 * (chi - 14) + 5] & std::byte{0xf0};
                            const uint16_t v = (uint8_t(h1) >> 4) + uint8_t(h2) + (uint16_t(h3) << 4);
                            if ((v >= 988) && (v <= 2011)) {
//                            if ((v >= value_type::Lower) && (v <= value_type::Upper)) {
                                return value_type{v};
                            }
                            else {
                                return {};
                            }
                        }
//                    }            
                    return value_type{};
                }
                static inline value_type valueMapped(const channel_t ch) {
                    return value(ch);
                }
                
                static inline bool process(const std::byte b) {
                    switch(mState) {
                    case State::Undefined:
                        csum.reset();
                        if (b == std::byte{0x20}) {
                            csum += b;
                            mState = State::GotStart20;
                        }
                        break;
                    case State::GotStart20:
                        if (b == std::byte{0x40}) {
                            csum += b;
                            mState = State::Data;
//                            mIndex.setToBottom();
                            mIndex = 0;
                        }
                        else {
                            mState = State::Undefined;
                        }
                        break;
                    case State::Data:
                        (*active)[mIndex] = b;
                        csum += b;
                        if (mIndex == 28) {
//                        if (mIndex.isTop()) {
                            mState = State::CheckL;
                        }
                        else {
                            ++mIndex;
                        }
                        break;
                    case State::CheckL:
                        csum.lowByte(b);
                        mState = State::CheckH;
                        break;
                    case State::CheckH:
                        csum.highByte(b);
                        mState = State::Undefined;
                        if (csum) {
                            ++mPackagesCounter;
                            if constexpr (!std::is_same_v<Dbg, void>) {
                                Dbg::toggle();
                            }
                            using std::swap;
                            swap(active, inactive);
                        }
                        break;
                    }
                    return true;
                }
                
                inline static void ratePeriodic() {
                }
                
                inline static void resetStats() {
                    mPackagesCounter = 0;
                }
                inline static uint16_t packages() {
                    return mPackagesCounter;
                }
            private:
                using MesgType = std::array<std::byte, 28>;
                
                inline static CheckSum csum;
                inline static State mState{State::Undefined};
                inline static MesgType mData0; // 0x20, 0x40 , 28 Bytes, checkH, checkL
                inline static MesgType mData1; // 0x20, 0x40 , 28 Bytes, checkH, checkL
                inline static MesgType* active = &mData0;
                inline static MesgType* inactive = &mData1;
                inline static uint8_t mIndex;
//                inline static etl::index_type_t<MesgType> mIndex;
                inline static uint16_t mPackagesCounter{};
            };
            
        }
        
        namespace SBus {
            
        }
        namespace Hott {
            
        }
    }    
}
