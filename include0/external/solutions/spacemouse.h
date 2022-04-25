#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>

#include <etl/ranged.h>

namespace External {
    namespace SpaceMouse {
        using namespace etl;
        using namespace std;
        
        template<uint8_t N>
        class ProtocollAdapter final {
            enum class State : uint8_t {Undefined, Data, CheckSumL, CheckSumH, End};
            
            using message_t = std::array<std::byte, 12>;
            
            inline static constexpr uint16_t halfSpan = 360;
        public:
            inline static constexpr uint16_t midValue = 8192;
            inline static constexpr uint16_t maxValue = midValue + halfSpan;
            inline static constexpr uint16_t minValue = midValue - halfSpan;
            using value_t = etl::uint_ranged<uint16_t, minValue, maxValue>; 
        private:
            using axis_t = std::array<value_t, 6>;            
        public:
            inline static constexpr std::byte startSymbol = 0x96_B;
            inline static constexpr std::byte endSymbol = 0x8D_B;
            using index_t = etl::index_type_t<axis_t>;
            
            inline static bool process(const std::byte b) { 
                ++mBytes;
                switch (mState) {
                case State::Undefined:
                    if (b == startSymbol) {
                        mState = State::Data;
                        mIndex.setToBottom();
                        mCheckSum = std::to_integer(startSymbol);
                    }
                    break;
                case State::Data:
                    mRawValues[mIndex] = b;
                    mCheckSum += std::to_integer(b);
                    if (mIndex.isTop()) {
                        mState = State::CheckSumH;
                    }
                    else {
                        ++mIndex;
                    }
                    break;
                case State::CheckSumH:
                    mCheckSum &= 0x3fff;
                    mCheckSumP = std::to_integer(b) << 7;
                    mState = State::CheckSumL;
                    break;
                case State::CheckSumL:
                    mCheckSumP += std::to_integer(b);
                    if (mCheckSum == mCheckSumP) {
                        mState = State::End;
                    }
                    else {
                        mState = State::Undefined;                        
                    }
                    break;
                case State::End:
                    if (b == endSymbol) {
                        ++mPackages; 
                        decode();
                    }
                    else {
                        mState = State::Undefined;
                    }
                    break;
                }
                return true;
            }  
            inline static void ratePeriodic() {}
            
            inline static uint16_t packages() {
                return mPackages;
            }
            inline static uint16_t bytes() {
                return mBytes;
            }
            inline static void resetStats() {
                mBytes = mPackages = 0;
            }   
            inline static value_t axis(const index_t index) {
                return mAxis[index];
            }
        private:
            static inline uint16_t rawValue(const uint8_t axis) {
                return (std::to_integer(mRawValues[2 * axis]) << 7) + std::to_integer(mRawValues[2 * axis + 1]);
            }
            static inline void decode() {
                for(uint8_t i = 0; auto& a : mAxis) {
                    a.set(rawValue(i++));
                }
            }
            inline static State mState{State::Undefined};
            inline static uint16_t mBytes{};
            inline static uint16_t mPackages{};
            inline static uint16_t mCheckSum{};
            inline static uint16_t mCheckSumP{};
            
            inline static message_t mRawValues;
            inline static etl::index_type_t<message_t> mIndex;
            inline static axis_t mAxis;
        };
    }
}

