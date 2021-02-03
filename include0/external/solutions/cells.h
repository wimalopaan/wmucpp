#pragma once

#include <etl/ranged.h>
#include <etl/fixedvector.h>

namespace External {
    
    template<auto N, typename CallBack, typename CheckSum, typename MCU = DefaultMcuType>
    struct CellPA {
        inline static constexpr std::byte startByte{0xa5};
        
        enum class State : uint8_t {Init, AwaitLength, AwaitDataL, AwaitDataH, AwaitCSLow, AwaitCSHigh};
        inline static bool process(const std::byte b) {
            static uint16_t v{};
            static CheckSum cs;
            switch(mState) {
            case State::Init:
                cs.reset();
                if (b == startByte) {
                    cs += b;
                    mState = State::AwaitLength;
                    mLength.setToBottom();
                }
                break;
            case State::AwaitLength:
                cs += b;
                mLength.set(static_cast<uint8_t>(b));
                if ((mLength != 0) && (mLength <= mValues.capacity)) {
                    mState = State::AwaitDataL;
                    mValues.clear();
                    mValues.reserve(mLength);
                    mIndex.setToBottom();
                }
                else {
                    mState = State::Init;
                }
                break;
            case State::AwaitDataL:
                cs += b;
                --mLength;
                v = static_cast<uint16_t>(b);
                mState = State::AwaitDataH;
                break;
            case State::AwaitDataH:
                cs += b;
                v |= (static_cast<uint16_t>(b) << 8);
                mValues[mIndex] = v;
                ++mIndex;
                if (mLength.isBottom()) {
                    mState = State::AwaitCSLow;
                }
                else {
                    mState = State::AwaitDataL;
                }
                break;
            case State::AwaitCSLow:
                cs.lowByte(b);
                mState = State::AwaitCSHigh;
                break;
            case State::AwaitCSHigh:
                cs.highByte(b);
                if (cs) {
                    if constexpr(!std::is_same_v<CallBack, void>) {
                        CallBack::copy(mValues);                
                    }
                }
                mState = State::Init;
                break;
            }
            return true;
        }
        inline static void ratePeriodic() {}
    private:
        inline static constexpr uint8_t mSize = 16;
        inline static etl::FixedVector<uint16_t, mSize> mValues{};
        inline static etl::uint_ranged<uint8_t, 0, mSize - 1> mLength{};
        inline static etl::uint_ranged<uint8_t, 0, mSize - 1> mIndex{};
        inline static State mState{State::Init};
    };
    
}
