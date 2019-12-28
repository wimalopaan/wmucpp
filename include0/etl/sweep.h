#pragma once

#include <cstdint>
#include <utility>

#include "ranged.h"

namespace etl {
    template<etl::Unsigned T, auto L, auto R, auto D>
    struct Sweep final {
        static_assert(L >= 0);
        static_assert(R >= 0);
        static_assert(D > 0);
        static_assert(R > L);
        
        using value_type = etl::uint_ranged<T, L, R>;
        
        enum class Mode : uint8_t {LeftToRight, RightToLeft, LeftToRightCirclar, RightToLeftCircular};
        enum class State : uint8_t {UpR, UpLR, UpL, DownL, DownLR, DownR};
        
        inline constexpr explicit Sweep(Mode m) : mMode{m} {
            if ((mMode == Mode::LeftToRight) || (mMode == Mode::LeftToRightCirclar)) {
                mState = State::UpR;
                mLeft = L;
                mRight = L;
            }
            else if ((mMode == Mode::RightToLeft) || (mMode == Mode::RightToLeftCircular)) {
                mState = State::DownL;
                mLeft = R;
                mRight = R;
            }
        }
        
        inline constexpr std::pair<value_type, value_type> next() {
            switch(mState) {
            case State::UpR:
                if ((mRight - mLeft) >= D) {
                    mState = State::UpLR;
                }
                else {
                    ++mRight;
                }
                break;
            case State::UpLR:
                if (mRight.isTop()) {
                    mState = State::UpL;
                }
                else {
                    ++mRight;
                    ++mLeft;
                }
                break;
            case State::UpL:
                if (mLeft.isTop()) {
                    if (mMode == Mode::LeftToRight) {
                        mState = State::UpR;
                        mLeft = mRight = L;
                    }
                    else {
                        mState = State::DownL;
                    }
                }
                else {
                    ++mLeft;
                }
                break;
            case State::DownL:
                if ((mRight - mLeft) >= D) {
                    mState = State::DownLR;                                                               
                }                            
                else {
                    --mLeft;
                }
                break;
            case State::DownLR:
                if (mLeft.isBottom()) {
                    mState = State::DownR;
                }
                else {
                    --mLeft;
                    --mRight;
                }
                break;
            case State::DownR:
                if (mRight.isBottom()) {
                    if (mMode == Mode::RightToLeft) {
                        mState = State::DownL;
                        mLeft = mRight = R;
                    }
                    else {
                        mState = State::UpR;
                    }
                }
                else {
                    --mRight;
                }
                break;
            }
            return {mLeft, mRight};
        }
    private:
        State mState{State::UpR};
        Mode mMode{Mode::LeftToRight};
        value_type mLeft{};
        value_type mRight{};
    };
}
