#pragma once

#include <array>
#include <cstdint>

#include "outputs.h"
#include "pattern_config.h"

namespace Multiswitch {
    class Pattern {
    public:
        void configure(const External::Pattern::EEProm& config, std::array<Output, 8>& outputs) {
            stop(outputs);
            mConfig = config;
            mLength = 0;
            for (const uint8_t member : config.member) {
                if (member > 0 && member <= outputs.size()) mMembers[mLength++] = member - 1;
            }
        }

        void start(const uint64_t now, std::array<Output, 8>& outputs, const bool chain = false) {
            if (!mConfig.type || !mLength ||
                (mState != State::Idle && !(chain && mState == State::Wait))) return;
            mForward = mConfig.type != 2 && mConfig.type != 5;
            mPosition = mForward ? 0 : mLength - 1;
            activate(now, outputs);
        }

        void stop(std::array<Output, 8>& outputs) {
            if (mState == State::On) outputs[mMembers[mPosition]].set(false);
            mState = State::Idle;
        }

        bool periodic(const uint64_t now, std::array<Output, 8>& outputs) {
            if (mState == State::Idle || mState == State::Wait || now < mDeadline) return false;
            if (mState == State::On) {
                outputs[mMembers[mPosition]].set(false);
                mState = State::Off;
                mDeadline = now + uint64_t{mConfig.offTime} * 10'000;
                return false;
            }
            if (!advance()) {
                mState = State::Wait;
                return true;
            }
            activate(now, outputs);
            return false;
        }

    private:
        enum class State {Idle, On, Off, Wait};

        void activate(const uint64_t now, std::array<Output, 8>& outputs) {
            mState = State::On;
            outputs[mMembers[mPosition]].set(true);
            mDeadline = now + uint64_t{mConfig.onTime} * 10'000;
        }

        bool advance() {
            const bool single = mConfig.type >= 4;
            const bool bounce = mConfig.type == 3 || mConfig.type == 6;
            if (mLength == 1) return !single;
            if (mForward) {
                if (mPosition + 1 < mLength) ++mPosition;
                else if (bounce) {
                    mForward = false;
                    --mPosition;
                }
                else if (single) return false;
                else mPosition = 0;
            }
            else {
                if (mPosition > 0) --mPosition;
                else if (single) return false;
                else if (bounce) {
                    mForward = true;
                    ++mPosition;
                }
                else mPosition = mLength - 1;
            }
            return true;
        }

        External::Pattern::EEProm mConfig;
        std::array<uint8_t, 8> mMembers{};
        State mState = State::Idle;
        uint64_t mDeadline = 0;
        uint8_t mLength = 0;
        uint8_t mPosition = 0;
        bool mForward = true;
    };
}
