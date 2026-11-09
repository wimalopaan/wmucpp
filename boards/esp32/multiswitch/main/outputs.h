#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <string_view>

#include <output.h>
#include "eeprom.h"

namespace Multiswitch {
    class Output {
    public:
        void configure(const EEProm::Output& config) {
            const auto previousPwm = mConfig.pwm;
            const bool restart = previousPwm != config.pwm || mConfig.blink != config.blink;
            if (previousPwm != config.pwm || mConfig.pwmDuty != config.pwmDuty) {
                mDuty = config.pwm == 2 ? 0 : config.pwmDuty;
            }
            mConfig = config;
            if (previousPwm == 2 && config.pwm != 2) mOn = false;
            if (restart) {
                mPhase = Phase::Start;
            }
        }

        void set(const bool on) {
            if (mOn != on) {
                mOn = on;
                mPhase = Phase::Start;
            }
        }

        void duty(const uint8_t value) {
            mDuty = std::min(value, uint8_t{100});
        }

        void failsafe(const bool on) {
            set(on);
            if (mConfig.pwm == 2) {
                mDuty = on ? mConfig.pwmDuty : 0;
            }
        }

        uint8_t value(const uint64_t now, const EEProm& config) {
            if (mConfig.pwm == 2) {
                return mDuty;
            }
            if (!mOn) {
                return 0;
            }
            const uint8_t high = mConfig.pwm ? mDuty : 100;
            if (mConfig.blink == 0) {
                return high;
            }
            if (mConfig.blink == 1) {
                if (mPhase == Phase::Start) {
                    mStart = now;
                    mPhase = Phase::Pulse;
                }
                const uint64_t on = uint64_t{mConfig.blinkOnTime} * 50'000;
                const uint64_t off = uint64_t{mConfig.blinkOffTime} * 50'000;
                const uint64_t burst = on * (2 * std::max(mConfig.flashCount, uint8_t{1}) - 1);
                const uint64_t elapsed = (now - mStart) % (burst + off);
                return elapsed < burst && (elapsed / on) % 2 == 0 ? high : 0;
            }
            return morse(now, config) ? high : 0;
        }

    private:
        enum class Phase {Start, Pulse, SignGap, LetterGap, WordGap, Done};

        bool morse(const uint64_t now, const EEProm& config) {
            const auto& text = config.morse_texts[0];
            if (mPhase == Phase::Start) {
                mLetter = 0;
                mSign = 0;
                mDeadline = now;
                mPhase = Phase::LetterGap;
            }
            if (now < mDeadline || mPhase == Phase::Done) {
                return mPhase == Phase::Pulse;
            }
            if (mPhase == Phase::Pulse) {
                mPhase = Phase::SignGap;
                mDeadline = now + uint64_t{config.morse_gap} * 100'000;
                return false;
            }
            if (mPhase == Phase::SignGap) {
                ++mSign;
                if (mSign >= mCode.size()) {
                    ++mLetter;
                    mSign = 0;
                    mPhase = Phase::LetterGap;
                    mDeadline = now + uint64_t{config.morse_igap} * 100'000;
                    return false;
                }
            }
            if (mSign == 0) {
                while (mLetter < text.size() && text[mLetter]) {
                    const unsigned char c = text[mLetter];
                    if (c == ' ') {
                        ++mLetter;
                        mPhase = Phase::WordGap;
                        mDeadline = now + uint64_t{config.morse_wgap} * 100'000;
                        return false;
                    }
                    mCode = code(static_cast<char>(std::toupper(c)));
                    if (!mCode.empty()) {
                        break;
                    }
                    ++mLetter;
                }
                if (mLetter >= text.size() || !text[mLetter]) {
                    mOn = false;
                    mPhase = Phase::Done;
                    return false;
                }
            }
            mPhase = Phase::Pulse;
            mDeadline = now + uint64_t{mCode[mSign] == '-' ? config.morse_dah : config.morse_dit} * 100'000;
            return true;
        }

        static constexpr std::string_view code(const char c) {
            constexpr std::array<std::string_view, 26> letters{
                ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---",
                "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-",
                "..-", "...-", ".--", "-..-", "-.--", "--.."};
            constexpr std::array<std::string_view, 10> digits{
                "-----", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----."};
            if (c >= 'A' && c <= 'Z') return letters[c - 'A'];
            if (c >= '0' && c <= '9') return digits[c - '0'];
            switch (c) {
            case '.': return ".-.-.-";
            case ',': return "--..--";
            case ':': return "---...";
            case ';': return "-.-.-.";
            case '?': return "..--..";
            case '!': return "-.-.--";
            case '-': return "-....-";
            case '=': return "-...-";
            case '+': return ".-.-.";
            default: return {};
            }
        }

        EEProm::Output mConfig{};
        bool mOn = false;
        uint8_t mDuty = 1;
        Phase mPhase = Phase::Start;
        uint64_t mStart = 0;
        uint64_t mDeadline = 0;
        size_t mLetter = 0;
        size_t mSign = 0;
        std::string_view mCode;
    };
}
