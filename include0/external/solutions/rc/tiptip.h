#pragma once

#include <std/chrono>
#include <etl/types.h>

namespace External {
    namespace Ppm {
        template<typename Ppm, uint8_t N, typename MCU = DefaultMcuType>
        struct MultiSwitch {
            using gpior_t = typename MCU::Gpior; 
            static inline constexpr auto flags = AVR::getBaseAddr<gpior_t, 1>;
            static inline constexpr uint8_t enableMask = 0x01;

            enum class State : uint8_t {UnDefined, CountUp, CountUpPause, ToggleUp, CountDown, CountDownPause, ToggleDown};
            enum class SwState : uint8_t {Off, On, Blink1 = On, Steady, Blink2};
            
            using index_t = etl::uint_ranged<uint8_t, 0, ((N / 2) - 1)>;
            
            inline static constexpr uint8_t size() {
                return N;
            }
            inline static void init() {
                Ppm::init();
                enable(true);
            }
            inline static void reset() {
                for(auto& s : swStates) {
                    s = SwState::Off;
                }
            }
            inline static auto ppm() {
                return Ppm::value();
            }
            inline static void enable(const bool en = true) {
                if (en) {
                    flags()->data = flags()->data | enableMask;
                }
                else {
                    flags()->data = flags()->data & ~enableMask;
                }
            }            
            inline static void periodic() {
                if (flags()->data & enableMask) {
                    Ppm::onCapture([]{
                        const auto v = Ppm::value(); 
                        if (!v) return;
                        switch(mState) {
                        case State::UnDefined:
                            if (isUp(v)) {
                                mState = State::CountUp;
                                count.setToBottom();
                                index.setToBottom();
                            }
                            if (isDown(v)) {
                                mState = State::CountDown;
                                count.setToBottom();
                                index.setToBottom();
                            }
                            break;
                        case State::CountUp:
                            if (isUp(v)) {
                                ++count;
                                if (count > setCycles) {
                                    toggle1(index);
                                    mState = State::ToggleUp;
                                }
                            }
                            else {
                                if (isNeutral(v)) {
                                    if (count > stepCycles) {
                                        mState = State::CountUpPause;
                                        count.setToBottom();
                                    }
                                    else {
                                        mState = State::UnDefined;
                                    }
                                }
                            }
                            break;
                        case State::ToggleUp:
                            if (isNeutral(v)) {
                                mState = State::UnDefined;
                            }
                            break;
                        case State::CountUpPause:
                            if (isNeutral(v)) {
                                ++count;
                                if (count > resetCycles) {
                                    mState = State::UnDefined;
                                }
                            }
                            else {
                                if (isUp(v)) {
                                    if (count > stepCycles) {
                                        mState = State::CountUp;
                                        count.setToBottom();
                                        ++index;
                                    }
                                    else {
                                        mState = State::UnDefined;
                                    }
                                }
                            }
                            break;
                        case State::CountDown:
                            if (isDown(v)) {
                                ++count;
                                if (count > setCycles) {
                                    toggle2(index);
                                    mState = State::ToggleDown;
                                }
                            }
                            else {
                                if (isNeutral(v)) {
                                    if (count > stepCycles) {
                                        mState = State::CountDownPause;
                                        count.setToBottom();
                                    }
                                    else {
                                        mState = State::UnDefined;
                                    }
                                }
                            }
                            break;
                        case State::ToggleDown:
                            if (isNeutral(v)) {
                                mState = State::UnDefined;
                            }
                            break;
                        case State::CountDownPause:
                            if (isNeutral(v)) {
                                ++count;
                                if (count > resetCycles) {
                                    mState = State::UnDefined;
                                }
                            }
                            else {
                                if (isDown(v)) {
                                    if (count > stepCycles) {
                                        mState = State::CountDown;
                                        count.setToBottom();
                                        ++index;
                                    }
                                    else {
                                        mState = State::UnDefined;
                                    }
                                }
                            }
                            break;
                        }
                    });
                }
            }
            static inline const auto& switches() {
                return swStates;
            }
        private:
            static inline etl::uint_ranged<uint8_t, 0, 200> count;
            
            static inline constexpr uint8_t stepCycles{10};
            static inline constexpr uint8_t setCycles{50};
            static inline constexpr uint8_t resetCycles{50};
            
            static inline std::array<SwState, N> swStates{};
            static inline constexpr uint16_t hysterese = (Ppm::span / 16);
            static inline constexpr uint16_t thresh_up = Ppm::medium + (Ppm::span / 4) + hysterese;
            static inline constexpr uint16_t thresh_down = Ppm::medium - (Ppm::span / 4) - hysterese;
            static inline constexpr uint16_t thresh_neutral_low  = Ppm::medium - (Ppm::span / 4) - hysterese;
            static inline constexpr uint16_t thresh_neutral_high = Ppm::medium + (Ppm::span / 4) + hysterese;
            static inline index_t index;
            static inline State mState{State::UnDefined};
            
            static inline void toggle1(const index_t i) {
                if (swStates[i] == SwState::Off) {
                    swStates[i] = SwState::On;
                }
                else {
                    swStates[i] = SwState::Off;
                }
            }
            static inline void toggle2(const index_t k) {
                const auto i = k + (N/2);
                if (swStates[i] == SwState::Off) {
                    swStates[i] = SwState::On;
                }
                else {
                    swStates[i] = SwState::Off;
                }
            }
            static inline bool isNeutral(const auto v) {
                const auto vi = v.toInt();
                return (vi >= thresh_neutral_low) && (vi <= thresh_neutral_high);
            }
            static inline bool isUp(const auto v) {
                return (v.toInt() >= thresh_up);
            }
            static inline bool isDown(const auto v) {
                return (v.toInt() <= thresh_down);
            }
        };
    }
}
