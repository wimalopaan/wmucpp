#pragma once

#include <std/chrono>
#include <etl/types.h>

namespace External {
    namespace Graupner {
        template<typename Ppm, uint8_t N, typename MCU = DefaultMcuType>
        struct MultiSwitch2N {
            enum class State : uint8_t {UnDefined, GotSync1, GotSync2};

            enum class SwState : uint8_t {Off, On, Blink1 = On, Steady, Blink2};

            using gpior_t = typename MCU::Gpior; 
            static inline constexpr auto flags = AVR::getBaseAddr<gpior_t, 1>;
            static inline constexpr auto enableMask = 0x01_B;

            
            static inline constexpr uint16_t thresh_up = Ppm::medium + (Ppm::span / 4);
            static inline constexpr uint16_t thresh_down = Ppm::medium - (Ppm::span / 4);

            static inline constexpr uint8_t numberOfSwitches = 2 * N;
            static inline constexpr uint8_t numberOfPropChs  = N;
            
            using sw_indext_t = etl::uint_ranged<uint8_t, 0, numberOfSwitches>;
            using p_indext_t  = etl::uint_ranged<uint8_t, 0, numberOfPropChs>;
            
            using value_type = Ppm::value_type;
            
            inline static void enable(const bool en = true) {
                if (en) {
                    flags()->data = flags()->data | enableMask;
                }
                else {
                    flags()->data = flags()->data & ~enableMask;
                }
            }            
            inline static constexpr uint8_t size() {
                return numberOfSwitches;
            }
            
            inline static void init() {
                Ppm::init();
                enable(true);
            }
            inline static void periodic() {
                Ppm::onCapture([]{
                    const auto v = Ppm::value(); 
                    if (const auto vi = v.toInt(); v) {
                        switch(mState) {
                        case State::UnDefined:
                            if (vi >= Ppm::ppmMax) {
                                mState = State::GotSync1;
                            }
                            break;
                        case State::GotSync1:
                            if (vi >= Ppm::ppmMax) {
                                mState = State::GotSync2;
                                index.setToBottom();
                            }
                            else {
                                mState = State::UnDefined;
                            }
                            break;
                        case State::GotSync2:
                            if (vi < Ppm::ppmMax) {
                                pValues[index] = vi;
                                if (std::any(flags()->data & enableMask)) {
                                    if (vi >= thresh_up) {
                                        swStates[toSwitchIndex(index, true)] = SwState::On;
                                        swStates[toSwitchIndex(index, false)] = SwState::Off;
                                    }
                                    else if (vi <= thresh_down) {
                                        swStates[toSwitchIndex(index, false)] = SwState::On;
                                        swStates[toSwitchIndex(index, true)] = SwState::Off;
                                    }
                                    else {
                                        swStates[toSwitchIndex(index, true)] = SwState::Off;
                                        swStates[toSwitchIndex(index, false)] = SwState::Off;
                                    }
                                }                            
                            }
                            else {
                                mState = State::GotSync1;
                            }
                            if (index.isTop()) {
                                mState = State::UnDefined;
                            }
                            else {
                                ++index;
                            }
                            break;
                        }
                    }
                });
            }
            
            static inline value_type ppm(const p_indext_t i = p_indext_t{0}) {
                return pValues[i];
            }
            
            static inline const auto& switches() {
                return swStates;
            }
            inline static void reset() {
            }
            
            
//        private:
            static inline sw_indext_t toSwitchIndex(const p_indext_t pi, const bool b){
                if (b) {
                    return sw_indext_t(2 * pi.toInt());
                }
                else {
                    return sw_indext_t(2 * pi.toInt() + 1);
                }
            }
            static inline std::array<value_type, numberOfPropChs> pValues{};
            static inline std::array<SwState, numberOfSwitches> swStates{};
            static inline p_indext_t index;
            static inline State mState{State::UnDefined};
        };

        template<typename Ppm, uint8_t N>
        struct MultiSwitch {
            enum class State : uint8_t {UnDefined, GotSync1, GotSync2};
            enum class SwState : uint8_t {Off, Up, Down};
            inline static constexpr uint8_t size() {
                return N;
            }
            inline static void init() {
                Ppm::init();
            }
            inline static void periodic() {
                Ppm::onCapture([]{
                    const auto v = Ppm::value(); 
                    if (!v) return;
                    switch(mState) {
                    case State::UnDefined:
                        if (v >= Ppm::ppmMax) {
                            mState = State::GotSync1;
                        }
                        break;
                    case State::GotSync1:
                        if (v >= Ppm::ppmMax) {
                            mState = State::GotSync2;
                            index.setToBottom();
                        }
                        else {
                            mState = State::UnDefined;
                        }
                        break;
                    case State::GotSync2:
                        if (v < Ppm::ppmMax) {
                            if (v >= thresh_up) {
                                swStates[index] = SwState::Up;
                            }
                            else if (v <= thresh_down) {
                                swStates[index] = SwState::Down;
                            }
                            else {
                                swStates[index] = SwState::Off;
                            }
                        }
                        else {
                            mState = State::GotSync1;
                        }
                        if (index.isTop()) {
                            mState = State::UnDefined;
                        }
                        else {
                            ++index;
                        }
                        break;
                    }
                });
            }
//        private:
            static inline std::array<SwState, N> swStates{};
            static inline constexpr uint16_t thresh_up = Ppm::medium + (Ppm::span / 4);
            static inline constexpr uint16_t thresh_down = Ppm::medium - (Ppm::span / 4);
            static inline etl::uint_ranged<uint8_t, 0, (N - 1)> index;
            static inline State mState{State::UnDefined};
        };
    }
}

