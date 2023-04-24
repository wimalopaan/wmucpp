#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <chrono>

#include <etl/ranged.h>
#include <etl/algorithm.h>

#include <external/solutions/tick.h>
#include <external/ibus/ibus2.h>

#include <mcu/internals/usart.h>

namespace External {
    namespace OpenTx {
        namespace TelemetryMirror {
            using namespace std::literals::chrono;
            using namespace External::Units::literals;
            
            namespace FlySky2A {
                template<auto N, typename Timer, typename Dbg = void>
                struct ProtocollAdapter {
                    enum class State : uint8_t {Undefined, Start, Length, Type, Instance, Value};
    
                    static inline constexpr std::byte start_byte = 0x06_B;
                    static inline constexpr std::byte length_byte = 0x1D_B;
                    
                    inline static constexpr uint8_t maxAngleInstances = 4;
                    
                    using angle_index_type = etl::uint_ranged<uint8_t, 0, maxAngleInstances - 1>;
                    
                    static inline uint16_t angle(const angle_index_type i) {
                        return mAngles[i];
                    }
                    static inline uint16_t rssi() {
                        return mRssi;
                    }
                    static inline uint16_t snr() {
                        return mSnr;
                    }
                    static inline uint16_t noise() {
                        return mNoise;
                    }
    
                    static_assert(Timer::frequency >= 1000_Hz);
                    
                    static inline uint8_t tc{0};
                    static inline void ratePeriodic() {
                        if (++tc > 5) {
                            mState = State::Undefined;
                        }
                    }
                    
                    static inline bool process(const std::byte b) {
                        tc = 0;
                        switch(mState) {
                        case State::Undefined:
                            if (b == start_byte) {
                                mState = State::Start;
                            }
                            break;
                        case State::Start:
                            if (b == length_byte) {
                                mState = State::Length;
                            }
                            else {
                                mState = State::Undefined;
                            }
                            break;
                        case State::Length:
                            if (const IBus2::Type::type t{std::to_integer(b)}; t != IBus2::Type::type::UNKNOWN) {
                                mActualType = t;
                                mState = State::Type;                                
                            } 
                            else {
                                mActualType = IBus2::Type::type::UNKNOWN;
                                mState = State::Undefined;
                            }
                            break;
                        case State::Type:
                            if (const uint8_t instance(etl::to_integer(b)); instance < maxAngleInstances) {
                                index.set<etl::RangeCheck<false>>(instance);
                                mState = State::Instance;
                            }
                            else {
                                index.setNaN();
                            }
                            break;
                        case State::Instance:
                            if (index) {
                                mActualValue = uint8_t(b);
                            }
                            mState = State::Value;
                            break;
                        case State::Value:
                            if (index) {
                                mActualValue |= (uint16_t(b) << 8);
                                if (mActualType == IBus2::Type::type::RX_RSSI) {
                                    mRssi = 135 - mActualValue;
                                }
                                else if (mActualType == IBus2::Type::type::RX_SNR) {
                                    mSnr = mActualValue;
                                }
                                else if (mActualType == IBus2::Type::type::RX_NOISE) {
                                    mNoise = 135 - mActualValue;
                                }
                                else if (mActualType == IBus2::Type::type::ANGLE) {
                                    mAngles[index.toInt()] = mActualValue;
                                }
                            }
                            ++mPackages;
                            mState = State::Length;
                            mActualValue = 0;
                            break;
                        }
                        return true;
                    }
                    inline static uint16_t packages() {
                        return mPackages;
                    }
                    inline static void resetStats() {
                        mPackages = 0;
                    }
                private:
                    inline static IBus2::Type::type mActualType{IBus2::Type::type::UNKNOWN};
                    inline static uint16_t mActualValue{0};
                    inline static State mState{State::Undefined};
                    inline static uint16_t mPackages{};
                    
                    inline static etl::uint_ranged_NaN<uint8_t, 0, maxAngleInstances - 1> index;
                    inline static std::array<uint16_t, maxAngleInstances> mAngles{};
                    
                    inline static uint16_t mRssi{0};
                    inline static uint16_t mSnr{0};
                    inline static uint16_t mNoise{0};
                };
                
            }
            
            namespace FrSky {
                
            }
            
            
        }

    }
}
