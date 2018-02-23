#pragma once

#include <cstdint>
#include <cstddef>
#include <mcu/ports.h>
#include <util/bits.h>

namespace HX711 {
    namespace Mode {
        struct A_Gain128 {
            inline static constexpr uint8_t pulses = 25;
        };
        struct A_Gain64 {
            inline static constexpr uint8_t pulses = 27;
        };
        struct B_Gain32 {
            inline static constexpr uint8_t pulses = 26;
        };
    }
    
    template<bool>
    struct UseDelay;
    template<>
    struct UseDelay<true> : std::true_type {};
    template<>
    struct UseDelay<false> : std::false_type {};
    
    template<typename ClockPin, typename DataPin, typename Mode = Mode::A_Gain128, typename ValueType = uint8_t, ::Util::NamedFlag useDelay = UseDelay<true>>
    class Sensor final {
        enum class State : uint8_t {Undefined, Busy, Pulse, Reading, SetMode1, SetMode2};
    public:
        inline static constexpr uint8_t maxBits = 24;
        inline static constexpr uint8_t numberOfValueBits = std::min(Util::numberOfBits<ValueType>(), maxBits);
        inline static constexpr ValueType max = (1 << (numberOfValueBits - 1));
        typedef ClockPin clock;
        typedef DataPin  data;
        typedef ValueType value_type;
        
        inline static void init() {
            clock::template dir<AVR::Output>(); 
            clock::low();
            data::template dir<AVR::Input>(); 
        }
        
        inline static void periodic() {
            switch (mState) {
            case State::Undefined:
                if (data::isHigh()) {
                    mState = State::Busy;
                }
                break;
            case State::Busy:
                if (!data::isHigh()) {
                    mState = State::Pulse;
                    mCount = 0;
                    mValue = 0;
                }
                break;
            case State::Pulse:
                clock::high();
                mState = State::Reading;
                if constexpr(useDelay::value) {
                    Util::delay(1_us);
                }
                clock::low();
                break;
            case State::Reading:
                ++mCount;
                if (mCount <= numberOfValueBits) {
                    mValue <<= 1;
                    if (data::isHigh()) {
                        mValue |= 0x01;
                    }
                }
                if (mCount >= 24) {
                    if (mValue> max) {
                        mValue = ~(mValue - 1);
                    }
                    mState = State::SetMode1;
                }
                else {
                    mState = State::Pulse;
                }
                break;
            case State::SetMode1:
                clock::high();
                mState = State::SetMode2;
                if constexpr(useDelay::value) {
                    Util::delay(1_us);
                }
                clock::low();
                break;
            case State::SetMode2:
                ++mCount;
                if (mCount >= Mode::pulses) {
                    mState = State::Undefined;
                }
                else {
                    mState = State::SetMode1;
                }
                break;
            }
        }
        inline static value_type value() {
            return mValue;
        }
    private:
        inline static uint8_t mCount = 0;
        inline static ValueType mValue = 0;
        inline static State mState = State::Undefined;
    };
    
}

