#pragma once

#include "rc.h"

namespace RC::SPort {
    using namespace etl::literals;
    using namespace Units::literals;
    using namespace std::literals::chrono_literals;
    
    struct CheckSum {
        void reset() {
            mValue = 0;
        }
        void operator+=(const std::byte b) {
            mValue += uint8_t(b);
            mValue += mValue >> 8;
            mValue &= 0x00ff;
        }
        std::byte value() const {
            return etl::nth_byte<0>(0xff - mValue);
        }
    private:
        uint16_t mValue{};
    };
    
    struct ByteStuff {
        enum class State : uint8_t {Normal, Stuff};
        auto decode(const std::byte b) {
            switch(mState) {
            case State::Normal:
                if (b == 0x7d_B) {
                    mState = State::Stuff;
                    return std::pair{b, false};
                }
                else {
                    return std::pair{b, true};
                }
            break;
            case State::Stuff:
                mState = State::Normal;
                if (b == 0x5d_B) {
                    return std::pair{0x7d_B, true};
                }
                else if (b == 0x5e_B) {
                    return std::pair{0x7e_B, true};
                }
            break;
            }
            return std::pair{b, false};
        }
    private:
        State mState{State::Normal};
    };
    
    namespace Master {
        
        template<uint8_t N, typename Timer, typename MCU = DefaultMcu>
        struct ProtocolAdapter {
            enum class State : uint8_t {Idle, WaitForResonse,
                                       AppIDLow, AppIDHigh, Value, Crc};

            static inline External::Tick<Timer> waitTicks{8ms};
            
            static inline void startWait() {
                if (mState == State::Idle) {
                    mStateTick.reset();
                    mState = State::WaitForResonse;
                }
            }
            static inline void stopWait() {
                mState = State::Idle;
            }
            
            static inline void process(const std::byte b1) {
                static CheckSum csum;
                static ByteStuff bstuff;
                const auto [b, ok] = bstuff.decode(b1);
                if (!ok) return;
                switch(mState) {
                case State::Idle:
                break;
                case State::WaitForResonse:
                    if (b == 0x10_B) {
                        csum.reset();
                        csum += b;
                        mState = State::AppIDLow;
                    }
                break;
                case State::AppIDLow:
                    csum += b;
                    mAppId = uint16_t(b);
                    mState = State::AppIDHigh;
                break;
                case State::AppIDHigh:
                    csum += b;
                    mAppId += uint16_t(b) << 8;
                    mState = State::Value;
                    mValueByteCounter = 0;
                    mValue = 0;
                break;
                case State::Value:
                    mValue += uint32_t(b) << (mValueByteCounter * 8);
                    ++mValueByteCounter;
                    if (mValueByteCounter >= 4) {
                        mState = State::Crc;
                    }
                break;
                case State::Crc:
                break;
                }
            }  
            
            static inline void ratePeriodic() {
                const auto oldState = mState;
                switch(mState) {
                case State::Idle:
                break;
                case State::WaitForResonse:
                    mStateTick.on(waitTicks, []{
                        mState = State::Idle;
                    });                    
                break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                }
            }
        private:
            static inline uint16_t mAppId{};
            static inline uint32_t mValue{};
            static inline uint8_t mValueByteCounter{};
            static inline External::Tick<Timer> mStateTick;
            static inline State mState{State::Idle};
        };
        
        template<typename UART, typename Timer, typename MCU = DefaultMcu>
        struct Fsm {
            using uart = UART;
            using pa_t = uart::pa_t;
            
            enum class State : uint8_t {SendRequenst, WaitComplete, Wait};
            static inline External::Tick<Timer> waitTicks{12ms};

            static inline void periodic() {
                switch(mState) {
                case State::WaitComplete:
                    if (uart::isIdle()) {
                        pa_t::startWait();
                        uart::template rxEnable<true>();
                        mState = State::Wait;
                    }
                break;
                case State::Wait:
                break;
                case State::SendRequenst:
                break;
                }
            }
            
            static inline void ratePeriodic() {
                ++mStateTick;
                switch(mState) {
                case State::SendRequenst:
                    uart::template rxEnable<false>();
                    uart::put(0x7e_B);
                    uart::put(std::byte(RC::Protokoll::SPort::sensor_ids[1]));
                    mState = State::WaitComplete;
                break;
                case State::WaitComplete:
                break;
                case State::Wait:
                    mStateTick.on(waitTicks, []{
                        pa_t::stopWait();
                        mState = State::SendRequenst; 
                    });
                break;
                }
            }
        private:    
            static inline External::Tick<Timer> mStateTick;
            static inline State mState{State::SendRequenst};
        };
    }
}

