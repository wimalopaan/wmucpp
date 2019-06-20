#pragma once

#include "../hott.h"

namespace Hott {
    namespace Experimental {

        template<AVR::Concepts::ComponentNumber CNumber, 
                 template<typename CN, typename PA, typename ISR, typename RXL> typename Uart, 
                 etl::Concepts::NamedConstant Baud, 
                 typename BinaryMesgType,
                 typename Clock,
                 typename MCU = DefaultMcuType>
        
        struct Sensor final {
            static inline constexpr auto UartNumber = CNumber::value;
            enum class hott_state_t {Undefined = 0, BinaryStartRequest, AsciiStartRequest, BinaryWaitIdle, AsciiWaitIdle, BinaryReply, AsciiReply, NumberOfStates};
            
            static inline constexpr std::byte msg_code = code_from_type<BinaryMesgType>::value;
//                std::integral_constant<std::byte, msg_code>::_;
            
            struct ProtocollAdapter final {
                ProtocollAdapter() = delete;
                
                static bool process(std::byte c) {
                    hott_state_t oldstate = mState;
                    switch (mState) {
                    case hott_state_t::Undefined:
                        if (c == Hott::msg_start) {
                            mState = hott_state_t::BinaryStartRequest;
                        }
                        else if (c == Hott::ascii_msg_start) {
                            mState = hott_state_t::AsciiStartRequest;
                        }
                        break;
                    case hott_state_t::AsciiStartRequest:
                        if (compare<Nibble::Upper>(c, ascii_id(msg_code))) {
                            mAsciiReceived++;
                            mLastKey = Hott::key_t{c & 0x0f_B};
                            mState = hott_state_t::AsciiWaitIdle;
                        }
                        else {
                            mState = hott_state_t::Undefined;
                        }
                        break;
                    case hott_state_t::BinaryStartRequest:
                        if (c == binary_id(msg_code)) {
                            mBinaryReceived++;
                            mState = hott_state_t::BinaryWaitIdle;
                        }
                        else if (c == Hott::broadcast_code) {
                            mState = hott_state_t::BinaryWaitIdle;
                        }
                        else {
                            mState = hott_state_t::Undefined;
                        }
                        break;
                    case hott_state_t::BinaryWaitIdle:
                    case hott_state_t::AsciiWaitIdle:
                        ++mBytesReceivedInIdlePeriod;
                        break;
                    default:
                        assert(false);
                        break;
                    }
                    if (oldstate != mState) {
                        switch (mState) {
                        case hott_state_t::AsciiWaitIdle:
                        case hott_state_t::BinaryWaitIdle:
                            mWaitTicks = 0;
                            mBytesReceivedInIdlePeriod = 0;
                            break;
                        default:
                            break;
                        }
                    }
                    return true;
                }
            };
            
            using uart = Uart<AVR::Component::Usart<UartNumber>, ProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;
          
            inline static constexpr void init() {
                uart::template init<Baud>();
//                uart::_;
                mTextMsg.esc = ascii_id(msg_code);
                for(auto& l : text()) {
                    l.clear();
                }
            }
            
            inline static constexpr void periodic() {
                uart::periodic();
            }

            static inline constexpr auto intervall = Clock::intervall;
            static_assert(intervall == hottDelayBetweenBytes);

            static inline constexpr auto ticks_to_wait = hottDelayBeforeAnswer / intervall;
//            std::integral_constant<uint8_t, ticks_to_wait>::_;
            
            inline static constexpr void ratePeriodic() {
                hott_state_t oldstate = mState;
                switch(mState) {
                case hott_state_t::AsciiWaitIdle:
                    ++mWaitTicks;
                    if (mWaitTicks >= ticks_to_wait) {
                        if (mBytesReceivedInIdlePeriod == 0) {
                            mState = hott_state_t::AsciiReply;
                        }
                        else {
                            ++mCollisions;
                        }
                    }
                    break;
                case hott_state_t::BinaryWaitIdle:
                    ++mWaitTicks;
                    if (mWaitTicks >= ticks_to_wait) {
                        if (mBytesReceivedInIdlePeriod == 0) {
                            mState = hott_state_t::BinaryReply;
                        }
                        else {
                            ++mCollisions;
                        }
                    }
                    break;
                case hott_state_t::AsciiReply:
                    if (sendNextAsciiByte()) {
                        mState = hott_state_t::Undefined;
                    }
                    break;
                case hott_state_t::BinaryReply:
                    if (sendNextBinaryByte()) {
                        mState = hott_state_t::Undefined;
                    }
                    break;
                default:
                    break;
                }
                if (oldstate != mState) {
                    switch(mState) {
                    case hott_state_t::BinaryReply:
                    case hott_state_t::AsciiReply:
                        uart::template rxEnable<false>();
                        mByteIndex = 0;
                        break;
                    case hott_state_t::Undefined:
                        uart::template rxEnable<true>();
                        break;
                    default:
                        break;
                    }
                }
            }

            inline static constexpr auto key() {
                auto k = Hott::key_t::nokey;
                std::swap(mLastKey, k);
                return k;
            }
            
            inline static constexpr auto& text() {
                return mTextMsg.text;
            }

            inline static constexpr auto collisions() {
                return mCollisions;
            }
            
            inline static constexpr auto asciiPackages() {
                return mAsciiReceived;
            }
            inline static constexpr auto binaryPackages() {
                return mBinaryReceived;
            }
            inline static constexpr auto& data() {
                return mMsg;
            }
        private:
            static inline bool sendNextAsciiByte() {
                if (mByteIndex < (sizeof(mTextMsg) - 1)) {
                    /*constexpr */const std::byte* ptr = (const std::byte*) &mTextMsg;  
                    const std::byte value = ptr[mByteIndex];
                    mTextMsg.parity += std::to_integer<uint8_t>(value);
                    uart::put(value);
                    ++mByteIndex;
                    return false;
                }
                else {
                    uart::put(std::byte{mTextMsg.parity});
                    mTextMsg.parity = 0;
                    return true;
                }
            }
            static inline bool sendNextBinaryByte() {
                if (mByteIndex < (sizeof(mMsg) - 1)) {
                    /*constexpr */const std::byte* ptr = (const std::byte*) &mMsg;  
                    const std::byte value = ptr[mByteIndex];
                    mMsg.parity += std::to_integer<uint8_t>(value);
                    uart::put(value);
                    ++mByteIndex;
                    return false;
                }
                else {
                    uart::put(std::byte{mMsg.parity});
                    mMsg.parity = 0;
                    return true;
                }
            }
            
            static inline hott_state_t mState = hott_state_t::Undefined;
            
            inline static uint_ranged<uint8_t, 0, ticks_to_wait> mWaitTicks = 0;
            
            inline static BinaryMesgType mMsg;
            inline static Hott::TextMsg mTextMsg;
            
            inline static uint_ranged<uint8_t, 0, std::max(sizeof(mMsg), sizeof(mTextMsg))> mByteIndex = 0;
            
            inline static Hott::key_t mLastKey = Hott::key_t::nokey;
            
            inline static uint8_t mBytesReceivedInIdlePeriod = 0;
            inline static uint8_t mCollisions = 0;
            inline static uint8_t mAsciiReceived = 0;
            inline static uint8_t mBinaryReceived = 0;
        };
    }        
}

