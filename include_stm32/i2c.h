#pragma once

#pragma once

#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"

#include <type_traits>
#include <concepts>
#include <cstddef>
#include <functional>

namespace Mcu::Stm {
    using namespace Units::literals;
    namespace I2C {

        inline bool isTLC59108F(const uint8_t address7bit) {
            return (address7bit & 0x70) == 0x40;
        }
        inline bool isPCA9955B(const uint8_t address7bit) { // AD2 -> Gnd
            const uint8_t a = address7bit & 0x7f;
            return (a == 0x01) || (a == 0x0a) || (a == 0x2a) || (a == 0x32);
        }
        inline bool isSSD1306(const uint8_t address7bit) { // AD2 -> Gnd
            const uint8_t a = address7bit & 0x7f;
            return a == 0x3c;
        }

        struct Write{};
        struct Read{};
        
        struct Address {
            static constexpr uint8_t lowest = 0x08;
            static constexpr uint8_t highest = 0x77;
            
            explicit constexpr Address(uint8_t a) : value(a & 0x7f){} // narrowing
            
            using value_type = uint8_t; 
            const value_type value{};  
        };
        
        template<uint8_t N, size_t Size = 16, typename MCU = DefaultMcu> struct Master;
        
        template<uint8_t N, size_t Size, G4xx MCU>
        struct Master<N, Size, MCU> {
            static inline /*constexpr */ I2C_TypeDef* const mcuI2c = reinterpret_cast<I2C_TypeDef*>(Mcu::Stm::Address<Master<N, Size, MCU>>::value);
            static inline void init() {
                if constexpr(N == 1) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
                }
                else if constexpr(N == 2) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
                }
                else if constexpr(N == 3) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN;
                }
                else {
                    static_assert(false);
                }
                // mcuI2c->TIMINGR = 0x00A0A5FA; // CubeMX
                mcuI2c->TIMINGR = 0x30A0A7FB; // CubeMX 100KHz
                mcuI2c->CR1 |= I2C_CR1_PE;
            }
            
            enum class State : uint8_t { Idle,
                                         WriteAdress, WriteWaitAdress, WriteData, WriteWaitData, WriteWaitComplete, WriteError,
                                         ScanStart, ScanWaitBusy, ScanWaitAck, ScanNext, ScanEnd
                                         };
            
            static inline void periodic() {
                switch(mState) {
                case State::Idle:
                break;
                case State::ScanStart:
                {
                    MODIFY_REG(mcuI2c->CR2, I2C_CR2_NBYTES_Msk, (0 << I2C_CR2_NBYTES_Pos));
                    MODIFY_REG(mcuI2c->CR2, I2C_CR2_SADD_Msk, (mScanSlaveAddress << 1) << I2C_CR2_SADD_Pos);
                    mcuI2c->CR2 |= I2C_CR2_AUTOEND;
                    mcuI2c->CR2 &= ~I2C_CR2_ADD10;
                    mcuI2c->CR2 &= ~I2C_CR2_RD_WRN;
                    mcuI2c->CR2 |= I2C_CR2_START;
                    mcuI2c->CR2 |= I2C_CR2_STOP;
                    mState = State::ScanWaitBusy;
                }
                break;
                case State::ScanWaitBusy:
                    break;
                case State::ScanWaitAck:
                    if (mcuI2c->ISR & I2C_ISR_NACKF) {
                        mState = State::ScanNext;
                    }
                    else if (mcuI2c->ISR & I2C_ISR_TXIS) {
                        mCallBack(mScanSlaveAddress);
                        setPresent(Address{mScanSlaveAddress});
                        mState = State::ScanNext;
                    }
                break;
                case State::ScanNext:
                    if (mScanSlaveAddress >= Address::highest) {
                        mcuI2c->CR1 &= ~I2C_CR1_PE;
                        mState = State::ScanEnd;
                    }
                    else {
                        ++mScanSlaveAddress;
                        mState = State::ScanStart;
                    }
                break;
                case State::ScanEnd:
                    mCallBack = nullptr;
                    mcuI2c->CR1 |= I2C_CR1_PE;
                    mState = State::Idle;
                break;
                case State::WriteAdress:
                {
                    uint32_t temp = mcuI2c->CR2;
                    temp &= ~I2C_CR2_NBYTES_Msk;
                    temp &= ~I2C_CR2_SADD_Msk;
                    temp |= (mCount << I2C_CR2_NBYTES_Pos);
                    temp |= ((mAddress << 1) << I2C_CR2_SADD_Pos);
                    mcuI2c->CR2 = temp;

                    mcuI2c->CR2 |= I2C_CR2_AUTOEND;
                    mcuI2c->CR2 &= ~I2C_CR2_ADD10;
                    mcuI2c->CR2 &= ~I2C_CR2_RD_WRN;
                    
                    mcuI2c->CR2 |= I2C_CR2_START;
                    mcuI2c->CR2 &= ~I2C_CR2_STOP;
                    
                    mIndex = 0;
                    mState = State::WriteWaitAdress;
                }
                break;
                case State::WriteWaitAdress:
                    if (mcuI2c->ISR & I2C_ISR_TXIS) {
                        mState = State::WriteData;
                    }
                    if (mcuI2c->ISR & I2C_ISR_NACKF) {
                        mState = State::WriteError;    
                    }
                break;
                case State::WriteData:
                    mcuI2c->TXDR = (uint32_t)mData[mIndex++];
                    if (mIndex == mCount) {
                        mState = State::WriteWaitComplete;
                    }
                    else {
                        mState = State::WriteWaitData;
                    }
                break;
                case State::WriteWaitData:
                    if (mcuI2c->ISR & I2C_ISR_TXIS) {
                        mState = State::WriteData;
                    }
                    if (mcuI2c->ISR & I2C_ISR_NACKF) {
                        mState = State::WriteError;    
                    }
                break;
                case State::WriteWaitComplete:
                    if (mcuI2c->ISR & I2C_ISR_TC) {
                        mState = State::Idle;
                    }
                    else {
                        mState = State::WriteError;
                    }
                break;
                case State::WriteError:
                    ++mErrors;
                    mState = State::Idle;
                break;
                }
            }
            static inline void ratePeriodic() {
                switch(mState) {
                case State::ScanWaitBusy:
                    if (!(mcuI2c->ISR & I2C_ISR_BUSY)) {
                        mState = State::ScanWaitAck;
                    }
                    break;
                default:
                    break;
                }
            }
            inline static bool isIdle() {
                return mState == State::Idle;
            }

            inline static bool scan(void (*cb)(uint8_t)) {
                if (mState != State::Idle) {
                    return false;
                }
                mScanSlaveAddress = Address::lowest;
                mState = State::ScanStart;
                mCallBack = cb;
                return true;
            }
            
            inline static bool write(const I2C::Address adr, const std::pair<uint8_t, uint8_t>& data) {
                return write(adr, std::pair{std::byte{data.first}, std::byte{data.second}});
            }

            inline static bool write(const I2C::Address adr, const std::pair<std::byte, std::byte>& data) {
                if (mState != State::Idle) {
                    return false;
                }
                mIndex = 0;
                mCount = 2;
                mErrors = 0;
                mData[0] = data.first;
                mData[1] = data.second;
                mAddress = adr.value;
                mState = State::WriteAdress;
                return true;
            }       
            template<auto L>
            requires (L < Size)
            inline static bool write(const I2C::Address adr, const std::byte offset, const std::array<std::byte, L>& data) {
                if (mState != State::Idle) {
                    return false;
                }
                mIndex = 0;
                mErrors = 0;
                mData[0] = offset;
                const auto last = std::copy(std::begin(data), std::end(data), std::begin(mData) + 1);
                mCount = L + 1;
                mAddress = adr.value;
                mState = State::WriteAdress;
                return true;
            }
            static inline bool isPresent(const Address a) {
                const uint8_t adr = a.value & 0x7f;
                const uint8_t index = adr / 8;
                const uint8_t bit = adr % 8;
                return mPresent[index] & (1 << bit);
            }

        private:
            static inline void setPresent(const Address a) {
                const uint8_t index = a.value / 8;
                const uint8_t bit = a.value % 8;
                mPresent[index] |= (1 << bit);
            }
            static inline void resetPresent(const Address a) {
                const uint8_t index = a.value / 8;
                const uint8_t bit = a.value % 8;
                mPresent[index] &= ~(1 << bit);
            }
            static inline uint8_t mScanSlaveAddress{0};
            static inline void(*mCallBack)(uint8_t) = nullptr;

            static inline uint8_t mAddress{0};
            static inline uint8_t mCount{0};
            static inline uint8_t mIndex{0};
            static inline uint8_t mErrors{0};
            static inline State mState{State::Idle};
            static inline std::array<std::byte, Size> mData{};
            static inline std::array<uint8_t, 128 / 8> mPresent{};
        };
    }
    

    template<size_t S, G4xx MCU>
    struct Address<I2C::Master<1, S, MCU>> {
        static inline constexpr uintptr_t value = I2C1_BASE;        
    };
    template<size_t S, G4xx MCU>
    struct Address<I2C::Master<2, S, MCU>> {
        static inline constexpr uintptr_t value = I2C2_BASE;        
    };
    template<size_t S, G4xx MCU>
    struct Address<I2C::Master<3, S, MCU>> {
        static inline constexpr uintptr_t value = I2C3_BASE;
    };
}
