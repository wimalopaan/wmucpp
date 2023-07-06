#pragma once

#pragma once

#include "mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu_traits.h"

#include <type_traits>
#include <concepts>
#include <cstddef>

namespace Mcu::Stm {
    using namespace Units::literals;

    namespace I2C {
        struct Write{};
        struct Read{};
        
        struct Address {
            static constexpr uint8_t lowest = 0x08;
            static constexpr uint8_t highest = 0x77;
            
            explicit constexpr Address(uint8_t a) : value{a}{}
            
//            using value_type = etl::uint_ranged<uint8_t, lowest, highest>; 
            using value_type = uint8_t; 
            const value_type value{};  
        };
        
        template<uint8_t N, typename MCU = void> struct Master;
        
        template<uint8_t N, G4xx MCU>
        struct Master<N, MCU> {
            static inline /*constexpr */ I2C_TypeDef* const mcuI2c = reinterpret_cast<I2C_TypeDef*>(Mcu::Stm::Address<Master<N, MCU>>::value);
            static inline void init() {
                if constexpr(N == 1) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
                }
                if constexpr(N == 2) {
                    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
                }
                else {
                    static_assert(false);
                }
                mcuI2c->TIMINGR = 0x00A0A5FA; // CubeMX
                mcuI2c->CR1 |= I2C_CR1_PE;
            }        
            
            enum class State : uint8_t {Idle, WriteAdress, WriteWaitAdress, WriteData, WriteWaitData, WriteWaitComplete, WriteError};
            
            static inline void periodic() {
                const auto oldState = mState;
                switch(mState) {
                case State::Idle:
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
        private:
            static inline uint8_t mAddress{0};
            static inline uint8_t mCount{0};
            static inline uint8_t mIndex{0};
            static inline uint8_t mErrors{0};
            static inline State mState{State::Idle};
            static inline std::array<std::byte, 16> mData;
        };
    }
    

    template<G4xx MCU>
    struct Address<I2C::Master<1, MCU>> {
        static inline constexpr uintptr_t value = I2C1_BASE;        
    };
    template<G4xx MCU>
    struct Address<I2C::Master<2, MCU>> {
        static inline constexpr uintptr_t value = I2C2_BASE;        
    };
}
