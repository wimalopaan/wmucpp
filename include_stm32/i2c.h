#pragma once

#include <type_traits>
#include <concepts>
#include <cstddef>
#include <functional>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"
#include "output.h"
#include "components.h"

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
            
            constexpr Address(const uint8_t a = lowest) : value(a & 0x7f){} // narrowing

            constexpr bool operator==(const Address& rhs) const {
                return value == rhs.value;
            }
            
            using value_type = uint8_t;
            value_type value{};
        };
        
        template<uint8_t N, size_t Size = 16, typename Debug = void, typename MCU = DefaultMcu> struct Master;
        
        template<uint8_t N, size_t Size, typename Debug, typename MCU>
        requires (
                    ((N >= 1) && (N <= 2) && std::is_same_v<Stm32G030, MCU>) ||
                    ((N >= 1) && (N <= 3) && std::is_same_v<Stm32G0B1, MCU>) ||
                    ((N >= 1) && (N <= 3) && std::is_same_v<Stm32G431, MCU>)
                )
        struct Master<N, Size, Debug, MCU> {
            static inline /*constexpr */ I2C_TypeDef* const mcuI2c = reinterpret_cast<I2C_TypeDef*>(Mcu::Stm::Address<Mcu::Components::I2C<N>>::value);
#ifdef STM32G4
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
#endif
#ifdef STM32G0
            static inline void init() {
                if constexpr(N == 1) {
                    RCC->APBENR1 |= RCC_APBENR1_I2C1EN;
                }
                else if constexpr(N == 2) {
                    RCC->APBENR1 |= RCC_APBENR1_I2C2EN;
                }
                else if constexpr(N == 3) {
                    RCC->APBENR1 |= RCC_APBENR1_I2C3EN;
                }
                else {
                    static_assert(false);
                }
                // mcuI2c->TIMINGR  = 0x10B17DB5; // CubeMx 100KHz@64MHz
                mcuI2c->TIMINGR  = 0x20705378; // CubeMx 100KHz@64MHz + Anafilter + Digfilter 0b0001
                mcuI2c->CR1 |= I2C_CR1_PE | I2C_CR1_ANFOFF | (0b0001 << I2C_CR1_DNF_Pos);
            }
#endif

            enum class State : uint8_t { Idle = 0,
                                         WriteAdress = 10, WriteWaitAdress, WriteData, WriteWaitData, WriteWaitComplete, WriteError,
                                         ScanStart = 20, ScanWaitBusy, ScanWaitAck, ScanNext, ScanEnd,
                                         ReadWriteAdress = 30, ReadAdressWait, ReadWriteCommand, ReadWriteComplete,
                                         ReadWriteAdress2 = 40, ReadAdressWait2, ReadDataWait, ReadData, ReadDataComplete, ReadError
                                       };
            
            static inline void periodic() {
                switch(mState) {
                case State::Idle:
                    break;
                case State::ReadWriteAdress:
                    mcuI2c->ICR = -1;
                    mcuI2c->CR2 = []{
                        uint32_t cr2 = 0;
                        MODIFY_REG(cr2, I2C_CR2_NBYTES_Msk, (1 << I2C_CR2_NBYTES_Pos));
                        MODIFY_REG(cr2, I2C_CR2_SADD_Msk, (mAddress << 1) << I2C_CR2_SADD_Pos);
                        cr2 |= I2C_CR2_AUTOEND;
                        cr2 |= I2C_CR2_START;
                        return cr2;
                    }();
                    mState = State::ReadAdressWait;
                    break;
                case State::ReadAdressWait:
                    if (mcuI2c->ISR & I2C_ISR_TXIS) {
                        mState = State::ReadWriteCommand;
                    }
                    if (mcuI2c->ISR & I2C_ISR_STOPF) {
                        mState = State::ReadWriteCommand;
                    }
                    if (mcuI2c->ISR & I2C_ISR_NACKF) {
                        mState = State::ReadError;
                    }
                    break;
                case State::ReadWriteCommand:
                    mcuI2c->TXDR = (uint32_t)mData[0];
                    mState = State::ReadWriteComplete;
                    break;
                case State::ReadWriteComplete:
                    if (mcuI2c->ISR & I2C_ISR_TXIS) {
                        mState = State::ReadWriteAdress2;
                    }
                    if (mcuI2c->ISR & I2C_ISR_TC) {
                        mState = State::ReadWriteAdress2;
                    }
                    if (mcuI2c->ISR & I2C_ISR_STOPF) {
                        mState = State::ReadWriteAdress2;
                    }
                    if (mcuI2c->ISR & I2C_ISR_NACKF) {
                        mState = State::ReadError;
                    }
                    break;
                case State::ReadWriteAdress2:
                    mcuI2c->CR2 = []{
                        uint32_t cr2 = 0;
                        MODIFY_REG(cr2, I2C_CR2_NBYTES_Msk, (mCount << I2C_CR2_NBYTES_Pos));
                        MODIFY_REG(cr2, I2C_CR2_SADD_Msk, (mAddress << 1) << I2C_CR2_SADD_Pos);
                        cr2 |= I2C_CR2_AUTOEND;
                        cr2 |= I2C_CR2_RD_WRN; // read
                        cr2 |= I2C_CR2_START;
                        return cr2;
                    }();
                    mState = State::ReadAdressWait2;
                    mIndex = 0;
                    break;
                case State::ReadAdressWait2:
                    if (mcuI2c->ISR & I2C_ISR_TXIS) {
                        mState = State::ReadDataWait;
                    }
                    if (mcuI2c->ISR & I2C_ISR_NACKF) {
                        mState = State::ReadError;
                    }
                    if (mcuI2c->ISR & I2C_ISR_RXNE) {
                        mState = State::ReadData;
                    }
                    break;
                case State::ReadDataWait:
                    if (mcuI2c->ISR & I2C_ISR_RXNE) {
                        mState = State::ReadData;
                    }
                    break;
                case State::ReadData:
                    mData[mIndex++] = (std::byte)mcuI2c->RXDR;
                    if (mIndex == mCount) {
                        // mcuI2c->CR2 |= I2C_CR2_STOP;
                        mState = State::ReadDataComplete;
                    }
                    else {
                        mState = State::ReadDataWait;
                    }
                    break;
                case State::ReadDataComplete:
                    // wait until read
                    break;
                case State::ReadError:
                    ++mErrors;
                    mcuI2c->ICR = -1;
                    mState = State::Idle;
                    break;
                case State::ScanStart:
                    mcuI2c->CR2 = []{
                        uint32_t cr2 = 0;
                        MODIFY_REG(cr2, I2C_CR2_NBYTES_Msk, (0 << I2C_CR2_NBYTES_Pos));
                        MODIFY_REG(cr2, I2C_CR2_SADD_Msk, (mScanSlaveAddress << 1) << I2C_CR2_SADD_Pos);
                        cr2 |= I2C_CR2_AUTOEND;
                        cr2 |= I2C_CR2_START;
                        cr2 |= I2C_CR2_STOP;
                        return cr2;
                    }();
                    mState = State::ScanWaitBusy;
                    break;
                case State::ScanWaitBusy:
                    break;
                case State::ScanWaitAck:
                    if (mcuI2c->ISR & I2C_ISR_NACKF) {
                        mcuI2c->ICR = I2C_ISR_NACKF;
                        mState = State::ScanNext;
                    }
                    else {
                    // else if (mcuI2c->ISR & I2C_ISR_TXIS) {
                        if (mCallBack) {
                            mCallBack(Address{mScanSlaveAddress});
                        }
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
                    // IO::outl<Debug>("I2C WA: ",  mAddress, " ", mCount);
                    mcuI2c->ICR = -1;
                    mcuI2c->CR2 = []{
                        uint32_t cr2 = 0;
                        MODIFY_REG(cr2, I2C_CR2_NBYTES_Msk, (mCount << I2C_CR2_NBYTES_Pos));
                        MODIFY_REG(cr2, I2C_CR2_SADD_Msk, (mAddress << 1) << I2C_CR2_SADD_Pos);
                        cr2 |= I2C_CR2_AUTOEND;
                        cr2 |= I2C_CR2_START;
                        return cr2;
                    }();
                    mIndex = 0;
                    mState = State::WriteWaitAdress;
                }
                    break;
                case State::WriteWaitAdress:
                {
                    // IO::outl<Debug>("I2C WWA");
                    mIsr = mcuI2c->ISR;
                    const uint32_t isr = mcuI2c->ISR;
                    if (isr & I2C_ISR_TXIS) {
                        mState = State::WriteData;
                    }
                    if (isr & I2C_ISR_NACKF) {
                        mState = State::WriteError;
                    }
                    if (isr & I2C_ISR_STOPF) {
                        mState = State::WriteData;
                    }
                }
                    break;
                case State::WriteData:
                    // IO::outl<Debug>("I2C WD");
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
                    if (mcuI2c->ISR & I2C_ISR_STOPF) {
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
                    mcuI2c->ICR = -1;
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
            inline static bool scan(void (* const cb)(Address)) {
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
                // IO::outl<Debug>("I2C write: ", adr.value);
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
            inline static bool read(const I2C::Address adr, const std::byte command, const uint8_t length) {
                // IO::outl<Debug>("I2C read: ", adr.value);
                if (mState != State::Idle) {
                    return false;
                }
                mData[0] = command;
                mIndex = 0;
                mCount = length;
                mErrors = 0;
                mAddress = adr.value;
                mState = State::ReadWriteAdress;
                return true;
            }
            inline static const auto& readData(){
                // IO::outl<Debug>("I2C readData");
                mState = State::Idle;
                return mData;
            }
            inline static bool readDataAvailable() {
                // IO::outl<Debug>("I2C readDataAvail");
                return mState == State::ReadDataComplete;
            }

            template<auto L, typename V>
            requires (L < Size)
            inline static bool write(const I2C::Address adr, const V offset, const std::array<V, L>& data) {
                // IO::outl<Debug>("I2C write array: ", adr.value);
                if (mState != State::Idle) {
                    return false;
                }
                mIndex = 0;
                mErrors = 0;
                mData[0] = (std::byte)offset;
                // const auto last = std::copy(std::begin(data), std::end(data), std::begin(mData) + 1);
                // std::copy(std::begin(data), std::end(data), std::begin(mData) + 1);
                std::copy(std::begin(data), std::end(data), (V*)(&mData[0] + 1));
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
            static inline uint16_t errors() {
                return mErrors;
            }
            // private:
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
            static inline void(*mCallBack)(Address) = nullptr;

            static inline uint32_t mIsr;

            static inline uint8_t mAddress{0};
            static inline uint8_t mCount{0};
            static inline uint8_t mIndex{0};
            static inline uint16_t mErrors{0};
            static inline State mState{State::Idle};
            static inline std::array<std::byte, Size> mData{};
            static inline std::array<uint8_t, 128 / 8> mPresent{};
        };
    }
    
    template<>
    struct Address<Mcu::Components::I2C<1>> {
        static inline constexpr uintptr_t value = I2C1_BASE;
    };
    template<>
    struct Address<Mcu::Components::I2C<2>> {
        static inline constexpr uintptr_t value = I2C2_BASE;
    };
#ifdef I2C3_BASE
    template<>
    struct Address<Mcu::Components::I2C<3>> {
        static inline constexpr uintptr_t value = I2C3_BASE;
    };
#endif
}
