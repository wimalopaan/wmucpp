#pragma once

#include <type_traits>
#include <concepts>
#include <cstddef>
#include <functional>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/alternate.h"
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

        namespace Util {
            /* ======================================================================
             * The function calculates optimal timing of the I2C bus
             *
             * @PAram pclk1_hz    - PCLK frequency (Hz)
             * @PAram i2c_freq_hz - target I2C clock frequency (Hz)
             * @retvalue          - timing register value
             * ======================================================================*/
            uint32_t I2C_calculate_timing(const uint32_t pclk1_hz, const uint32_t i2c_freq_hz)
            {
                const uint32_t clock_period_ns = 1000000000 / pclk1_hz;
                const uint32_t i2c_period_ns   = 1000000000 / i2c_freq_hz;
                const uint32_t af_delay_ns     = 50; // Analog filter delay (ns)

                // Determine speed mode limits & digital filter delay (ns)
                uint32_t tlow_min, thigh_min, tsudat_min, dfn;

                if (i2c_freq_hz <= 100000) {
                    tlow_min   = i2c_period_ns * 470 >> 10;
                    thigh_min  = i2c_period_ns * 400 >> 10;
                    tsudat_min = 250;
                    dfn = 4;
                } else if (i2c_freq_hz <= 400000) {
                    tlow_min   = i2c_period_ns * 540 >> 10;
                    thigh_min  = i2c_period_ns * 250 >> 10;
                    tsudat_min = 100;
                    dfn = 2;
                }  else if (i2c_freq_hz <= 1000000) {
                    tlow_min   = i2c_period_ns * 520 >> 10;
                    thigh_min  = i2c_period_ns * 270 >> 10;
                    tsudat_min = 50;
                    dfn = 0;
                } else {
                    return 0;
                }

                const uint32_t filter_delay_ns = af_delay_ns + (dfn * clock_period_ns);
                const uint32_t sdadel_min_ns = filter_delay_ns;
                const uint32_t sdadel_max_ns = i2c_period_ns >> 2;
                const uint32_t scldel_min_ns = tsudat_min + filter_delay_ns;

                // Target period with 20% margin
                const uint32_t period_max = i2c_period_ns + (i2c_period_ns / 5);
                const uint32_t period_min = i2c_period_ns - (i2c_period_ns / 5);

                uint32_t tmp_presc  = 0;
                uint32_t tmp_scldel = 0;
                uint32_t tmp_sdadel = 0;
                uint32_t tmp_sclh   = 0;
                uint32_t tmp_scll   = 0;
                uint32_t tmp_error  = UINT32_MAX;

                // Binary search helper function for SCLH and SCLL
                for (uint32_t presc = 0; presc < 16; presc++) {
                    const uint32_t ti2cclk = clock_period_ns * (presc + 1);

                    for (uint32_t scldel = 0; scldel < 16; scldel++) {
                        const uint32_t tscldel = (scldel + 1) * ti2cclk;
                        if (tscldel < scldel_min_ns) continue;

                        for (uint32_t sdadel = 0; sdadel < 16; sdadel++) {
                            const uint32_t tsdadel = sdadel * ti2cclk;
                            if (tsdadel < sdadel_min_ns || tsdadel > sdadel_max_ns) continue;

                            const uint32_t tsync = filter_delay_ns + 2 * ti2cclk;

                            // Binary search for SCLH
                            uint32_t sclh_min = (thigh_min - filter_delay_ns + ti2cclk - 1) / ti2cclk - 1;
                            if (sclh_min >= 256) continue;
                            uint32_t sclh_max = 255;

                            while (sclh_min <= sclh_max) {
                                const uint32_t sclh = (sclh_min + sclh_max) >> 1;
                                const uint32_t tsclh = (sclh + 1) * ti2cclk;
                                const uint32_t thigh = tsclh + filter_delay_ns;

                                if (thigh < thigh_min) {
                                    sclh_min = sclh + 1;
                                    continue;
                                }

                                if (ti2cclk >= thigh) {
                                    sclh_min = sclh + 1;
                                    continue;
                                }

                                // Binary search for SCLL
                                uint32_t scll_min = (tlow_min - filter_delay_ns + ti2cclk - 1) / ti2cclk - 1;
                                if (scll_min >= 256) {
                                    sclh_min = sclh + 1;
                                    continue;
                                }
                                uint32_t scll_max = 255;

                                while (scll_min <= scll_max) {
                                    const uint32_t scll = (scll_min + scll_max) >> 1;
                                    const uint32_t tscll = (scll + 1) * ti2cclk;
                                    const uint32_t tlow = tscll + filter_delay_ns;

                                    if (tlow < tlow_min) {
                                        scll_min = scll + 1;
                                        continue;
                                    }

                                    if (ti2cclk >= (tlow - filter_delay_ns) / 4) {
                                        scll_min = scll + 1;
                                        continue;
                                    }

                                    const uint32_t ttotal = tscll + tsclh + 2 * tsync;

                                    if (ttotal < period_min) {
                                        scll_min = scll + 1;
                                        continue;
                                    }

                                    if (ttotal > period_max) {
                                        scll_max = scll - 1;
                                        continue;
                                    }

                                    // Calculate error
                                    const uint32_t error = (ttotal > i2c_period_ns)
                                        ? ttotal - i2c_period_ns
                                        : i2c_period_ns - ttotal;

                                    // Update if better timing found
                                    if (error < tmp_error) {
                                        tmp_error  = error;
                                        tmp_presc  = presc;
                                        tmp_scldel = scldel;
                                        tmp_sdadel = sdadel;
                                        tmp_sclh   = sclh;
                                        tmp_scll   = scll;
                                    }

                                    // Try to find better error by decreasing SCLL
                                    scll_max = scll - 1;
                                }
                                // Try to find better error by decreasing SCLH
                                sclh_max = sclh - 1;
                            }
                        }
                    }
                }

                if (tmp_error == UINT32_MAX) {
                    return 0;
                }

                return (tmp_presc << 28) | (tmp_scldel << 20) | (tmp_sdadel << 16) | (tmp_sclh << 8) | tmp_scll;
            }
        }

        namespace V2 {
            template<uint8_t N, typename Config, typename MCU = DefaultMcu> struct Master;
            template<uint8_t N, typename Config, typename MCU>
            requires (
                        ((N >= 1) && (N <= 2) && std::is_same_v<Stm32G030, MCU>) ||
                        ((N >= 1) && (N <= 3) && std::is_same_v<Stm32G0B1, MCU>) ||
                        ((N >= 1) && (N <= 3) && std::is_same_v<Stm32G431, MCU>)
                    )
            struct Master<N, Config, MCU> {
                static inline /*constexpr */ I2C_TypeDef* const mcuI2c = reinterpret_cast<I2C_TypeDef*>(Mcu::Stm::Address<Mcu::Components::I2C<N>>::value);
                static inline constexpr size_t size = Config::size;
                using sda_pin = Config::sda_pin;
                using scl_pin = Config::scl_pin;
                using component_t = Mcu::Components::I2C<N>;
                using debug = Config::debug;
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

                    static constexpr uint8_t sdaaf = Mcu::Stm::AlternateFunctions::mapper_v<sda_pin, Master, Mcu::Stm::AlternateFunctions::SDA>;
                    sda_pin::afunction(sdaaf);
                    static constexpr uint8_t sclaf = Mcu::Stm::AlternateFunctions::mapper_v<sda_pin, Master, Mcu::Stm::AlternateFunctions::SDA>;
                    scl_pin::afunction(sclaf);
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

                    // mcuI2c->TIMINGR  = 0x20705378; // CubeMx 100KHz@64MHz + Anafilter + Digfilter 0b0001
                    // mcuI2c->CR1 |= I2C_CR1_PE | (0b0001 << I2C_CR1_DNF_Pos);

                    mcuI2c->TIMINGR  = 0x10b075ae; // CubeMx 100KHz@64MHz + Anafilter + Digfilter 0b1111
                    mcuI2c->CR1 |= I2C_CR1_PE | (0b1111 << I2C_CR1_DNF_Pos);

                    static constexpr uint8_t sdaaf = Mcu::Stm::AlternateFunctions::mapper_v<sda_pin, Master, Mcu::Stm::AlternateFunctions::SDA>;
                    sda_pin::afunction(sdaaf);
                    static constexpr uint8_t sclaf = Mcu::Stm::AlternateFunctions::mapper_v<sda_pin, Master, Mcu::Stm::AlternateFunctions::SDA>;
                    scl_pin::afunction(sclaf);
                }
    #endif

                enum class State : uint8_t { Idle = 0,
                                             WriteAdress = 10, WriteWaitAdress, WriteData, WriteWaitData, WriteWaitComplete, WriteError,
                                             ScanStart = 20, ScanWaitBusy, ScanWaitAck, ScanNext, ScanEnd,
                                             ReadWriteAdress = 30, ReadAdressWait, ReadWriteCommand, ReadWriteComplete,
                                             ReadWriteAdress2 = 40, ReadAdressWait2, ReadDataWait, ReadData, ReadDataComplete, ReadError,
                                             ReEnable = 50
                                           };

                static inline void periodic() {
                    switch(mState) {
                    case State::Idle:
                        break;
                    case State::ReadWriteAdress:
                        mcuI2c->ICR = -1;
                        mcuI2c->CR2 = [] {
                            uint32_t cr2 = 0;
                            cr2 |= (1 << I2C_CR2_NBYTES_Pos);
                            cr2 |= ((mAddress << 1) << I2C_CR2_SADD_Pos);
                            cr2 |= I2C_CR2_AUTOEND;
                            cr2 |= I2C_CR2_START;
                            return cr2;
                        }();
                        mState = State::ReadAdressWait;
                        break;
                    case State::ReadAdressWait:
                    {
                        const uint32_t isr = mcuI2c->ISR;
                        if (isr & I2C_ISR_TXIS) {
                            mState = State::ReadWriteCommand;
                        }
                        if (isr & I2C_ISR_STOPF) {
                            mState = State::ReadWriteCommand;
                        }
                        if (isr & I2C_ISR_NACKF) {
                            mState = State::ReadError;
                        }
                    }
                        break;
                    case State::ReadWriteCommand:
                        mcuI2c->TXDR = (uint32_t)mData[0];
                        mState = State::ReadWriteComplete;
                        break;
                    case State::ReadWriteComplete:
                    {
                        const uint32_t isr = mcuI2c->ISR;
                        if (isr & I2C_ISR_TXIS) {
                            mState = State::ReadWriteAdress2;
                        }
                        if (isr & I2C_ISR_TC) {
                            mState = State::ReadWriteAdress2;
                        }
                        if (isr & I2C_ISR_STOPF) {
                            mState = State::ReadWriteAdress2;
                        }
                        if (isr & I2C_ISR_NACKF) {
                            mState = State::ReadError;
                        }
                    }
                        break;
                    case State::ReadWriteAdress2:
                        mcuI2c->CR2 = [] {
                            uint32_t cr2 = 0;
                            cr2 |= (mCount << I2C_CR2_NBYTES_Pos);
                            cr2 |= ((mAddress << 1) << I2C_CR2_SADD_Pos);
                            cr2 |= I2C_CR2_AUTOEND;
                            cr2 |= I2C_CR2_RD_WRN; // read
                            cr2 |= I2C_CR2_START;
                            return cr2;
                        }();
                        mState = State::ReadAdressWait2;
                        mIndex = 0;
                        break;
                    case State::ReadAdressWait2:
                    {
                        const uint32_t isr = mcuI2c->ISR;
                        if (isr & I2C_ISR_TXIS) {
                            mState = State::ReadDataWait;
                        }
                        if (isr & I2C_ISR_NACKF) {
                            mState = State::ReadError;
                        }
                        if (isr & I2C_ISR_RXNE) {
                            mState = State::ReadData;
                        }
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
                        mState = State::ReEnable;
                        break;
                    case State::ScanStart:
                        mcuI2c->CR2 = [] {
                            uint32_t cr2 = 0;
                            cr2 |= ((0 << I2C_CR2_NBYTES_Pos));
                            cr2 |= ((mScanSlaveAddress << 1) << I2C_CR2_SADD_Pos);
                            cr2 |= I2C_CR2_AUTOEND;
                            cr2 |= I2C_CR2_START;
                            cr2 |= I2C_CR2_STOP;
                            return cr2;
                        }();
                        mBusyCounter = 0;
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
                        IO::outl<debug>("I2C WA: ",  mAddress, " ", mCount);
                        mcuI2c->ICR = -1;
                        mcuI2c->CR2 = [] {
                            uint32_t cr2 = 0;
                            cr2 |= ((mCount << I2C_CR2_NBYTES_Pos));
                            cr2 |= ((mAddress << 1) << I2C_CR2_SADD_Pos);
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
                        IO::outl<debug>("I2C WWA");
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
                        IO::outl<debug>("I2C WD");
                        mcuI2c->TXDR = (uint32_t)mData[mIndex++];
                        if (mIndex == mCount) {
                            mState = State::WriteWaitComplete;
                        }
                        else {
                            mState = State::WriteWaitData;
                        }
                        break;
                    case State::WriteWaitData:
                    {
                        const uint32_t isr = mcuI2c->ISR;
                        if (isr & I2C_ISR_TXIS) {
                            mState = State::WriteData;
                        }
                        if (isr & I2C_ISR_STOPF) {
                            mState = State::WriteData;
                        }
                        if (isr & I2C_ISR_NACKF) {
                            mState = State::WriteError;
                        }
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
                        mState = State::ReEnable;
                        break;
                    case State::ReEnable:
                        mState = State::Idle;
                        break;
                    }
                }
                static inline uint8_t mBusyCounter = 0;
                static inline void ratePeriodic() {
                    switch(mState) {
                    case State::ScanWaitBusy:
                        if (!(mcuI2c->ISR & I2C_ISR_BUSY)) {
                            mState = State::ScanWaitAck;
                        }
                        else {
                            if (++mBusyCounter > 2) {
                                mState = State::ScanNext;
                            }
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
                    IO::outl<debug>("I2C write: ", adr.value);
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
                inline static bool write(const I2C::Address adr, const uint8_t data) {
                    IO::outl<debug>("I2C write: ", adr.value);
                    if (mState != State::Idle) {
                        return false;
                    }
                    mIndex = 0;
                    mCount = 1;
                    mErrors = 0;
                    mData[0] = std::byte{data};
                    mAddress = adr.value;
                    mState = State::WriteAdress;
                    return true;
                }
                inline static bool read(const I2C::Address adr, const std::byte command, const uint8_t length) {
                    IO::outl<debug>("I2C read: ", adr.value);
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
                    IO::outl<debug>("I2C readData");
                    mState = State::Idle;
                    return mData;
                }
                inline static bool readDataAvailable() {
                    IO::outl<debug>("I2C readDataAvail");
                    return mState == State::ReadDataComplete;
                }

                template<auto L, typename V>
                requires (L < size)
                inline static bool write(const I2C::Address adr, const V offset, const std::array<V, L>& data) {
                    IO::outl<debug>("I2C write array: ", adr.value);
                    if (mState != State::Idle) {
                        return false;
                    }
                    mIndex = 0;
                    mErrors = 0;
                    mData[0] = (std::byte)offset;
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
                static inline void clearErrors() {
                    mErrors = 0;
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

                static inline uint8_t mAddress{0};
                static inline uint8_t mCount{0};
                static inline uint8_t mIndex{0};
                static inline uint16_t mErrors{0};
                static inline State mState{State::Idle};
                static inline std::array<std::byte, size> mData{};
                static inline std::array<uint8_t, 128 / 8> mPresent{};
            };
        }
        
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
