#pragma once

#include <cstring>

#include "etl/fixedvector.h"
#include "etl/stackstring.h"

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "pwm.h"
#include "usart.h"
#include "si5351.h"
#include "i2c.h"
#include "timer.h"
#include "opamp.h"
#include "adc.h"
#include "dac.h"
#include "clock.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "gpio.h"
#include "tick.h"
#include "meta.h"
#include "rc/rc.h"
#include "rc/crsf.h"
#include "rc/sport.h"
#include "rc/roboremo.h"
#include "rc/escape.h"
#include "rc/sbus2.h"
#include "rc/vesc.h"
#include "bluetooth/hc05.h"
#include "motor/bdc.h"

#include "cppm_gen.h"
#include "pulse_form.h"
#include "robo_cli.h"
#include "pcal6408.h"

struct Var01;
struct Var02;
struct Var03;

using namespace Mcu::Stm;
using namespace std::literals::chrono_literals;

namespace Obsolete {
    struct BtCallback;
}

template<typename HW, typename Config, typename MCU = void>
struct Devices;

template<typename Config>
struct Devices<Var02, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using mcuconfig = Config::mcuconfig;

    using clock = mcuconfig::clock;
    using trace = mcuconfig::trace;

    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;

    using data = Config::data;
    using crsfcallback = Config::crsfcallback;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    // PB11 CRSF TX
    using crsftx = Mcu::Stm::Pin<gpiob, 11, MCU>;
    // PB10 CRSF RX
    using crsfrx = Mcu::Stm::Pin<gpiob, 10, MCU>;

    // Usart 3: CRSF
    struct CrsfAdapterConfig;
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfAdapterConfig, MCU>;
    using crsf    = Mcu::Stm::Uart<3, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;

    struct CrsfAdapterConfig {
        using out = crsf_out;
        using callback = Config::crsfcallback;
        using debug = trace;
    };

    // PA8 Led
    using led = Mcu::Stm::Pin<gpioa, 8, MCU>;
    struct LedCallback {
        static inline void on(const bool on) {
            if (on) {
                led::set();
            }
            else {
                led::reset();
            }
        }
    };

    // PB5 BT Pwr
    using btPwr = Mcu::Stm::Pin<gpiob, 5, MCU>;

    // Usart1: BT
    struct PassThru {
        static inline void start() {
        }
        static inline void stop() {
        }
        static inline void put(std::byte) {
        }
    };
    using passthru = PassThru;
    struct SendBack {
        static inline void put(std::byte) {
        }
    };
    using sendback = SendBack;

    using atbuffer = External::AT::Response<16>;
    using btcallback = Config::btcallback;
    using btPa = External::RoboCli::ProtocollAdapter<0, systemTimer, passthru, sendback, btcallback, 16, atbuffer>;
    using btUsart = Mcu::Stm::Uart<1, btPa, 2048, char, clock, MCU>;
    using bttx = Mcu::Stm::Pin<gpioa, 10, MCU>;
    using btrx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    // Usart2: ext serial
    using extrx = Mcu::Stm::Pin<gpiob, 4, MCU>;
    using exttx = Mcu::Stm::Pin<gpiob, 3, MCU>;
    using extpa = RC::Protokoll::Hott::SumDV3::Servo<0>;
    using extUsart = Mcu::Stm::Uart<2, extpa, 16, char, clock, MCU>;

    // PB13 HF Pwr
    using hfPwr = Mcu::Stm::Pin<gpiob, 13, MCU>;

    // PA6 FreqSel Tim3-Ch1 AF(2)
    using fSel = Mcu::Stm::Pin<gpioa, 6, MCU>;

    // I2C1
    using sda1 = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using scl1 = Mcu::Stm::Pin<gpioa, 15, MCU>;

    using i2c1 = Mcu::Stm::I2C::Master<1, 16, MCU>;
    static inline constexpr Mcu::Stm::I2C::Address si5351{0x60};
    using si = External::SI5351::Clock<i2c1, si5351, 867>;
    // using si = External::SI5351::Clock<i2c1, si5351, 0>;

    static inline constexpr Mcu::Stm::I2C::Address pcaAdr0{0x20};
    static inline constexpr Mcu::Stm::I2C::Address pcaAdr1{0x21};
    using pca0 = External::PCAL6408<i2c1, pcaAdr1, systemTimer>;
    using pca1 = External::PCAL6408<i2c1, pcaAdr0, systemTimer>;

    // PA12 Drehspul; Tim4-Ch2, AF(10)
    using ds = Mcu::Stm::Pin<gpioa, 12, MCU>;
    // PA11 Buzzer; Tim4-Ch1, AF(10)
    using buzz = Mcu::Stm::Pin<gpioa, 11, MCU>;
    // Timer 4
    using pwm = Mcu::Stm::V2::Pwm::Simple<4, clock, MCU>;

    struct BuzzerCallback {
        static inline void on(const bool on) {
            if (on) {
                buzz::afunction(10);
            }
            else {
                buzz::template dir<Mcu::Output>();
            }
        }
    };

    struct DsCallback {
        static inline constexpr float max = pwm::period;
        static inline void setPercent(const uint8_t p) {
            if (p > 100) return;
            const uint16_t duty = (max * p) / 100.0f;
            pwm::duty2(duty);
        }
    };

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    using adcDmaChannel1 = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaChannel2 = Mcu::Stm::Dma::Channel<dma1, 2, MCU>;
    using cppmDmaChannel = Mcu::Stm::Dma::Channel<dma1, 3, MCU>;
    using dacDmaChannel = Mcu::Stm::Dma::Channel<dma1, 4, MCU>;

    using adcDmaStorage1 = std::array<volatile uint16_t, 7>;
    using adcDmaStorage2 = std::array<volatile uint16_t, 1>;
    using adc1 = Mcu::Stm::V3::Adc<1, Meta::NList<1, 2, 3, 4, 5, 10, 12>, pwm, adcDmaChannel1, adcDmaStorage1, void, MCU>;
    using adc2 = Mcu::Stm::V3::Adc<2, Meta::NList<12>, pwm, adcDmaChannel2, adcDmaStorage2, Meta::List<EndOfSequence>, MCU>;
    using an1 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using an2 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using an3 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using an4 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using an5 = Mcu::Stm::Pin<gpiob, 14, MCU>;
    using an10 = Mcu::Stm::Pin<gpiof, 0, MCU>;
    using vtSense = Mcu::Stm::Pin<gpiob, 1, MCU>; // adc1 in12
    using vinSense = Mcu::Stm::Pin<gpiob, 2, MCU>; // adc2 in12

    // Timer 3
    using cppm = Mcu::Stm::Cppm::Generator<3, cppmDmaChannel, clock, MCU>;

    using dac = Mcu::Stm::Dac<1, MCU>;
    // Timer 6
    using pulse = Mcu::Stm::Cppm::RollOnOff<6, dacDmaChannel, dac, clock, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();

        led::template dir<Mcu::Output>();
        btPwr::template dir<Mcu::Output>();
        hfPwr::template dir<Mcu::Output>();

        fSel::template dir<Mcu::Output>();
        fSel::set();
        cppm::init();

        sda1::openDrain();
        scl1::openDrain();
        sda1::afunction(4);
        scl1::afunction(4);

        i2c1::init();

        an1::analog();
        an2::analog();
        an3::analog();
        an4::analog();
        an5::analog();
        an10::analog();
        vtSense::analog();
        vinSense::analog();

        adc1::init();
        adc2::init();

        pwm::init();
        pwm::frequency(400);
        pwm::duty1(50); // buzzer

        DsCallback::setPercent(0);
        ds::afunction(10);

        BuzzerCallback::on(false);

        dac::init();
        pulse::init();

        btUsart::init();
        btUsart::baud(115200);
        bttx::afunction(7);
        btrx::afunction(7);

        crsf::init();
        crsf::baud(420'000);
        crsftx::afunction(7);
        crsftx::pullup<true>();
        crsfrx::afunction(7);
    }
};

template<typename Config>
struct Devices<Var01, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    // using data = Config::data;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    // PB11 CRSF TX
    using crsftx = Mcu::Stm::Pin<gpiob, 11, MCU>;
    // using tp0 = crsftx;
    // PB10 CRSF RX
    using crsfrx = Mcu::Stm::Pin<gpiob, 10, MCU>;
    // using tp1 = crsfrx;

    // Usart 3: CRSF
    struct CrsfCallback;
    using crsf_pa = RC::Protokoll::Crsf::V1::Adapter<0, CrsfCallback, trace, MCU>;
    using crsf    = Mcu::Stm::Uart<3, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;

    struct CrsfCallback {
        using out = crsf_out;
        using Param_t = RC::Protokoll::Crsf::Parameter;
        using PType = RC::Protokoll::Crsf::Parameter::Type;

        static inline void setParameter(const uint8_t index, const uint8_t value) {
            IO::outl<trace>("SetP adr: ", index, " v: ", value);
            if ((index >= 1) && (index <= params.size())) {
                params[index - 1].mValue = value;
                mLastChangedParameter = index;
            }
        }
        static inline RC::Protokoll::Crsf::Parameter parameter(const uint8_t index) {
            if ((index >= 1) && (index <= params.size())) {
                return params[index - 1];
            }
            return {};
        }
        static inline bool isCommand(const uint8_t index) {
            if ((index >= 1) && (index <= params.size())) {
                return params[index - 1].mType == PType::Command;
            }
            return false;
        }
        static inline const char* name() {
            return mName;
        }
        static inline uint32_t serialNumber() {
            return mSerialNumber;
        }
        static inline uint32_t hwVersion() {
            return mHWVersion;
        }
        static inline uint32_t swVersion() {
            return mSWVersion;
        }
        static inline uint8_t numberOfParameters() {
            return params.size();
        }
        static inline uint8_t protocolVersion() {
            return 0;
        }
        static inline void whenParameterChanged(auto f) {
            if (mLastChangedParameter > 0) {
                f(mLastChangedParameter);
                mLastChangedParameter = 0;
            }
        }
        template<auto L>
        static inline void command(const std::array<uint8_t, L>& payload) {
            if (payload[0] == (uint8_t)RC::Protokoll::Crsf::Address::Controller) {
                if (payload[2] == (uint8_t)RC::Protokoll::Crsf::CommandType::CC) {
                    if (payload[3] == (uint8_t)RC::Protokoll::Crsf::CcCommand::SetAltData) {
                        IO::outl<trace>("AData: ", payload[4], " ", payload[5]);
                        mLastChangedAltData = (uint8_t)payload[4];
                        mLastChangedAltValue = (uint8_t)payload[5];
                    }
                }
            }
        }
        static inline void whenAlternateDataChanged(auto f) {
            if (mLastChangedAltData < 0xff) {
                f(mLastChangedAltData, mLastChangedAltValue);
                mLastChangedAltData = 0xff;
            }
        }
    private:
        static inline uint8_t mLastChangedAltData{};
        static inline uint8_t mLastChangedAltValue{};
        static inline uint8_t mLastChangedParameter{};
        static inline constexpr uint32_t mSerialNumber{1234};
        static inline constexpr uint32_t mHWVersion{1};
        static inline constexpr uint32_t mSWVersion{1};
        static inline constexpr auto mVersionString = [](){
            std::array<char, 16> s{};
            auto [ptr, e] = std::to_chars(std::begin(s), std::end(s), mHWVersion);
            *ptr++ = ':';
            std::to_chars(ptr, std::end(s), mSWVersion);
            return s;
        }();
        static inline constexpr const char* const mName = "VarioProp";
        static inline etl::FixedVector<Param_t, 64> params {
            Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]},
            Param_t{0, PType::Sel, "RF", "Off;On", 0, 0, 1},
            Param_t{0, PType::Sel, "Override", "Off;On", 0, 0, 1},
            Param_t{0, PType::Sel, "Bluetooth", "Off;On", 0, 0, 1},
        };
    };
    template<typename Out>
    struct CrsfTelemetry {
        using out = Out;
        static inline constexpr External::Tick<systemTimer> telemTicks{5ms};

        enum class State : uint8_t {Gps, Batt, Temp, Rpm};

        static inline void ratePeriodic() {
            (++mTelemTick).on(telemTicks, []{
                switch(mState) {
                case State::Gps:
                    out::data(RC::Protokoll::Crsf::Type::Gps, mGps); // crsf gps data
                    mState = State::Batt;
                    break;
                case State::Batt:
                    out::data(RC::Protokoll::Crsf::Type::Battery, mBatt);
                    mState = State::Temp;
                    break;
                case State::Temp:
                    out::data(RC::Protokoll::Crsf::Type::Temp1, mTemp);
                    mState = State::Rpm;
                    break;
                case State::Rpm:
                    out::data(RC::Protokoll::Crsf::Type::Rpm1, mRpm);
                    mState = State::Batt;
                    break;
                }
            });
        }
        //    private:
        static inline State mState{State::Batt};
        static inline std::array<std::byte, 4> mGps;
        static inline std::array<std::byte, 8> mBatt; //volts amps mAh percent
        static inline std::array<std::byte, 2> mTemp;
        static inline std::array<std::byte, 2> mRpm;
        static inline External::Tick<systemTimer> mTelemTick;
    };
    using crsfTelemetry = CrsfTelemetry<crsf_out>;



    // PA8 Led
    using led = Mcu::Stm::Pin<gpioa, 8, MCU>;

    // PB5 BT Pwr
    using btPwr = Mcu::Stm::Pin<gpiob, 5, MCU>;

    // Usart1: BT

    struct PassThru {
        static inline void start() {
        }
        static inline void stop() {
        }
        static inline void put(std::byte) {
        }
    };
    using passthru = PassThru;
    struct SendBack {
        static inline void put(std::byte) {
        }
    };
    using sendback = SendBack;
    using btPa = External::RoboCli::ProtocollAdapter<0, systemTimer, passthru, sendback, Obsolete::BtCallback>;
    using btUsart = Mcu::Stm::Uart<1, btPa, 2048, char, clock, MCU>;
    using bttx = Mcu::Stm::Pin<gpioa, 10, MCU>;
    using btrx = Mcu::Stm::Pin<gpioa, 9, MCU>;

    // PB13 HF Pwr
    using hfPwr = Mcu::Stm::Pin<gpiob, 13, MCU>;

    // PA6 FreqSel Tim3-Ch1 AF(2)
    using fSel = Mcu::Stm::Pin<gpioa, 6, MCU>;

    // I2C 1
    using sda1 = Mcu::Stm::Pin<gpiob, 7, MCU>;
    using scl1 = Mcu::Stm::Pin<gpioa, 15, MCU>;

    using i2c1 = Mcu::Stm::I2C::Master<1, 16, MCU>;
    static inline constexpr Mcu::Stm::I2C::Address si5351{0x60};
    using si = External::SI5351::Clock<i2c1, si5351, 867>;
    // using si = External::SI5351::Clock<i2c1, si5351, 0>;

    static inline constexpr Mcu::Stm::I2C::Address pcaAdr0{0x20};
    static inline constexpr Mcu::Stm::I2C::Address pcaAdr1{0x21};
    using pca0 = External::PCAL6408<i2c1, pcaAdr1, systemTimer>;
    using pca1 = External::PCAL6408<i2c1, pcaAdr0, systemTimer>;

    // PA12 Drehspul; Tim4-Ch2, AF(10)
    using ds = Mcu::Stm::Pin<gpioa, 12, MCU>;
    // PA11 Buzzer; Tim4-Ch1, AF(10)
    using buzz = Mcu::Stm::Pin<gpioa, 11, MCU>;
    using pwm = Mcu::Stm::Motor::Bdc<4, clock, MCU>;

    // DMA
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    using adcDmaChannel1 = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;
    using adcDmaChannel2 = Mcu::Stm::Dma::Channel<dma1, 2, MCU>;
    using cppmDmaChannel3 = Mcu::Stm::Dma::Channel<dma1, 3, MCU>;
    using dacDmaChannel4 = Mcu::Stm::Dma::Channel<dma1, 4, MCU>;

    using adcDmaStorage1 = std::array<volatile uint16_t, 7>;
    using adcDmaStorage2 = std::array<volatile uint16_t, 1>;
    using adc1 = Mcu::Stm::V3::Adc<1, Meta::NList<1, 2, 3, 4, 5, 10, 12>, pwm, adcDmaChannel1, adcDmaStorage1, void, MCU>;
    using adc2 = Mcu::Stm::V3::Adc<2, Meta::NList<12>, pwm, adcDmaChannel2, adcDmaStorage2, Meta::List<EndOfSequence>, MCU>;
    using an1 = Mcu::Stm::Pin<gpioa, 0, MCU>;
    using an2 = Mcu::Stm::Pin<gpioa, 1, MCU>;
    using an3 = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using an4 = Mcu::Stm::Pin<gpioa, 3, MCU>;
    using an5 = Mcu::Stm::Pin<gpiob, 14, MCU>;
    using an10 = Mcu::Stm::Pin<gpiof, 0, MCU>;
    using vtSense = Mcu::Stm::Pin<gpiob, 1, MCU>; // adc1 in12
    using vinSense = Mcu::Stm::Pin<gpiob, 2, MCU>; // adc2 in12

    using cppm = Mcu::Stm::Cppm::Generator<3, cppmDmaChannel3, clock, MCU>;

    using dac = Mcu::Stm::Dac<1, MCU>;
    using pulse = Mcu::Stm::Cppm::RollOnOff<6, dacDmaChannel4, dac, clock, MCU>;

    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        gpioa::init();
        gpiob::init();
        gpioc::init();
        gpiof::init();

        led::template dir<Mcu::Output>();
        btPwr::template dir<Mcu::Output>();
        hfPwr::template dir<Mcu::Output>();

        fSel::template dir<Mcu::Output>();
        fSel::set();
        // fSel::afunction(2);
        cppm::init();

        sda1::openDrain();
        scl1::openDrain();
        sda1::afunction(4);
        scl1::afunction(4);

        i2c1::init();

        an1::analog();
        an2::analog();
        an3::analog();
        an4::analog();
        an5::analog();
        an10::analog();
        vtSense::analog();
        vinSense::analog();

        adc1::init();
        adc2::init();

        pwm::init();
        pwm::pwm(400);
        pwm::duty(50);
        ds::afunction(10);
        // ds::template dir<Mcu::Output>();

        buzz::template dir<Mcu::Output>();
        // buzz::afunction(10);

        dac::init();
        pulse::init();

        btUsart::init();
        btUsart::baud(115200);
        bttx::afunction(7);
        btrx::afunction(7);

        // tp0::template dir<Mcu::Output>();
        // tp1::template dir<Mcu::Output>();

        crsf::init();
        crsf::baud(420'000);
        crsftx::afunction(7);
        crsftx::pullup<true>();
        crsfrx::afunction(7);
        // crsfrx::template dir<Mcu::Output>();
        // crsfrx::set();

        pca0::init();
        // pca1::init();
    }
};

