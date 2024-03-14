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

struct Var01;

using namespace Mcu::Stm;
using namespace std::literals::chrono_literals;

struct BtCallback;

template<typename HW, typename Config, typename MCU = void>
struct Devices;

template<typename Config>
struct Devices<Var01, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>;
    using trace = Arm::Trace<clock, 2_MHz, 128>;

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;
    using gpiof = Mcu::Stm::GPIO<Mcu::Stm::F, MCU>;

    // PB11 CRSF TX
    using crsftx = Mcu::Stm::Pin<gpiob, 11, MCU>;
    using tp0 = crsftx;
    // PB10 CRSF RX
    using crsfrx = Mcu::Stm::Pin<gpiob, 10, MCU>;
    using tp1 = crsfrx;

    // Usart 3: CRSF
    struct CrsfCallback;
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfCallback, trace, MCU>;
    using crsf    = Mcu::Stm::Uart<3, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;

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
    using btPa = External::RoboCli::ProtocollAdapter<0, systemTimer, passthru, sendback, BtCallback>;
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

        tp0::template dir<Mcu::Output>();
        tp1::template dir<Mcu::Output>();

    }
};
