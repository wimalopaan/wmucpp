#define USE_MCU_STM_V3
#define USE_CRSF_V3

#define ESCAPE32_ASCII // enable ESCape32 ascii configuration parameter menu
#define SERVO_CALIBRATION // enable snalog feedback servo calibration
#define SERVO_ADDRESS_SET // eanble setting waveshare servo IDs
#define CRSF_ADDRESS 192
#define SERIAL_DEBUG // enable debug on esc-tlm-1
#define TEST_EEPROM // fill eeprom with test setup

#define NDEBUG

#include <cstdint>
#include <chrono>

#include "output_aux.h"
#include "output_esc.h"
#include "output_relay.h"
#include "output_servo.h"
#include "gfsm.h"

#include "devices.h"

using namespace std::literals::chrono_literals;

struct Storage {
    static inline void init() {
        std::memcpy(&eeprom, &eeprom_flash, sizeof(EEProm));
        // eeprom = eeprom_flash; // not working: needs volatile
    }
    static inline void reset() {
        eeprom = EEProm{};
    }

    __attribute__((__section__(".eeprom")))
    static inline const EEProm eeprom_flash{};

    static inline EEProm eeprom;
};

struct DevsConfig;
using devs = Devices<SW01, DevsConfig, Mcu::Stm::Stm32G0B1>;
using servooutputs = ServoOutputs<devs>;
using escoutputs = EscOutputs<devs>;
using relayoutputs = Relays<devs>;
using auxoutputs = Auxes<devs>;

struct DevsConfig {
    using storage = Storage;
    using servos = servooutputs;
    using escs = escoutputs;
    using relays = relayoutputs;
    using auxes = auxoutputs;
};
using gfsm = GFSM<devs, servooutputs, escoutputs, relayoutputs, auxoutputs>;

int main() {
    Storage::init();
    gfsm::init();
    gfsm::updateFromEeprom();

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART2_LPUART2_IRQn);
    NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);
    NVIC_EnableIRQ(TIM3_TIM4_IRQn);
    __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}
extern "C" {
void TIM3_TIM4_IRQHandler() {
    using pulse_in = devs::pulse_in;
    static_assert(pulse_in::timerNumber == 4);
    pulse_in::onCapture([]{
        // devs::tp3::set();
        // devs::tp3::reset();
    });
}

void ADC1_COMP_IRQHandler() {
    using adc = devs::adc;
    if (adc::mcuAdc->ISR & ADC_ISR_EOS) {
        adc::mcuAdc->ISR = ADC_ISR_EOS; // end-of-sequence
    }
}
void DMA1_Channel1_IRQHandler() {
    using crsf_in = devs::crsf_in;
    using dmaChRead = crsf_in::dmaChRead;
    static_assert(dmaChRead::number == 1);
}
void DMA1_Channel2_3_IRQHandler() {
    using adc = devs::adc;
    using adcDma = adc::dmaChannel;
    static_assert(adcDma::number == 3);

    adcDma::onTransferComplete([]{
        // devs::tp3::set();
        // devs::tp3::reset();
    });

    using crsf_in = devs::crsf_in;
    using dmaChWrite = crsf_in::dmaChWrite;
    static_assert(dmaChWrite::number == 2);

    dmaChWrite::onTransferComplete([]{
        crsf_in::dmaDisable(); // end-of-transmission
    });
}
void DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler() {
    using ws1 = devs::srv1_waveshare;
    static_assert(ws1::dmaChRW::number == 4);
    ws1::dmaChRW::onTransferComplete([]{
        ws1::event(ws1::Event::ReadReply);
    });
    using ws2 = devs::srv2_waveshare;
    static_assert(ws2::dmaChRW::number == 3);
    ws2::dmaChRW::onTransferComplete([]{
        ws2::event(ws2::Event::ReadReply);
    });

    using sbus1 = devs::sbus1;
    static_assert(sbus1::dmaChRW::number == 6);
    sbus1::dmaChRW::onTransferComplete([]{
        sbus1::slotReceived();
    });
    using esc32_1 = devs::esc32_1;
    static_assert(esc32_1::dmaChRW::number == 5);
    esc32_1::dmaChRW::onTransferComplete([]{
        esc32_1::event(esc32_1::Event::ReceiveComplete);
    });
    using esc32_2 = devs::esc32_2;
    static_assert(esc32_2::dmaChRW::number == 4);
    esc32_2::dmaChRW::onTransferComplete([]{
        esc32_2::event(esc32_2::Event::ReceiveComplete);
    });
}
void USART2_LPUART2_IRQHandler(){
    using esc32_1 = devs::esc32_1;
    static_assert(esc32_1::uart::number == 2);
    esc32_1::onTransferComplete([]{
        esc32_1::rxEnable();
    });
    using esc32ascii_1 = devs::esc32ascii_1;
    static_assert(esc32ascii_1::uart::number == 2);
    esc32ascii_1::onTransferComplete([]{
        esc32ascii_1::rxEnable();
    });
    esc32ascii_1::onIdleWithDma([]{
        esc32ascii_1::event(esc32ascii_1::Event::ReceiveComplete);
    });
    using vesc_1 = devs::vesc_1;
    static_assert(vesc_1::uart::number == 2);
    vesc_1::Isr::onTransferComplete([]{
        vesc_1::rxEnable();
    });
    vesc_1::Isr::onIdle([]{
    });
    esc32_1::uart::mcuUart->ICR = -1;

    using sbus1 = devs::sbus1;
    static_assert(sbus1::uart::number == 102);
    sbus1::onTransferComplete([]{
        sbus1::event(sbus1::Event::SendComplete);
    });
    using relay = devs::relay1;
    if constexpr(relay::uart::number == 102) {
        static_assert(relay::uart::number == 102);
        relay::onTransferComplete([]{
            // devs::tp3::set();
            relay::rxEnable();
            // devs::tp3::reset();
        });
        relay::onIdleWithDma([]{
            // devs::tp3::set();
            relay::event(relay::Event::ReceiveComplete);
            // devs::tp3::reset();
        });
    }
    using ibus_in = devs::ibus_in;
    if constexpr(ibus_in::uart::number == 102) {
        static_assert(ibus_in::uart::number == 102);
        ibus_in::onIdle([]{
            // devs::tp3::set();
            ibus_in::event(ibus_in::Event::ReceiveComplete);
            // devs::tp3::reset();
        });
    }
    using sbus_in = devs::sbus_in;
    if constexpr(sbus_in::uart::number == 102) {
        static_assert(sbus_in::uart::number == 102);
        sbus_in::onIdle([]{
            // devs::tp3::set();
            sbus_in::event(sbus_in::Event::ReceiveComplete);
            // devs::tp3::reset();
        });
    }
    using sumdv3_in = devs::sumdv3_in;
    if constexpr(sumdv3_in::uart::number == 102) {
        static_assert(sumdv3_in::uart::number == 102);
        sumdv3_in::onIdle([]{
            // devs::tp3::set();
            sumdv3_in::event(sumdv3_in::Event::ReceiveComplete);
            // devs::tp3::reset();
        });
    }
    // sbus1 / relay1 / ibus / sbus_in / sumdv3 use same LPUART(2), be sure to clear all flags
    relay::uart::mcuUart->ICR = -1;
}
void USART3_4_5_6_LPUART1_IRQHandler(){
    using esc32_2 = devs::esc32_2;
    static_assert(esc32_2::uart::number == 3);
    esc32_2::uart::onTransferComplete([]{
        esc32_2::rxEnable();
    });
    using esc32ascii_2 = devs::esc32ascii_2;
    static_assert(esc32ascii_2::uart::number == 3);
    esc32ascii_2::onTransferComplete([]{
        esc32ascii_2::rxEnable();
    });
    esc32ascii_2::onIdleWithDma([]{
        esc32ascii_2::event(esc32ascii_2::Event::ReceiveComplete);
    });
    using vesc_2 = devs::vesc_2;
    static_assert(vesc_2::uart::number == 3);
    vesc_2::Isr::onTransferComplete([]{
        vesc_2::rxEnable();
    });
    vesc_2::Isr::onIdle([]{
    });
    esc32_2::uart::mcuUart->ICR = -1;

    using ws1 = devs::srv1_waveshare;
    static_assert(ws1::uart::number == 5);
    ws1::uart::onTransferComplete([]{
        ws1::rxEnable();
    });
    using ws2 = devs::srv2_waveshare;
    static_assert(ws2::uart::number == 6);
    ws2::uart::onTransferComplete([]{
        ws2::rxEnable();
    });
    using relay = devs::relay_aux;
    if constexpr(relay::uart::number == 4) {
        static_assert(relay::uart::number == 4);
        relay::uart::onTransferComplete([]{
            // devs::tp3::set();
            relay::rxEnable();
            // devs::tp3::reset();
        });
        relay::uart::onIdleWithDma([]{
            // devs::tp3::set();
            relay::event(relay::Event::ReceiveComplete);
            // devs::tp3::reset();
        });
    }
}
void USART1_IRQHandler() {
    using crsf_in = devs::crsf_in;
    static_assert(crsf_in::number == 1);
    crsf_in::isr(); // idle-line
}

extern int _end;
static unsigned char *heap = NULL;
void* _sbrk(const int incr) {
    if (heap == NULL) {
        heap = (unsigned char *)&_end;
    }
    unsigned char* prev_heap = heap;
    heap += incr;
    return prev_heap;
}
void _exit(int) {
    __asm("BKPT #0");
}
void _kill(int, int) {
    return;
}
int _getpid(void) {
    return -1;
}

}

