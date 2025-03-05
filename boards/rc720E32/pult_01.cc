#define USE_MCU_STM_V3
#define USE_CRSF_V3

#define SERIAL_DEBUG // enable debug on esc-tlm-1

#define USE_UART_2

#define SW_VERSION 1
#define HW_VERSION 1

#define NDEBUG

#include <cstdint>
#include <chrono>

#include "devices_pult.h"

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
    static inline const EEProm eeprom_flash;
    __attribute__ ((aligned (8)))
    static inline EEProm eeprom;
};

struct DevsConfig {
};
using devs = Devices<SW01, DevsConfig, Mcu::Stm::Stm32G0B1>;

template <typename Devs>
struct GFSM {
    static inline void init() {

    }
    static inline void periodic() {

    }
    static inline void ratePeriodic() {

    }
};

using gfsm = GFSM<devs>;
int main() {
    Storage::init();
    gfsm::init();

    // NVIC_EnableIRQ(USART1_IRQn);
    // NVIC_EnableIRQ(USART2_LPUART2_IRQn);
    // NVIC_EnableIRQ(USART3_4_5_6_LPUART1_IRQn);
    // NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    // NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);
    // NVIC_EnableIRQ(ADC1_COMP_IRQn);
    // NVIC_EnableIRQ(TIM3_TIM4_IRQn);
    __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}
extern "C" {

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

