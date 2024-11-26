#define USE_MCU_STM_V3
#define USE_CRSF_V3

#define NDEBUG

#include <cstdint>
#include <chrono>

#include "devices.h"

extern "C" {
extern int _end;
extern int _ebss;
extern unsigned char *heap;
}

template<typename Devs>
struct Auxes {
    using devs = Devs;
    using debug = devs::debug;
    using relay = devs::relay_aux;
    // using gps = devs::gps;

    static inline void set(const uint8_t r) {
        IO::outl<debug>("# aux ", r);
        switch(r) {
        case 0: // crsf
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<relay>>();
            break;
        case 1: // gps
            mRelay = nullptr;
            break;
        case 2: // none
            mRelay = nullptr;
            break;
        default:
            break;
        }
    }
    static inline constexpr void forwardPacket(const std::byte type, const std::array<uint8_t, 64>& data, const uint16_t length) {
        if (mRelay) {
            mRelay->forwardPacket(type, data, length);
        }
    }
    static inline void command(const std::array<uint8_t, 64>& data, const uint16_t length){
        if (mRelay) {
            mRelay->command(data, length);
        }
    }
    static inline void ping() {
        if (mRelay) {
            mRelay->ping();
        }
    }
    static inline void update() {
        if (mRelay) {
            mRelay->update();
        }
    }
    static inline void periodic() {
        if (mRelay) {
            mRelay->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mRelay) {
            mRelay->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<IRelay> mRelay{};
};

template<typename Devs>
struct Relays {
    using devs = Devs;
    using debug = devs::debug;
    using relay = devs::relay1;
    using sbus = devs::sbus1;

    static inline void set(const uint8_t r) {
        IO::outl<debug>("# relay ", r);
        switch(r) {
        case 0: //sbus
        {
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<sbus>>();
            Relay<sbus>* const rptr = static_cast<Relay<sbus>*>(mRelay.get());
            rptr->activateSBus2(false);
        }
            break;
        case 1: // crsf
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<relay>>();
            break;
        case 2: // sbus2
        {
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<sbus>>();
            Relay<sbus>* const rptr = static_cast<Relay<sbus>*>(mRelay.get());
            rptr->activateSBus2(true);
        }
            break;
        case 3: // none
            mRelay = nullptr;
            break;
        default:
            break;
        }
    }
    static inline void setChannel(const uint8_t ch, const uint16_t v) {
        if (mRelay) {
            mRelay->setChannel(ch, v);
        }
    }

    static inline constexpr void forwardPacket(const std::byte type, const std::array<uint8_t, 64>& data, const uint16_t length) {
        if (mRelay) {
            mRelay->forwardPacket(type, data, length);
        }
    }
    static inline void command(const std::array<uint8_t, 64>& data, const uint16_t length){
        if (mRelay) {
            mRelay->command(data, length);
        }
    }
    static inline void ping() {
        if (mRelay) {
            mRelay->ping();
        }
    }
    static inline void update() {
        if (mRelay) {
            mRelay->update();
        }
    }
    static inline void periodic() {
        if (mRelay) {
            mRelay->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mRelay) {
            mRelay->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<IRelay> mRelay{};
};

template<typename Devs>
struct EscOutputs {
    using devs = Devs;
    using debug = devs::debug;
    using esc1_pwm = devs::esc1_pwm;
    using esc2_pwm = devs::esc2_pwm;
    using esc32_1 = devs::esc32_1;
    using esc32_2 = devs::esc32_2;

    template<uint8_t N>
    static inline void esc(const uint8_t e) {
        IO::outl<debug>("# esc", N, ": ", e);
        static_assert(N <= 1);
        using esc_pwm_t = std::conditional_t<(N == 0), Esc<esc1_pwm>, Esc<esc2_pwm>>;
        using esc_esc32_t = std::conditional_t<(N == 0), Esc<esc32_1>, Esc<esc32_2>>;
        switch(e) {
        case 0: // PWM
            escs[N] = nullptr;
            escs[N] = std::make_unique<esc_pwm_t>();
            break;
        case 1: // Serial
            escs[N] = nullptr;
            escs[N] = std::make_unique<esc_esc32_t>();
            break;
        case 2: // Vesc
            escs[N] = nullptr;
            break;
        case 3: // None
            escs[N] = nullptr;
            break;
        default:
            break;
        }
    }
    static inline uint16_t current(const uint8_t n) {
        if ((n < escs.size()) && escs[n]) {
            return escs[n]->current();
        }
        return 0;
    }
    static inline uint16_t rpm(const uint8_t n) {
        if ((n < escs.size()) && escs[n]) {
            return escs[n]->rpm();
        }
        return 0;
    }
    static inline void set(const uint8_t n, const uint16_t v) {
        if ((n < escs.size()) && escs[n]) {
            escs[n]->set(v);
        }
    }
    static inline void periodic() {
        for(const auto& e : escs) {
            if (e) {
                e->periodic();
            }
        }
    }
    static inline void ratePeriodic() {
        for(const auto& e : escs) {
            if (e) {
                e->ratePeriodic();
            }
        }
    }
    private:
    static inline std::array<std::unique_ptr<IEsc>, 2> escs{};
};

template<typename Devs>
struct ServoOutputs {
    using devs = Devs;
    using debug = devs::debug;
    using servo1_ws = devs::srv1_waveshare;
    using servo2_ws = devs::srv2_waveshare;
    using servo1_ft = devs::srv1_feetech;
    // using servo2_ft = devs::srv2_feetech;

    template<uint8_t N>
    static inline void offset(const uint16_t o) {
        static_assert(N <= 1);
        if (servos[N]) {
            servos[N]->offset(o);
        }
    }
    template<uint8_t N>
    static inline void speed(const uint16_t o) {
        static_assert(N <= 1);
        if (servos[N]) {
            servos[N]->speed(o);
        }
    }
    static inline void update(const uint8_t n) {
        if ((n < servos.size()) && servos[n]) {
            servos[n]->update();
        }
    }
    static inline int8_t turns(const uint8_t n) {
        if ((n < servos.size()) && servos[n]) {
            return servos[n]->turns();
        }
        return 0;
    }
    static inline uint16_t actualPos(const uint8_t n) {
        if ((n < servos.size()) && servos[n]) {
            return servos[n]->actualPos();
        }
        return 0;
    }
    template<uint8_t N>
    static inline void servo(const uint8_t s) {
        IO::outl<debug>("# servo", N, ": ", s);
        static_assert(N <= 1);
        using srv_ws_t = std::conditional_t<(N == 0), Servo<servo1_ws>, Servo<servo2_ws>>;
        // using srv_ft_t = std::conditional_t<(N == 0), Servo<servo1_ft>, Servo<servo2_ft>>;
        using srv_ft_t = servo1_ft;
        switch(s) {
        case 0: // analog FB
            servos[N] = nullptr;
            servos[N] = std::make_unique<Servo<srv_ft_t>>();
            break;
        case 1: // PWM feedback
            servos[N] = nullptr;
            servos[N] = std::make_unique<Servo<srv_ft_t>>();
            break;
        case 2: // serial
            servos[N] = nullptr;
            servos[N] = std::make_unique<srv_ws_t>();
            break;
        case 3: // none
            servos[N] = nullptr;
            break;
        default:
            break;
        }
    }
    static inline void zero() {
        for(const auto& s : servos) {
            if (s) {
                s->zero();
            }
        }
    }
    static inline void periodic() {
        for(const auto& s : servos) {
            if (s) {
                s->periodic();
            }
        }
    }
    static inline void ratePeriodic() {
        for(const auto& s : servos) {
            if (s) {
                s->ratePeriodic();
            }
        }
    }
    private:
    static inline std::array<std::unique_ptr<IServo>, 2> servos{};
};

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

template<typename Devices, typename Servos, typename Escs, typename Relays, typename Auxes>
struct GFSM {
    using devs = Devices;
    using systemTimer = devs::systemTimer;

    using debug = devs::debug;

    using crsf_in = devs::crsf_in;
    using crsf_in_pa = crsf_in::adapter;
    using crsf_in_responder = crsf_in_pa::responder;
    using crsfBuffer = devs::crsfBuffer;

    using led1 = devs::ledBlinker1;
    using led2 = devs::ledBlinker2;

    using adc = devs::adc;

    using telemetry = devs::telem;

    using polar1 = devs::polar1;
    using polar2 = devs::polar2;

    // using tp0 = devs::tp0;
    using tp1 = devs::tp1;
    // using tp2 = devs::tp2;

    static inline void init() {
        devs::init();
        crsf_in_responder::address(std::byte(192));
        crsf_in_responder::telemetrySlot(0);
    }

    enum class State : uint8_t {Undefined, Init, Calib, Run};

    static inline void updateFromEeprom() {
        using cb = crsf_in_pa::CB;
        cb::callbacks(true);
    }

    static inline void periodic() {
        debug::periodic();
        crsf_in::periodic(); // status: new channels?
        Servos::periodic();
        Escs::periodic();
        Relays::periodic();
        Auxes::periodic();

        crsfBuffer::periodic();
    }

    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> telemetryTicks{100ms};

    static inline void ratePeriodic() {
        led1::ratePeriodic();
        led2::ratePeriodic();
        crsf_in_pa::ratePeriodic([]{});
        Servos::ratePeriodic();
        Escs::ratePeriodic();
        Relays::ratePeriodic();
        Auxes::ratePeriodic();

        crsfBuffer::ratePeriodic();

        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                if (adc::ready()) {
                    mState = State::Calib;
                }
                else {
                    mStateTick.reset();
                }
            });
            mState = State::Run;
            break;
        case State::Calib:
            mState = State::Run;
            break;
        case State::Run:
            (++mTelemetryTick).on(telemetryTicks, []{
                telemetry::next();
            });
            mStateTick.on(debugTicks, []{
                // IO::outl<debug>("_end:", &_end, " _ebss:", &_ebss, " heap:", heap);
                IO::outl<debug>("ch0: ", crsf_in_pa::values()[0], " phi: ", polar1::phi(), " amp: ", polar1::amp(), " a: ", Servos::actualPos(0), " t: ", Servos::turns(0));
                // IO::outl<debug>("esc32_1 e:", esc32_1::errorCount());
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                led1::event(led1::Event::Steady);
                break;
            case State::Calib:
                // servo1::event(servo1::Event::Calibrate);
                // servo1::event(servo1::Event::Run);
                break;
            case State::Run:
                led1::event(led1::Event::Slow);
                adc::start();
                break;
            }
        }
    }

    private:
    // static inline External::Tick<systemTimer> mUpdateTick;
    static inline External::Tick<systemTimer> mTelemetryTick;
    static inline External::Tick<systemTimer> mStateTick;
    static inline State mState{State::Undefined};
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
    __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}
extern "C" {
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
    using sbus1 = devs::sbus1;
    static_assert(sbus1::uart::number == 102);
    sbus1::onTransferComplete([]{
        sbus1::event(sbus1::Event::SendComplete);
    });
    using esc32_1 = devs::esc32_1;
    static_assert(esc32_1::uart::number == 2);
    esc32_1::uart::onTransferComplete([]{
        esc32_1::rxEnable();
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
    // sbus1 / relay1 use same LPUART(2), be sure to clear all flags
    relay::uart::mcuUart->ICR = -1;
}
void USART3_4_5_6_LPUART1_IRQHandler(){
    using esc32_2 = devs::esc32_2;
    static_assert(esc32_2::uart::number == 3);
    esc32_2::uart::onTransferComplete([]{
        esc32_2::rxEnable();
    });
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
// static
unsigned char *heap = NULL;
void* _sbrk(const int incr) {
    unsigned char *prev_heap;

    if (heap == NULL) {
        heap = (unsigned char *)&_end;
    }
    prev_heap = heap;

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

