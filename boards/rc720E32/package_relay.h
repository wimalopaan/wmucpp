#pragma once

#include "usart.h"
#include "rc/crsf.h"

using namespace std::literals::chrono_literals;

template<uint8_t N, bool HalfDuplex, typename Pin, typename Src, typename Dest, typename DmaChRW, typename Timer, typename Clk, typename Debug1, typename MCU = DefaultMcu>
struct PacketRelay {
    using pin = Pin;
    using src = typename Src::adapter;
    using dest = Dest;
    using clock = Clk;
    using debug = Debug1;
    using dmaChRW = DmaChRW;
    using systemTimer = Timer;

    struct UartConfig {
        using Clock = clock;
        using ValueType = uint8_t;
        static inline constexpr size_t size = 64;
        static inline constexpr size_t minSize = 4;
        using DmaChannelWrite = dmaChRW;
        using DmaChannelRead = DmaChRW;
        static inline constexpr bool useDmaTCIsr = false;
        static inline constexpr bool useIdleIsr = true;
        static inline constexpr bool useRxToIsr = false;
        static inline constexpr uint16_t rxToCount = 0;
        using Adapter = void;
        using Debug = debug;
    };

    using uart = Mcu::Stm::V2::Uart<N, UartConfig, MCU>;
    static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;

    static inline void init() {
        IO::outl<typename debug::debug>("# Relay ", N, " init");
        __disable_irq();
        uart::init();
        uart::template rxEnable<false>();
        uart::baud(420'000);
        uart::template halfDuplex<HalfDuplex>();
        uart::template enableTCIsr<true>();
        __enable_irq();

        pin::afunction(af);
        pin::template pullup<true>();
    }
    static inline void reset() {
        IO::outl<typename debug::debug>("# Relay ", N, " reset");
        uart::reset();
        pin::analog();
    }

    enum class Event : uint8_t {None, ReceiveComplete};
    enum class State : uint8_t {Init, Run};

    static inline constexpr External::Tick<systemTimer> initTicks{100ms};

    static inline void event(const Event e) {
        mEvent = e;
    }
    // static inline void activateSBus2(bool) { // only technical: to fullfill type: Relay
    // }

    static inline void periodic() {
        // besser:
        // dest sollte ein message-queue haben (max. size einer message wie crsf-packet (64))
        // diese queue muss dann auch vom crsf-responder
        // und telemetry genutzt werden.

        switch(mState) {
        case State::Init:
            break;
        case State::Run:
            if (std::exchange(mEvent, Event::None) == Event::ReceiveComplete) {
                // const auto span = std::span{uart::readBuffer(), uart::readBuffer() + uart::readCount()};
                // dest::emplace_back(span);
                std::copy(uart::readBuffer(), uart::readBuffer() + uart::readCount(), (uint8_t*)dest::outputBuffer());
                dest::startSend(uart::readCount());
            }
            break;
        }
    }
    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Init:
            mStateTick.on(initTicks, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Init:
                break;
            case State::Run:
                break;
            }
        }
    }
    static inline void rxEnable() {
        uart::dmaReenable([]{
            dmaChRW::clearTransferCompleteIF();
            uart::dmaSetupRead2(UartConfig::size);
        });
        uart::template rxEnable<true>();
    }
    static inline void update() { // channels to dest
        // debug::tp::set();
        RC::Protokoll::Crsf::V3::pack(src::values(), uart::outputBuffer());
        uart::template rxEnable<false>();
        uart::startSend(26);
        // debug::tp::reset();
    }
    template<typename C>
    static inline void command(const C& data, const uint16_t length) {
        // debug::tp::set();
        uart::outputBuffer()[0] = 0xc8;
        uart::outputBuffer()[1] = length + 2;
        uart::outputBuffer()[2] = 0x32; // command
        std::copy(std::begin(data), std::begin(data) + length + 1, &uart::outputBuffer()[3]);
        uart::template rxEnable<false>();
        uart::startSend(length + 2 + 2);
        // debug::tp::reset();
    }
    static inline void ping() {
        // debug::tp::set();
        uart::outputBuffer()[0] = 0xc8;
        uart::outputBuffer()[1] = 0x04;
        uart::outputBuffer()[2] = 0x28; // command
        uart::outputBuffer()[3] = 0x00;
        uart::outputBuffer()[4] = 0xea;
        uart::outputBuffer()[5] = 0x54;
        uart::template rxEnable<false>();
        uart::startSend(6);
        // debug::tp::reset();
    }

    template<typename C>
    static inline void forwardPacket(const std::byte type, const C& data, const uint16_t payLength) {
        const uint8_t crsfLength = payLength + 2;
        const uint8_t totalLength = crsfLength + 2;
        // debug::tp::set();
        uart::outputBuffer()[0] = 0xc8;
        uart::outputBuffer()[1] = crsfLength;
        uart::outputBuffer()[2] = (uint8_t)type;
        std::copy(std::begin(data), std::begin(data) + payLength + 1, // including CRC
                  &uart::outputBuffer()[3]);
        uart::template rxEnable<false>();
        uart::startSend(totalLength);
        // debug::tp::reset();
    }
    private:
    static inline Event mEvent = Event::None;
    static inline State mState = State::Init;
    static inline External::Tick<systemTimer> mStateTick;
};
