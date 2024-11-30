#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <algorithm>
#include <array>

#include "mcu/alternate.h"
#include "byte.h"
#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "units.h"
#include "tick.h"
#include "rc/rc.h"

template<uint8_t N, typename Config, typename MCU = DefaultMcu>
struct SBusInput {
    using clock = Config::clock;
    using systemTimer = Config::systemTimer;
    using debug = Config::debug;
    using dmaChRead = Config::dmaChRead;
    using pin = Config::pin;
    using tp = Config::tp;

    struct UartConfig {
        using Clock = clock;
        using ValueType = uint8_t;
        static inline constexpr size_t size = 27; // 25 + X buffer size
        static inline constexpr size_t minSize = 4; // receive size
        using DmaChannelWrite = void;
        using DmaChannelRead = dmaChRead;
        static inline constexpr bool useDmaTCIsr = false;
        static inline constexpr bool useIdleIsr = true;
        static inline constexpr bool useRxToIsr = false;
        static inline constexpr uint16_t rxToCount = 0;
        using Adapter = void;
        using Debug = struct {
            using tp = void;
        };
    };

    using uart = Mcu::Stm::V2::Uart<N, UartConfig, MCU>;

    static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;

    static inline void init() {
        IO::outl<debug>("# SBus init");
        __disable_irq();
        mState = State::Init;
        mErrorCount = 0;
        mEvent = Event::None;
        mActive = true;
        uart::init();
        uart::template txEnable<false>();
        uart::baud(100'000);
        uart::template halfDuplex<true>();
        uart::parity(true);
        uart::invert(true);
        __enable_irq();
        for(auto& v: mChannels) {
            v = 992;
        }
        mFlagsAndSwitches = 0;
        pin::afunction(af);
        pin::template pullup<true>();
    }
    static inline void reset() {
        IO::outl<debug>("# SBus reset");
        __disable_irq();
        mActive = false;
        dmaChRead::enable(false);
        uart::reset();
        __enable_irq();
        pin::analog();
    }

    enum class State : uint8_t {Init, Run};
    enum class Event : uint8_t {None, ReceiveComplete};

    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void periodic() {
        switch(mState) {
        case State::Init:
            break;
        case State::Run:
            if (const auto e = std::exchange(mEvent, Event::None); e == Event::ReceiveComplete) {
                readReply();
            }
            break;
        }
    }

    static inline constexpr External::Tick<systemTimer> initTicks{1000ms};

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
    static inline void update() {
    }
    static inline void onIdle(const auto f) {
        if (mActive) {
            uart::onIdle([&]{
                uart::rxClear();
                dmaChRead::count(UartConfig::size);
                uart::onParityGood([&]{
                    f();
                });
            });
        }
    }
    static inline uint16_t value(const uint8_t ch) {
        return mChannels[ch];
    }
    static inline auto errorCount() {
        return mErrorCount;
    }
    private:

    static inline void readReply() {
        const volatile uint8_t* const buf = uart::readBuffer();
        if (buf[0] != 0x0f) return;
        if (buf[24] != 0x00) return;
        const volatile uint8_t* const mData = buf + 1;
        mChannels[0]  = (uint16_t) (((mData[0]    | mData[1] << 8))                     & 0x07FF);
        mChannels[1]  = (uint16_t) ((mData[1]>>3  | mData[2] <<5)                     & 0x07FF);
        mChannels[2]  = (uint16_t) ((mData[2]>>6  | mData[3] <<2 |mData[4]<<10)  	 & 0x07FF);
        mChannels[3]  = (uint16_t) ((mData[4]>>1  | mData[5] <<7)                     & 0x07FF);
        mChannels[4]  = (uint16_t) ((mData[5]>>4  | mData[6] <<4)                     & 0x07FF);
        mChannels[5]  = (uint16_t) ((mData[6]>>7  | mData[7] <<1 |mData[8]<<9)   	 & 0x07FF);
        mChannels[6]  = (uint16_t) ((mData[8]>>2  | mData[9] <<6)                     & 0x07FF);
        mChannels[7]  = (uint16_t) ((mData[9]>>5  | mData[10]<<3)                     & 0x07FF);
        mChannels[8]  = (uint16_t) ((mData[11]    | mData[12]<<8)                     & 0x07FF);
        mChannels[9]  = (uint16_t) ((mData[12]>>3 | mData[13]<<5)                     & 0x07FF);
        mChannels[10] = (uint16_t) ((mData[13]>>6 | mData[14]<<2 |mData[15]<<10) 	 & 0x07FF);
        mChannels[11] = (uint16_t) ((mData[15]>>1 | mData[16]<<7)                     & 0x07FF);
        mChannels[12] = (uint16_t) ((mData[16]>>4 | mData[17]<<4)                     & 0x07FF);
        mChannels[13] = (uint16_t) ((mData[17]>>7 | mData[18]<<1 |mData[19]<<9)  	 & 0x07FF);
        mChannels[14] = (uint16_t) ((mData[19]>>2 | mData[20]<<6)                     & 0x07FF);
        mChannels[15] = (uint16_t) ((mData[20]>>5 | mData[21]<<3)                     & 0x07FF);
        mFlagsAndSwitches = mData[22] & 0x0f;
    }
    static inline volatile bool mActive = false;
    static inline std::array<uint16_t, 16> mChannels;
    static inline uint8_t mFlagsAndSwitches = 0;
    static inline uint16_t mErrorCount = 0;
    static inline volatile Event mEvent = Event::None;
    static inline volatile State mState = State::Init;
    static inline External::Tick<systemTimer> mStateTick;
};

