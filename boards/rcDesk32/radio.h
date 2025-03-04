#pragma once

#include <etl/event.h>
#include <etl/algorithm.h>

using namespace std::literals::chrono_literals;

template<uint8_t N, typename Config, typename MCU = DefaultMcu>
struct HwExtension {
    using clock = Config::clock;
    using systemTimer = Config::systemTimer;
    using debug = Config::debug;
    using dmaChComponent = Config::dmaChComponent;
    using pin = Config::pin;
    using tp = Config::tp;

    struct UartConfig {
        using Clock = clock;
        using ValueType = uint8_t;
        using DmaChComponent = dmaChComponent;
        static inline constexpr Mcu::Stm::Uarts::Mode mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
        static inline constexpr uint32_t baudrate = 115'200;
        struct Rx {
            static inline constexpr size_t size = 64;
            static inline constexpr size_t idleMinSize = 4;
        };
        struct Tx {
            static inline constexpr bool singleBuffer = true;
            static inline constexpr bool enable = true;
            static inline constexpr size_t size = 64;
        };
        struct Isr {
            static inline constexpr bool idle = true;
            static inline constexpr bool txComplete = true;
        };
        using tp = Config::tp;
    };

    using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

    static inline void init() {
        static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
        IO::outl<debug>("# HwExt init");
        Mcu::Arm::Atomic::access([]{
            mState = State::Init;
            mErrorCount = 0;
            mEvent = Event::None;
            mActive = true;
            uart::init();
        });
        pin::afunction(af);
        pin::template pullup<true>();
    }
    static inline void reset() {
        IO::outl<debug>("# HwExt reset");
        Mcu::Arm::Atomic::access([]{
            uart::reset();
            mActive = false;
        });
        pin::analog();
    }

    enum class State : uint8_t {Init, Run};
    enum class Event : uint8_t {None, ReceiveComplete, Send};

    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void periodic() {
        switch(mState) {
        case State::Init:
            break;
        case State::Run:
            if (mEvent.is(Event::ReceiveComplete)) {
                readReply();
            }
            else if (mEvent.is(Event::Send)) {
                send();
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
    struct Isr {
        static inline void onIdle(const auto f) {
            if (mActive) {
                const auto f2 = [&](const volatile uint8_t* const data, const uint16_t size){
                    f();
                    if (validityCheck(data, size)) {
                        event(Event::ReceiveComplete);
                        return true;
                    }
                    return false;
                };
                uart::Isr::onIdle(f2);
            }
        }
        static inline void onTransferComplete(const auto f) {
            if (mActive) {
                const auto fEnable = [&]{
                    f();
                    uart::template rxEnable<true>();
                };
                uart::Isr::onTransferComplete(fEnable);
            }
        }
    };
    static inline auto errorCount() {
        return mErrorCount;
    }
    static inline void set(const uint64_t sw) {
        mSwitches = sw;
    }
    private:
    static inline void send() {
        uart::fillSendBuffer([](auto& data){
            data[0] = 0xaa;
            data[1] = mControllerNumber;
            data[2] = 0x00; // type
            data[3] = 0x08; // length

            uint8_t cs = 0;
            for(uint8_t i = 0; i < 8; ++i) {
                cs += data[4 + i] = (mSwitches >> (i * 8));
            }
            data[4 + 8] = cs;
            return 4 + 8 + 1;
        });
    }
    static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t size) {
        if (size < 6) {
            return false;
        }
        if (data[0] != 0xaa) {
            return false;
        }
        const uint8_t l = data[3];
        uint8_t cs = 0;
        for(uint8_t i = 0; i < l; ++i) {
            cs += data[4 + i];
        }
        const uint8_t ics = data[4 + l];
        if (cs != ics) {
            ++mErrorCount;
            return false;
        }
        return true;
    }
    static inline void readReply() {
        uart::readBuffer([&](const auto& data){
            const uint8_t inController = data[1];
            if ((inController + 1) == mControllerNumber) {
                mEvent = Event::Send;
            }
        });
    }
    static inline uint64_t mSwitches = 0;
    static inline uint8_t mControllerNumber = 1;
    static inline uint16_t mErrorCount = 0;
    static inline etl::Event<Event> mEvent;
    static inline volatile State mState = State::Init;
    static inline External::Tick<systemTimer> mStateTick;
    static inline volatile bool mActive = false;
};
