#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <chrono>

#include "etl/algorithm.h"
#include "etl/ranged.h"
#include "units.h"
#include "tick.h"
#include "usart_2.h"
#include "rc_2.h"

namespace RC::Protokoll::SPort::V2 {
    using namespace etl::literals;
    using namespace Units::literals;
    using namespace std::literals::chrono_literals;

    struct CheckSum {
        void reset() {
            mValue = 0;
        }
        void operator+=(const uint8_t b) {
            mValue += b;
            mValue += mValue >> 8;
            mValue &= 0x00ff;
        }
        uint8_t value() const {
            return (0xff - mValue);
        }
        private:
        uint16_t mValue{};
    };

    struct ByteStuff {
        enum class State : uint8_t {Normal, Stuff};
        auto decode(const std::byte b) {
            switch(mState) {
            case State::Normal:
                if (b == 0x7d_B) {
                    mState = State::Stuff;
                    return std::pair{b, false};
                }
                else {
                    return std::pair{b, true};
                }
                break;
            case State::Stuff:
                mState = State::Normal;
                if (b == 0x5d_B) {
                    return std::pair{0x7d_B, true};
                }
                else if (b == 0x5e_B) {
                    return std::pair{0x7e_B, true};
                }
                break;
            }
            return std::pair{b, false};
        }
        private:
        State mState{State::Normal};
    };
    namespace Master {
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Serial {
            using clock = Config::clock;
            using systemTimer = Config::systemTimer;
            using debug = Config::debug;
            using dmaChComponent = Config::dmaChComponent;
            using pin = Config::pin;
            using tp = Config::tp;
            using callback = Config::callback;

            struct UartConfig {
                using Clock = Config::clock;
                using ValueType = uint8_t;
                using DmaChComponent = dmaChComponent;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                static inline constexpr uint32_t baudrate = RC::Protokoll::SPort::V2::baudrate;
                static inline constexpr bool invert = true;
                struct Rx {
                    static inline constexpr size_t size = 16;
                    static inline constexpr size_t idleMinSize = 2;
                };
                struct Tx {
                    static inline constexpr bool singleBuffer = true;
                    static inline constexpr size_t size = 16;
                    static inline constexpr bool enable = true;
                };
                struct Isr {
                    static inline constexpr bool idle = true;
                    static inline constexpr bool txComplete = true;
                };
                using tp = Config::tp;
            };
            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

            static inline void init() {
                using pin = Config::pin;
                static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                IO::outl<debug>("# SPort Master init ", N);
                Mcu::Arm::Atomic::access([]{
                    mState = State::Init;
                    mEvent = Event::None;
                    mStateTick.reset();
                    mActive = true;
                    for(auto& r: mRequestCounters) {
                        r = 0;
                    }
                    mLastOnlineIndex = -1;
                    mLastOfflineIndex = -1;
                    uart::init();
                });
                pin::afunction(af);
                pin::template pulldown<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# SPort reset");
                Mcu::Arm::Atomic::access([]{
                    uart::reset();
                    mActive = false;
                });
            }

            enum class Event : uint8_t {None, ReadReply, StartReceive};
            enum class State : uint8_t {Init, ReceiveOnline, ReceiveOffline, RequestOnline, RequestOffline, WaitOnline, WaitOffline};

            static inline constexpr External::Tick<systemTimer> initTicks{100ms};
            static inline constexpr External::Tick<systemTimer> slotTicks{12ms};
            static inline constexpr External::Tick<systemTimer> waitTicks{4ms};

            static inline void event(const Event e) {
                mEvent = e;
            }
            static inline void periodic() {
                const State oldState = mState;
                switch(mState) {
                case State::Init:
                    break;
                case State::RequestOnline:
                    mEvent.on(Event::StartReceive, []{
                        mState = State::ReceiveOnline;
                    });
                    break;
                case State::RequestOffline:
                    mEvent.on(Event::StartReceive, []{
                        mState = State::ReceiveOffline;
                    });
                    break;
                case State::ReceiveOnline:
                    mEvent.on(Event::ReadReply, []{
                        if constexpr(!std::is_same_v<tp, void>) {
                            tp::set();
                            tp::reset();
                        }
                        mRequestCounters[mLastOnlineIndex] = 3;
                        readReply();
                        mState = State::WaitOnline;
                    });
                    break;
                case State::ReceiveOffline:
                    mEvent.on(Event::ReadReply, []{
                        mRequestCounters[mLastOfflineIndex] = 3;
                        mLastOnlineIndex = mLastOfflineIndex;
                        mLastOfflineIndex = -1;
                        readReply();
                        mState = State::WaitOnline;
                    });
                    break;
                case State::WaitOffline:
                    break;
                case State::WaitOnline:
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Init:
                        break;
                    case State::RequestOnline:
                        break;
                    case State::RequestOffline:
                        break;
                    case State::ReceiveOffline:
                        break;
                    case State::ReceiveOnline:
                        break;
                    case State::WaitOffline:
                        break;
                    case State::WaitOnline:
                        break;
                    }
                }
            }
            static inline void ratePeriodic() {
                const State oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::Init:
                    mStateTick.on(initTicks, []{
                        mState = State::RequestOffline;
                    });
                    break;
                case State::RequestOnline:
                    mStateTick.on(waitTicks, []{
                        mState = State::RequestOffline;
                    });
                    break;
                case State::RequestOffline:
                    mStateTick.on(waitTicks, []{
                        mState = State::RequestOnline;
                    });
                    break;
                case State::ReceiveOnline:
                    mStateTick.on(slotTicks, []{
                        if (mRequestCounters[mLastOnlineIndex] > 0) {
                            --mRequestCounters[mLastOnlineIndex];
                            mState = State::RequestOffline;
                        }
                        else {
                            mLastOfflineIndex = mLastOnlineIndex;
                            mLastOnlineIndex = -1;
                            mState = State::RequestOnline;
                        }
                    });
                    break;
                case State::ReceiveOffline:
                    mStateTick.on(slotTicks, []{
                        mState = State::RequestOnline;
                    });
                    break;
                case State::WaitOnline:
                    mStateTick.on(waitTicks, []{
                        mState = State::RequestOffline;
                    });
                    break;
                case State::WaitOffline:
                    mStateTick.on(waitTicks, []{
                        mState = State::RequestOnline;
                    });
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Init:
                        break;
                    case State::RequestOnline:
                        uart::fillSendBuffer([&](auto& data){
                            data[0] = 0x7e;
                            for(uint8_t i = 0; i < mRequestCounters.size(); ++i) {
                                const uint8_t k = (i + mLastOnlineIndex + 1) % mRequestCounters.size();
                                if ((k != mLastOfflineIndex) && (mRequestCounters[k] > 0)) {
                                     data[1] = (uint8_t)RC::Protokoll::SPort::V2::sensor_ids[k];
                                     --mRequestCounters[k];
                                     mLastOnlineIndex = k;
                                     return 2;
                                }
                            }
                            return 0;
                        });
                        break;
                    case State::RequestOffline:
                        uart::fillSendBuffer([&](auto& data){
                            data[0] = 0x7e;
                            for(uint8_t i = 0; i < mRequestCounters.size(); ++i) {
                                const uint8_t k = (i + mLastOfflineIndex + 1) % mRequestCounters.size();
                                if ((k != mLastOnlineIndex) && (mRequestCounters[k] == 0)) {
                                     data[1] = (uint8_t)RC::Protokoll::SPort::V2::sensor_ids[k];
                                     mLastOfflineIndex = k;
                                     return 2;
                                }
                            }
                            return 0;
                        });
                        break;
                        if constexpr(!std::is_same_v<tp, void>) {
                            tp::set();
                            tp::reset();
                        }

                    case State::ReceiveOffline:
                        break;
                    case State::ReceiveOnline:
                        break;
                    case State::WaitOffline:
                        break;
                    case State::WaitOnline:
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
                                event(Event::ReadReply);
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
                            event(Event::StartReceive);
                        };
                        uart::Isr::onTransferComplete(fEnable);
                    }
                }
            };
        private:
            static inline void readReply() {
                uart::readBuffer([&](const auto& /*data*/){

                });
            }
            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t size) {
                if constexpr(!std::is_same_v<tp, void>) {
                    tp::set();
                    tp::reset();
                }
                // calc checksum
                return (size >= 2) && (data[0] == 0x10);
            }
            static inline bool mActive = false;
            static inline etl::Event<Event> mEvent;
            static inline State mState{State::Init};
            static inline External::Tick<systemTimer> mStateTick;
            static inline int8_t mLastOnlineIndex = -1;
            static inline int8_t mLastOfflineIndex = -1;
            static inline std::array<uint8_t, RC::Protokoll::SPort::V2::sensor_ids.size()> mRequestCounters{};
        };
    }
    namespace Client {
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Serial {
            using clock = Config::clock;
            using systemTimer = Config::systemTimer;
            using debug = Config::debug;
            using dmaChComponent = Config::dmaChComponent;
            using pin = Config::pin;
            using tp = Config::tp;
            using callback = Config::callback;

            struct UartConfig {
                using Clock = Config::clock;
                using ValueType = uint8_t;
                using DmaChComponent = dmaChComponent;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                static inline constexpr uint32_t baudrate = RC::Protokoll::SPort::V2::baudrate;
                static inline constexpr bool invert = true;                              // optional
                struct Rx {
                    static inline constexpr size_t size = 16;
                    static inline constexpr size_t idleMinSize = 2;
                };
                struct Tx {
                    static inline constexpr bool singleBuffer = true;
                    static inline constexpr size_t size = 16;
                    static inline constexpr bool enable = true;
                };
                struct Isr {
                    static inline constexpr bool idle = true;
                    static inline constexpr bool txComplete = true;
                };
                using tp = Config::tp;
            };

            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

            static inline void init() {
                using pin = Config::pin;
                static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                IO::outl<debug>("# SPort Client init ", N);
                Mcu::Arm::Atomic::access([]{
                    mState = State::Init;
                    mEvent = Event::None;
                    mStateTick.reset();
                    mActive = true;
                    uart::init();
                });
                pin::afunction(af);
                pin::template pulldown<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# SPort reset");
                Mcu::Arm::Atomic::access([]{
                    uart::reset();
                    mActive = false;
                });
            }

            enum class Event : uint8_t {None, ReadRequest, StartReceive};
            enum class State : uint8_t {Init, Receive, Reply};

            static inline constexpr External::Tick<systemTimer> replyTicks{4ms};

            static inline void event(const Event e) {
                mEvent = e;
            }
            static inline void periodic() {
                switch(mState) {
                case State::Init:
                    break;
                case State::Receive:
                    if (mEvent.is(Event::ReadRequest)) {
                        if (readRequest()) {
                            mState = State::Reply;
                            mStateTick.reset();
                        }
                    }
                    break;
                case State::Reply:
                    if (mEvent.is(Event::StartReceive)) {
                        mState = State::Receive;
                        uart::template rxEnable<true>();
                    }
                    break;
                }
            }
            static inline void ratePeriodic() {
                const auto oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::Init:
                    mState = State::Receive;
                    break;
                case State::Receive:
                    break;
                case State::Reply:
                    mStateTick.on(replyTicks, []{
                        sendReply();
                    });
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Init:
                        break;
                    case State::Receive:
                        uart::template rxEnable<true>();
                        break;
                    case State::Reply:
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
                                event(Event::ReadRequest);
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
                            event(Event::StartReceive);
                        };
                        uart::Isr::onTransferComplete(fEnable);
                    }
                }
            };
            static inline void setPhysID0(const uint8_t id){
                mPhysicalID0 = RC::Protokoll::SPort::V2::getDataId(id);
            }
            static inline void setPhysID1(const uint8_t id){
                mPhysicalID1 = RC::Protokoll::SPort::V2::getDataId(id);
            }
            static inline void setAppID0(const uint8_t id){
                mValueID0 = (id << 8);
            }
            static inline void setAppID1(const uint8_t id){
                mValueID1 = (id << 8);
            }
            static inline void setValue0(const uint32_t v){
                mValue0 = v;
            }
            static inline void setValue1(const uint32_t v){
                mValue1 = v;
            }
            private:
            static inline bool readRequest() {
                bool sendReply = false;
                uart::readBuffer([&](const auto& data){
                    if (data[1] == mPhysicalID0) {
                        if (data.size() == 2) {
                            sendReply = true;
                            mActualValueID = mValueID0;
                            mActualValue = mValue0;
                        }
                        else {
                            CheckSum csum;
                            for(uint8_t i = 2; i < 9; ++i) {
                                csum += data[i];
                            }
                            if (csum.value() == data[9]) {
                                callback::command(&data[2]);
                            }
                            mActualValueID = 0;
                        }
                    }
                    else if (data[1] == mPhysicalID1) {
                        if (data.size() == 2) {
                            sendReply = true;
                            mActualValueID = mValueID1;
                            mActualValue = mValue1;
                        }
                    }
                });
                return sendReply;
            }
            inline static void stuffResponse(const uint8_t b, auto& data, uint8_t& i, CheckSum& cs) {
                cs += b;
                if (b == 0x7e) {
                    data[i++] = 0x7d;
                    data[i++] = 0x5e;
                }
                else if (b == 0x7d) {
                    data[i++] = 0x7d;
                    data[i++] = 0x5d;
                }
                else {
                    data[i++] = b;
                }
            }
            static inline void sendReply() {
                if (mActualValueID > 0) {
                    uart::template rxEnable<false>();
                    uart::fillSendBuffer([&](auto& data){
                        CheckSum csum;
                        uint8_t i = 0;
                        stuffResponse(0x10, data, i, csum);
                        stuffResponse((mActualValueID & 0xff), data, i, csum);
                        stuffResponse((mActualValueID >> 8) & 0xff, data, i, csum);
                        stuffResponse((mActualValue >> 0) & 0xff, data, i, csum);
                        stuffResponse((mActualValue >> 8) & 0xff, data, i, csum);
                        stuffResponse((mActualValue >> 16) & 0xff, data, i, csum);
                        stuffResponse((mActualValue >> 24) & 0xff, data, i, csum);
                        data[i++] = csum.value();
                        return i;
                    });
                }
            }
            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t size) {
                return (size >= 2) && (data[0] == 0x7e);
            }
            static inline uint16_t mActualValueID = 0;
            static inline uint32_t mActualValue = 0;

            static inline bool mActive = false;
            static inline etl::Event<Event> mEvent;
            static inline State mState{State::Init};
            static inline External::Tick<systemTimer> mStateTick;
            static inline uint8_t mPhysicalID0 = RC::Protokoll::SPort::V2::getDataId(0x00);
            static inline uint8_t mPhysicalID1 = RC::Protokoll::SPort::V2::getDataId(0x01);
            static inline uint16_t mValueID0 = (uint16_t)RC::Protokoll::SPort::V2::ValueId::DIY;
            static inline uint16_t mValueID1 = (uint16_t)RC::Protokoll::SPort::V2::ValueId::DIY2;
            static inline uint32_t mValue0 = 1234;
            static inline uint32_t mValue1 = 2345;
        };
    }
}

