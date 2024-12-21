#pragma once

#include "mcu/alternate.h"

#include "usart_2.h"
#include "output.h"
#include "util.h"
#include "debug_pin.h"

using namespace std::literals::chrono_literals;

namespace External::WaveShare {
    namespace V2 {
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Servo {
            using pin = Config::pin;
            using polar = Config::polar;
            using clock = Config::clk;
            using debug = Config::dbg;
            using tp = Config::tp;
            using dmaChComponent = Config::dmaChComponent;
            using systemTimer = Config::timer;
            using storage = Config::storage;

            struct UartConfig {
                using Clock = clock;
                using ValueType = uint8_t;
                using DmaChComponent = dmaChComponent;
                static inline constexpr bool fifo = false;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
                static inline constexpr uint32_t baudrate = 1'000'000;
                struct Rx {
                    static inline constexpr bool enable = false;
                    static inline constexpr size_t size = 64;
                    static inline constexpr size_t idleMinSize = 4;
                };
                struct Tx {
                    static inline constexpr bool enable = true;
                    static inline constexpr size_t size = 16;
                };
                struct Isr {
                    static inline constexpr bool idle = true;
                    static inline constexpr bool txComplete = true;
                };
                using tp = Servo::tp;
            };

            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

            static inline void init() {
                static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
                IO::outl<debug>("# WS init");
                Mcu::Arm::Atomic::access([]{
                    mState = State::Init;
                    mEvent = Event::None;
                    mReadErrorCount = 0;
                    mReceivedPackets = 0;
                    mLastError = 0;
                    mStateTick.reset();
                    mActive = true;
                    uart::init();
                });
                pin::afunction(af);
                pin::template pullup<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# WS reset");
                Mcu::Arm::Atomic::access([]{
                    uart::reset();
                    mActive = false;
                });
            }

            static inline constexpr External::Tick<systemTimer> initTicks{3000ms};
            static inline constexpr External::Tick<systemTimer> stepTicks{50ms};
            static inline constexpr External::Tick<systemTimer> telemTicks{10ms};

            static inline constexpr uint8_t CmdPing  = 0x01;
            static inline constexpr uint8_t CmdRead  = 0x02;
            static inline constexpr uint8_t CmdWrite = 0x03;

            enum class Event : uint8_t {None, ReadReply};

            enum class State : uint8_t {Init,
                                        ReadFwHw,
                                        SetAbsoluteMode, SetMin, SetMax,
                                        SetSpeed, SetDisableAutoReply,
                                        SetAbsoluteZero, AbsoluteZeroWait,
                                        ReadPosition,
                                        Run};

            static inline void speed(const uint16_t s) {
                IO::outl<debug>("# WS speed ", s);
                mSpeed = std::min(s, uint16_t{3400});
                if (mState == State::Run) {
                    setSpeed(mSpeed);
                }
            }
            static inline void event(const Event e) {
                mEvent = e;
            }
            static inline void zero() {
                if (mState == State::Run) {
                    mTurns = 0;
                    positionMultiTurn(0);
                }
            }
            static inline void update() {
                if (mState == State::Run) {
                    const uint16_t phi = polar::phi();
                    positionMultiTurn(phi);
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
                    auto fEnable = [&]{
                        f();
                        uart::template rxEnable<true>();
                    };
                    if (mActive) {
                        uart::Isr::onTransferComplete(fEnable);
                    }
                }
            };

            static inline void periodic() {
                const auto oldState = mState;
                switch(mState) {
                case State::Init:
                    break;
                case State::ReadFwHw:
                    if (mEvent.is(Event::ReadReply)) {
                        if (readReply()) {
                            mFW = uart::readBuffer()[5] * 100 + uart::readBuffer()[6];
                            mHW = uart::readBuffer()[7] * 100 + uart::readBuffer()[8];
                            ++mReceivedPackets;
                        }
                        else {
                            mState = State::Init;
                        }
                        mState = State::SetAbsoluteMode;
                    }
                    break;
                case State::SetAbsoluteMode:
                    if (mEvent.is(Event::ReadReply)) {
                        mState = State::SetAbsoluteZero;
                    }
                    break;
                case State::SetAbsoluteZero:
                    if (mEvent.is(Event::ReadReply)) {
                        mState = State::AbsoluteZeroWait;
                    }
                    break;
                case State::AbsoluteZeroWait:
                    break;
                case State::SetMin:
                    if (mEvent.is(Event::ReadReply)) {
                        mState = State::SetMax;
                    }
                    break;
                case State::SetMax:
                    if (mEvent.is(Event::ReadReply)) {
                        mState = State::SetSpeed;
                    }
                    break;
                case State::SetSpeed:
                    if (mEvent.is(Event::ReadReply)) {
                        if (readReply()) {
                            mState = State::ReadPosition;
                            ++mReceivedPackets;
                        }
                    }
                    break;
                case State::ReadPosition:
                    if (mEvent.is(Event::ReadReply)) {
                        if (readReply()) {
                            mLastPos = (uart::readBuffer()[6] << 8) + uart::readBuffer()[5];
                            mStartPos = mLastPos;
                            ++mReceivedPackets;
                        }
                        mState = State::SetDisableAutoReply;
                    }
                    break;
                case State::SetDisableAutoReply:
                    break;
                case State::Run:
                    if (mEvent.is(Event::ReadReply)) {
                        if (readReply()) {
                            ++mReceivedPackets;
                            getActualAbsPos();
                        }
                    }
                    break;
                }
                exitEnterState(oldState);
            }
            static inline void ratePeriodic() {
                const auto oldState = mState;
                ++mStateTick;
                switch(mState) {
                case State::Init:
                    mStateTick.on(initTicks, []{
                       mState = State::ReadFwHw;
                    });
                    break;
                case State::ReadFwHw:
                    mStateTick.on(stepTicks, []{
                        if (mReadErrorCount == 0) {
                            if (mReceivedPackets > 0) {
                                mState = State::SetAbsoluteMode;
                            }
                        }
                    });
                    break;
                case State::SetAbsoluteMode:
                    mStateTick.on(stepTicks, []{
                       mState = State::SetAbsoluteZero;
                    });
                    break;
                case State::SetAbsoluteZero:
                    mStateTick.on(stepTicks, []{
                       mState = State::AbsoluteZeroWait;
                    });
                    break;
                case State::AbsoluteZeroWait:
                    mStateTick.on(initTicks, []{
                       mState = State::SetMin;
                    });
                    break;
                case State::SetMin:
                    mStateTick.on(stepTicks, []{
                       mState = State::SetMax;
                    });
                    break;
                case State::SetMax:
                    mStateTick.on(stepTicks, []{
                       mState = State::SetSpeed;
                    });
                    break;
                case State::SetSpeed:
                    mStateTick.on(stepTicks, []{
                       mState = State::ReadPosition;
                    });
                    break;
                case State::ReadPosition:
                    mStateTick.on(stepTicks, []{
                       mState = State::SetDisableAutoReply;
                    });
                    break;
                case State::SetDisableAutoReply:
                    mStateTick.on(stepTicks, []{
                       mState = State::Run;
                    });
                    break;
                case State::Run:
                    mStateTick.on(telemTicks, []{
                        readPosition();
                    });
                    break;
                }
                exitEnterState(oldState);
            }
            static inline void offset(const uint16_t o) {
                mOffset = normalize<4096>(o);
            }
            static inline uint16_t offset() {
                return mOffset;
            }
            static inline int8_t turns() {
                return mTurns;
            }
            static inline int16_t absPos() {
                return (mTurns * 4096) + mLastPos;
            }
            static inline uint16_t speed() {
                return mSpeed;
            }
            static inline uint16_t lastPos() {
                return mLastPos;
            }
            static inline uint16_t startPos() {
                return mStartPos;
            }
            static inline int16_t actualPos() {
                return mActualPos;
            }
            static inline uint16_t errorCount() {
                return mReadErrorCount;
            }
            static inline uint16_t hwVersion() {
                return mHW;
            }
            static inline uint16_t fwVersion() {
                return mFW;
            }
            private:
            static inline void exitEnterState(const State oldState) {
                if (oldState != mState) {
                    mStateTick.reset();
                    switch(mState) {
                    case State::Init:
                        IO::outl<debug>("# WS: init");
                        break;
                    case State::ReadFwHw:
                        IO::outl<debug>("# WS: ReadFwHw");
                        mReceivedPackets = 0;
                        queryFwHw();
                        break;
                    case State::SetAbsoluteMode:
                        IO::outl<debug>("# WS: SetAbsMode");
                        setAboluteMode();
                        break;
                    case State::SetAbsoluteZero:
                        IO::outl<debug>("# WS: SetAbsZero");
                        position(0);
                        break;
                    case State::AbsoluteZeroWait:
                        IO::outl<debug>("# WS: AbsWait");
                        break;
                    case State::SetMin:
                        IO::outl<debug>("# WS: SetMin");
                        setMinAngle();
                        break;
                    case State::SetMax:
                        IO::outl<debug>("# WS: SetMax");
                        setMaxAngle();
                        break;
                    case State::SetSpeed:
                        IO::outl<debug>("# WS: SetSpeed ", mSpeed);
                        setSpeed(mSpeed);
                        break;
                    case State::ReadPosition:
                        IO::outl<debug>("# WS: ReadPos");
                        readPosition();
                        break;
                    case State::SetDisableAutoReply:
                        IO::outl<debug>("# WS: DisAutoReply");
                        disableAutoReply();
                        break;
                    case State::Run:
                        IO::outl<debug>("# WS: Run");
                        break;
                    }
                }
            }
            static inline void queryFwHw() {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x04;
                send(mId, 0x00, payload, CmdRead);
            }
            static inline void readPosition() {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x02; // len
                send(mId, 0x38, payload, CmdRead);
            }
            static inline void setMinAngle() {
                std::array<uint8_t, 2> payload;
                payload[0] = 0x00;
                payload[1] = 0x00;
                send(mId, 0x09, payload);
            }
            static inline void setMaxAngle() {
                std::array<uint8_t, 2> payload;
                payload[0] = 0x00;
                payload[1] = 0x00;
                send(mId, 0x0b, payload);
            }
            static inline void setSpeed(const uint16_t s = 2000) {
                std::array<uint8_t, 2> payload;
                payload[0] = s & 0xff;
                payload[1] = s >> 8;
                send(mId, 0x2e, payload);
            }
            static inline void setStepMode() {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x03; // step mode
                send(mId, 0x21, payload);
            }
            static inline void setAboluteMode() {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x00; // absolute mode
                send(mId, 0x21, payload);
            }
            static inline void disableAutoReply() {
                std::array<uint8_t, 1> payload;
                payload[0] = 0x00; // not auto reply
                send(mId, 0x08, payload);
            }
            static inline void positionMultiTurn(const uint16_t p) {
                std::array<uint8_t, 2> payload;
                const uint16_t po = normalize<4096>(p + mOffset);
                int16_t d = po - mLastPos;
                if (d >= 2048) {
                    mTurns -= 1;
                }
                else if (d <= -2048) {
                    mTurns += 1;
                }
                int32_t pa = std::clamp(po + mTurns * 4096, -6 * 4096, 6 * 4096);
                const uint16_t s = std::abs(pa);
                if (pa >= 0) {
                    payload[0] = s & 0xff;
                    payload[1] = s >> 8;
                    send(mId, 0x2a, payload);
                }
                else if (pa < 0) {
                    payload[0] = s & 0xff;
                    payload[1] = (s >> 8) | 0x80 ; // neg. direction
                    send(mId, 0x2a, payload);
                }
                mLastPos = po;
            }
            static inline void position(const uint16_t p) {
                std::array<uint8_t, 2> payload;
                payload[0] = p & 0xff;
                payload[1] = (p >> 8);
                send(mId, 0x2a, payload);
                mLastPos = p;
            }
            static inline void send(const uint8_t id, const uint8_t address, const auto& payload, const uint8_t cmd = CmdWrite) {
                uint8_t csum = 0;
                mCounter = 0;
                addToBuffer(0xff);
                addToBuffer(0xff);
                csum += addToBuffer(id);
                csum += addToBuffer(3 + payload.size()); // len
                csum += addToBuffer(cmd);
                csum += addToBuffer(address);
                for(uint8_t i = 0; i < payload.size(); ++i) {
                    csum += addToBuffer(payload[i]);
                }
                addToBuffer(~csum);
                uart::startSend(mCounter);
            }
            static inline void getActualAbsPos() {
                const uint8_t lb = uart::readBuffer()[5];
                const uint8_t hb = uart::readBuffer()[6];
                int16_t p = 0;
                if (hb & 0x80) {
                    p = -(((hb & 0x7f) << 8) + lb);
                }
                else {
                    p =  (((hb & 0x7f) << 8) + lb);
                }
                mActualPos = normalize(p - mOffset);
            }
            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t size) {
                if (data[0] != 0xff) {
                    return false;
                }
                if (data[1] != 0xff) {
                    return false;
                }
                const uint8_t len = data[3];
                if (len > size) {
                    return false;
                }
                return true;
            }
            static inline bool readReply() {
                [[maybe_unused]] Debug::Scoped<tp> tp;
                const uint8_t id = uart::readBuffer()[2];
                const uint8_t len = uart::readBuffer()[3];

                mLastError = uart::readBuffer()[4];

                uint8_t csum = id + len;
                for(uint8_t i = 0; i < len - 1; ++i) {
                    csum += uart::readBuffer()[4 + i];
                }
                if (uart::readBuffer()[3 + len] != (~csum & 0xff)) {
                    ++mReadErrorCount;
                    return false;
                }
                return true;
            }
            static inline uint8_t addToBuffer(const uint8_t v) {
                volatile uint8_t* const data = uart::outputBuffer();
                if (data) {
                    data[mCounter++] = v;
                }
                return v;
            }
            static inline void posVeloAcc(const uint16_t p, const uint16_t v = 500, const uint16_t a = 50) { // [172;992;1812]
                uint8_t csum = 0;
                volatile uint8_t* const data = uart::outputBuffer();
                data[0] = 0xff;
                data[1] = 0xff;
                csum += data[2] = 0x01; // id
                csum += data[3] = 2 + 7 + 1; // len = 7
                csum += data[4] = 0x03; // write
                csum += data[5] = 0x29; // acceleration address
                csum += data[6] = a; //acc
                csum += data[8] = p >> 8;
                csum += data[7] = p & 0xff;
                csum += data[9] = 0;
                csum += data[10] = 0;
                csum += data[12] = v >> 8;
                csum += data[11] = v & 0xff;
                data[13] = ~csum;
                uart::startSend(14);
            }
            static inline void ping() {
                uint8_t csum = 0;
                volatile uint8_t* const data = uart::outputBuffer();
                data[0] = 0xff;
                data[1] = 0xff;
                csum += (data[2] = mId); // id
                csum += data[3] = 0x02; // len
                csum += data[4] = 0x01; // ping
                data[5] = ~csum; // csum
                uart::startSend(6);
            }
            static inline bool mActive = false;
            static inline uint32_t mReadErrorCount = 0;
            static inline uint8_t mLastError = 0;
            static inline uint32_t mReceivedPackets = 0;
            static inline etl::Event<Event> mEvent;
            static inline State mState{State::Init};
            static inline uint16_t mFW = 0;
            static inline uint16_t mHW = 0;
            static inline External::Tick<systemTimer> mStateTick;
            static inline uint8_t mCounter = 0;
            static inline uint8_t mId = 1;
            static inline uint16_t mLastPos = 0;
            static inline uint16_t mStartPos = 0;
            static inline int16_t mActualPos = 0;
            static inline uint16_t mOffset = 0;
            static inline uint16_t mSpeed = 1000;
            static inline int32_t mSteps = 0;
            static inline int8_t mTurns = 0;
        };
    }
}


