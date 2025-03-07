/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "mcu/alternate.h"

#include "usart.h"
#include "output.h"
#include "etl/util.h"

using namespace std::literals::chrono_literals;

template<uint8_t N, typename Config, typename MCU = DefaultMcu>
struct WaveShare {
    using pin = Config::pin;
    using polar = Config::polar;
    using clock = Config::clk;
    using debug = Config::dbg;
    using tp = Config::tp;
    using dmaChRW = Config::dmaChRW;
    using systemTimer = Config::timer;
    using storage = Config::storage;

    struct UartConfig {
        using Clock = clock;
        using ValueType = uint8_t;
        static inline constexpr size_t size = 64; // ???
        static inline constexpr size_t minSize = 4;
        using DmaChannelWrite = dmaChRW;
        using DmaChannelRead = dmaChRW;
        static inline constexpr bool useDmaTCIsr = false;
        static inline constexpr bool useIdleIsr = false;
        static inline constexpr bool useRxToIsr = false;
        static inline constexpr uint16_t rxToCount = 0;
        using Adapter = void;
        using Debug = struct {
        using tp = void;
    };
};

using dbg = debug;

using uart = Mcu::Stm::V2::Uart<N, UartConfig, MCU>;

static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;

static inline void init() {
    IO::outl<debug>("# WS init");
    mState = State::Init;
    mEvent = Event::None;
    mReadErrorCount = 0;
    mReceivedPackets = 0;
    mNextReplyLength = 0;
    mLastError = 0;
    mStateTick.reset();

    __disable_irq();
    uart::init();
    uart::template rxEnable<false>();
    uart::baud(1'000'000);
    uart::template halfDuplex<true>();
    uart::template enableTCIsr<true>();
    __enable_irq();
    pin::afunction(af);
    pin::template pullup<true>();
}
static inline void reset() {
    IO::outl<debug>("# WS reset");
    __disable_irq();
    dmaChRW::enable(false);
    uart::reset();
    __enable_irq();
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
                            // SetStepMode,
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
static inline void rxEnable() {
    uart::dmaReenable([]{
        dmaChRW::clearTransferCompleteIF();
        dmaChRW::template setTCIsr<true>();
        uart::dmaSetupRead2(std::min((size_t)mNextReplyLength, UartConfig::size));
    });
    uart::template rxEnable<true>();
}

static inline void periodic() {
    const auto oldState = mState;
    switch(mState) {
    case State::Init:
        break;
    case State::ReadFwHw:
        if (std::exchange(mEvent, Event::None) == Event::ReadReply) {
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
        if (std::exchange(mEvent, Event::None) == Event::ReadReply) {
            mState = State::SetAbsoluteZero;
        }
        break;
    case State::SetAbsoluteZero:
        if (std::exchange(mEvent, Event::None) == Event::ReadReply) {
            mState = State::AbsoluteZeroWait;
        }
        break;
    case State::AbsoluteZeroWait:
        break;
        // case State::SetStepMode:
        //     if (std::exchange(mEvent, Event::None) == Event::ReadReply) {
        //         mState = State::SetMin;
        //     }
        //     break;
    case State::SetMin:
        if (std::exchange(mEvent, Event::None) == Event::ReadReply) {
            mState = State::SetMax;
        }
        break;
    case State::SetMax:
        if (std::exchange(mEvent, Event::None) == Event::ReadReply) {
            mState = State::SetSpeed;
        }
        break;
    case State::SetSpeed:
        if (std::exchange(mEvent, Event::None) == Event::ReadReply) {
            if (readReply()) {
                mState = State::ReadPosition;
                ++mReceivedPackets;
            }
        }
        break;
    case State::ReadPosition:
        if (std::exchange(mEvent, Event::None) == Event::ReadReply) {
            if (readReply()) {
                mLastPos = (uart::readBuffer()[6] << 8) + uart::readBuffer()[5];
                mStartPos = mLastPos;
                ++mReceivedPackets;
            }
            mState = State::SetDisableAutoReply;
        }
        break;
    case State::SetDisableAutoReply:
        // no reply
        break;
    case State::Run:
        if (std::exchange(mEvent, Event::None) == Event::ReadReply) {
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
        // case State::SetStepMode:
        //     mStateTick.on(stepTicks, []{
        //        mState = State::SetMin;
        //     });
        //     break;
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
static inline void exitEnterState(const State oldState) {
    if (oldState != mState) {
        mStateTick.reset();
        switch(mState) {
        case State::Init:
            IO::outl<dbg>("# WS: init");
            break;
        case State::ReadFwHw:
            IO::outl<dbg>("# WS: ReadFwHw");
            mReceivedPackets = 0;
            queryFwHw();
            break;
        case State::SetAbsoluteMode:
            IO::outl<dbg>("# WS: SetAbsMode");
            setAboluteMode();
            break;
        case State::SetAbsoluteZero:
            IO::outl<dbg>("# WS: SetAbsZero");
            position(0);
            break;
        case State::AbsoluteZeroWait:
            IO::outl<dbg>("# WS: AbsWait");
            break;
            // case State::SetStepMode:
            //     IO::outl<dbg>("# WS: SetStepMode");
            //     setStepMode();
            //     break;
        case State::SetMin:
            IO::outl<dbg>("# WS: SetMin");
            setMinAngle();
            break;
        case State::SetMax:
            IO::outl<dbg>("# WS: SetMax");
            setMaxAngle();
            break;
        case State::SetSpeed:
            IO::outl<dbg>("# WS: SetSpeed ", mSpeed);
            setSpeed(mSpeed);
            break;
        case State::ReadPosition:
            IO::outl<dbg>("# WS: ReadPos");
            readPosition();
            break;
        case State::SetDisableAutoReply:
            IO::outl<dbg>("# WS: DisAutoReply");
            disableAutoReply();
            break;
        case State::Run:
            IO::outl<dbg>("# WS: Run");
            break;
        }
    }
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
static inline void queryFwHw() {
    std::array<uint8_t, 1> payload;
    const uint8_t len = payload[0] = 0x04;
    send(mId, 0x00, payload, CmdRead, len + 6);
}
static inline void readPosition() {
    std::array<uint8_t, 1> payload;
    const uint8_t len = payload[0] = 0x02;
    send(mId, 0x38, payload, CmdRead, len + 6);
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
static inline void steps(const uint16_t p) {
    std::array<uint8_t, 2> payload;
    const uint16_t po = normalize<4096>(p + mOffset);
    int16_t d = po - (mSteps % 4096);
    // int16_t d = po - mLastPos;
    if (d >= 2048) {
        d -= 4096;
    }
    else if (d <= -2048) {
        d += 4096;
    }
    const uint16_t s = std::abs(d) & 0xfff;
    if (d > 0) {
        payload[0] = s & 0xff;
        payload[1] = s >> 8;
        send(mId, 0x2a, payload);
    }
    else if (d < 0) {
        payload[0] = s & 0xff;
        payload[1] = (s >> 8) | 0x80 ; // neg. direction
        send(mId, 0x2a, payload);
    }
    mSteps += d;
    // mLastPos = po;
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

static inline void send(const uint8_t id, const uint8_t address, const auto& payload, const uint8_t cmd = CmdWrite, const uint8_t replyLength = 6) {
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
    mNextReplyLength = replyLength;
    uart::template rxEnable<false>();
    uart::startSend(mCounter);
}
// static inline void getActualPos() {
//     const uint8_t lb = uart::readBuffer()[5];
//     const uint8_t hb = uart::readBuffer()[6];
//     int16_t d = 0;
//     if (hb & 0x80) {
//         d = -(((hb & 0x7f) << 8) + lb);
//     }
//     else {
//         d =  (((hb & 0x7f) << 8) + lb);
//     }
//     mActualPos = normalize(mSteps % 4096 - mOffset - d);
//     // mActualPos = normalize(mLastPos - mOffset - d);
// }
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

static inline bool readReply() {
    if constexpr(!std::is_same_v<tp, void>) {
        tp::set();
    }
    if (uart::readBuffer()[0] != 0xff) {
        ++mReadErrorCount;
        return false;
    }
    if (uart::readBuffer()[1] != 0xff) {
        ++mReadErrorCount;
        return false;
    }
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
    if constexpr(!std::is_same_v<tp, void>) {
        tp::reset();
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

static inline uint8_t mNextReplyLength = 6;

static inline uint32_t mReadErrorCount = 0;
static inline uint8_t mLastError = 0;
static inline uint32_t mReceivedPackets = 0;

static inline Event mEvent{Event::None};
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
