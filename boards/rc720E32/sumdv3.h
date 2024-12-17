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
struct SumDV3Input {
    using clock = Config::clock;
    using systemTimer = Config::systemTimer;
    using debug = Config::debug;
    using dmaChRead = Config::dmaChRead;
    using pin = Config::pin;
    using tp = Config::tp;

    struct UartConfig {
        using Clock = clock;
        using ValueType = uint8_t;
        static inline constexpr size_t size = 43; // 41 + X buffer size
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

    struct Crc16 {
        inline void reset() {
            sum = 0;
        }
        inline uint8_t operator+=(const uint8_t v) {
            sum = sum ^ (((uint16_t)v) << 8);
            for(uint8_t i = 0; i < 8; ++i) {
                if (sum & 0x8000) {
                    sum = (sum << 1) ^ crc_polynome;
                }
                else {
                    sum = (sum << 1);
                }
            }
            return v;
        }
        inline operator uint16_t() const {
            return sum;
        }
    private:
        static constexpr uint16_t crc_polynome = 0x1021;
        uint16_t sum{};
    };


    static inline void init() {
        IO::outl<debug>("# SumDV3 init");
        __disable_irq();
        mState = State::Init;
        mErrorCount = 0;
        mEvent = Event::None;
        mActive = true;
        uart::init();
        uart::template txEnable<false>();
        uart::baud(115'200);
        uart::template halfDuplex<true>();
        __enable_irq();
        for(auto& v: mChannels) {
            v = 992;
        }
        pin::afunction(af);
        pin::template pullup<true>();
    }
    static inline void reset() {
        IO::outl<debug>("# SumDV3 reset");
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
                f();
                uart::rxClear();
                dmaChRead::count(UartConfig::size);
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

    enum class Frame : uint8_t {Ch1to12 = 0x00, First = Ch1to12,
                                Ch1to8and13to16 = 0x01, Ch1to16 = 0x02, Ch1to8and17to24 = 0x03,
                                Ch1to8and25to32 = 0x04, Ch1to12and64Switches = 0x05,
                                Last = Ch1to12and64Switches,
                                Undefined = 0xff};


    static inline void readReply() {
        const volatile uint8_t* const data = uart::readBuffer();
        uint8_t i = 0;
        Crc16 cs;
        if ((cs += data[i++]) != 0xa8) return;
        const uint8_t version = (cs += data[i++]) & 0x0f;
        const uint8_t nChannels = (cs += data[i++]);
        if (!((nChannels >= 2) && (nChannels <= 32))) return;
        for(uint8_t k = 0; k < (2 * nChannels); ++k) {
            cs += data[i++];
        }
        if (version == 0x01) {
            const uint8_t crcH = data[i++];
            const uint8_t crcL = data[i++];

            if constexpr(!std::is_same_v<tp, void>) {
                tp::set();
            }
            if (cs == ((uint16_t(crcH) << 8) | crcL)) {
                decodeV1(data + 3, nChannels);
            }
            if constexpr(!std::is_same_v<tp, void>) {
                tp::reset();
            }
        }
        else if (version == 0x03) {
            const uint8_t crcH = data[i++];
            const uint8_t crcL = data[i++];

            const uint8_t fcode = data[i - 6];
            [[maybe_unused]] const uint8_t reserved = data[i - 5];
            [[maybe_unused]] const uint8_t command = data[i - 4];
            [[maybe_unused]] const uint8_t subcmd = data[i - 3];

            if constexpr(!std::is_same_v<tp, void>) {
                tp::set();
            }
            if (cs == ((uint16_t(crcH) << 8) | crcL)) {
                decodeV3(data + 3, Frame{fcode}, nChannels);
            }
            if constexpr(!std::is_same_v<tp, void>) {
                tp::reset();
            }
        }
    }
    template<uint8_t B, uint8_t E> struct range_t {};
    template<uint8_t O> using offset_t = std::integral_constant<uint8_t, O>;

    static inline void decodeV1(const volatile uint8_t* const frame, const uint8_t) {
        extract(range_t<0, 15>{}, frame);
    }
    static inline void decodeV3(const volatile uint8_t* const fptr, const Frame frame, const uint8_t) {
        switch(frame) {
        case Frame::Ch1to12:
            extract(range_t<0, 11>{}, fptr);
            break;
        case Frame::Ch1to8and13to16:
            extract(range_t<0, 7>{}, fptr);
            extract(range_t<8, 11>{}, fptr, offset_t<12>{});
            break;
        case Frame::Ch1to8and17to24:
            extract(range_t<0, 7>{}, fptr);
            extract(range_t<8, 15>{}, fptr, offset_t<16>{});
            break;
        case Frame::Ch1to8and25to32:
            extract(range_t<0, 7>{}, fptr);
            extract(range_t<8, 15>{}, fptr, offset_t<24>{});
            break;
        case Frame::Ch1to16:
            extract(range_t<0, 15>{}, fptr);
            break;
        case Frame::Ch1to12and64Switches:
            extract(range_t<0, 11>{}, fptr);
            sumSwitches(fptr);
            break;
        case Frame::Undefined:
            break;
        }

    }
    template<uint8_t Begin, uint8_t End, typename F, uint8_t Off = 0>
    static inline void extract(const range_t<Begin, End>&, const F frame, offset_t<Off> = offset_t<0>{}) {
        uint8_t out{Off};
        for(uint8_t i = Begin; i <= End; ++i) {
            uint16_t raw = (frame[2 * i] << 8) | frame[2 * i + 1];
            mChannels[out++] = raw2sbus(raw);
        }
    }
    static constexpr uint16_t MinValue = 0x2260; // 8800
    static constexpr uint16_t CenterValue = 0x2ee0; // 12000
    static constexpr uint16_t MaxValue = 0x3b60; // 15200

    static inline constexpr uint16_t sbmax = 1812;
    static inline constexpr uint16_t sbmin = 172;

    static inline uint16_t raw2sbus(const uint16_t r) {
        const int rc = std::clamp(r, MinValue, MaxValue);
        const int sb = ((uint32_t)(rc - MinValue) * (sbmax - sbmin)) / (MaxValue - MinValue) + sbmin;

        return sb;
    }


    static inline void sumSwitches(const volatile uint8_t* const fptr) {
        const volatile uint8_t* sptr = fptr + 24;
        uint64_t sw = *sptr++;
        sw = (sw << 8) | *sptr++;
        sw = (sw << 8) | *sptr++;
        sw = (sw << 8) | *sptr++;
        sw = (sw << 8) | *sptr++;
        sw = (sw << 8) | *sptr++;
        sw = (sw << 8) | *sptr++;
        sw = (sw << 8) | *sptr++;
        mSwitches = sw;
        // uint64_t diff = lastSwitches ^ sw;
        // lastSwitches = sw;
    }

    static inline uint64_t mSwitches;
    static inline volatile bool mActive = false;
    static inline std::array<uint16_t, 32> mChannels; // sbus
    static inline uint16_t mErrorCount = 0;
    static inline volatile Event mEvent = Event::None;
    static inline volatile State mState = State::Init;
    static inline External::Tick<systemTimer> mStateTick;
};

