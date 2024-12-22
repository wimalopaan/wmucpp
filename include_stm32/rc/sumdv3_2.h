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
#include "rc/rc_2.h"

namespace RC::Protokoll::SumDV3 {
    namespace V2 {
        template<uint8_t N, typename Config, typename MCU = DefaultMcu>
        struct Input {
            using clock = Config::clock;
            using systemTimer = Config::systemTimer;
            using debug = Config::debug;
            using pin = Config::pin;
            using tp = Config::tp;

            struct UartConfig {
                using Clock = clock;
                using ValueType = uint8_t;
                using Adapter = void;
                static inline constexpr bool rxtxswap = true;
                static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::RxOnly;
                static inline constexpr uint32_t baudrate = RC::Protokoll::Hott::SumDV3::V2::baudrate;
                using DmaChComponent = Config::dmaChComponent;
                struct Rx {
                    static inline constexpr bool enable = true;
                    static inline constexpr size_t size = 64;
                    static inline constexpr size_t idleMinSize = 4;
                };
                struct Isr {
                    static inline constexpr bool idle = true;
                };
                using tp = Config::tp;
            };
            using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

            static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;

            static inline void init() {
                IO::outl<debug>("# SumDV3 init");
                for(auto& v: mChannels) {
                    v = RC::Protokoll::SBus::V2::mid;
                }
                Mcu::Arm::Atomic::access([]{
                    uart::init();
                    mState = State::Init;
                    mErrorCount = 0;
                    mEvent = Event::None;
                    mActive = true;
                });
                pin::afunction(af);
                pin::template pullup<true>();
            }
            static inline void reset() {
                IO::outl<debug>("# SumDV3 reset");
                Mcu::Arm::Atomic::access([]{
                    mActive = false;
                    uart::reset();
                });
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
                    if (mEvent.is(Event::ReceiveComplete)) {
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
                        IO::outl<debug>("# SumDV3 Run");
                        uart::template rxEnable<true>();
                        break;
                    }
                }
            }
            static inline void update() {
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
            };
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


            static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t) {
                if (data[0] != 0xa8) {
                    return false;
                }
                return true;
            }
            static inline void readReply() {
                const volatile uint8_t* const data = uart::readBuffer();
                uint8_t i = 0;
                RC::Protokoll::Hott::SumDV3::V2::Crc16 cs;
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
            static inline volatile etl::Event<Event> mEvent;
            static inline volatile State mState = State::Init;
            static inline External::Tick<systemTimer> mStateTick;
        };
    }
}


