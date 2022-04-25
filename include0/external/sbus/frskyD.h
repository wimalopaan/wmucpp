#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <chrono>

#include <etl/ranged.h>
#include <etl/algorithm.h>

#include <external/solutions/tick.h>

#include <mcu/internals/usart.h>

namespace External {
    namespace FrskyD {
        
        enum class ID : uint8_t {GPS = 0x01, Temp1 = 0x02, Rpm = 0x03, Fuel = 0x04, Temp2 = 0x05, Volt = 0x06, Alt = 0x10,
                                 DIY1 = 0x2a, DIY2 = 0x2b, DIY3 = 0x2c, DIY4 = 0x2d, DIY5 = 0x2e, DIY6 = 0x2f};
        
        namespace Output {
            using namespace std::literals::chrono;
            
            template<typename CN, typename Timer, typename dbg = void, uint8_t Size = 6>
            struct Generator {
                static inline constexpr External::Tick<Timer> timeoutTicks{50_ms};
                static_assert(timeoutTicks.value > 1);
//                std::integral_constant<uint8_t, timeoutTicks.value>::_;

                static inline constexpr std::byte user_packet= 0xfd_B;
                static inline constexpr std::byte start_byte = 0x5e_B;
                static inline constexpr std::byte length = std::byte{4};

                static constexpr uint8_t bufferLength = 10;
                
                using usart = AVR::Usart<CN, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<bufferLength>>;
                
                inline static void init() {
                    usart::template init<AVR::BaudRate<9600>>();
                }

                static inline void set(const uint8_t index, const ID id, const uint16_t v) {
                    data[index][0] = std::byte{id};
                    data[index][1] = std::byte(v);
                    data[index][2] = std::byte(v >> 8);
                }

                inline static void periodic() {
                    usart::periodic();
                }
                
                inline static void ratePeriodic() { // 14ms
                    (++ticks).on(timeoutTicks, []{
                        if constexpr(!std::is_same_v<dbg, void>) {
                            dbg::toggle();
                        }
                        putNextPacket();
                        ++mPacketIndex;
                    });
                }
                inline static void putNextPacket() {
                    usart::put(user_packet);
                    usart::put(length);
                    usart::put(0x00_B);
                    usart::put(0x5e_B);
                    const auto& p = data[mPacketIndex.toInt()];
                    usart::put(p[0]);
                    byteStuff(p[1]);
                    byteStuff(p[2]);
                }
                inline static void byteStuff(const std::byte b) {
                    if (b == 0x5e_B) {
                        usart::put(0x5d_B);
                        usart::put(0x3e_B);
                    }
                    else if (b == 0x5d_B) {
                        usart::put(0x5d_B);
                        usart::put(0x3d_B);
                    }
                    else {
                        usart::put(b);
                    }
                }
            private:
                static inline etl::uint_ranged_circular<uint8_t, 0, Size - 1> mPacketIndex;
                static inline std::array<std::array<std::byte, 3>, Size> data{};
                static inline External::Tick<Timer> ticks{};
            };          
        }
    }
}
