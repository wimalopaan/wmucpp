#pragma once

#include <external/solutions/tick.h>

namespace External {
    namespace SPort {
        // https://github.com/jcheger/frsky-arduino/tree/master/FrskySP
        
        using namespace AVR;
        using namespace std::literals::chrono;
        using namespace External::Units::literals;
        
        enum class SensorId : uint8_t { ID1  = 0x00, ID2  = 0xA1, ID3  = 0x22, ID4  = 0x83, ID5  = 0xE4, ID6  = 0x45, ID7  = 0xC6,
                                        ID8  = 0x67, ID9  = 0x48, ID10 = 0xE9, ID11 = 0x6A, ID12 = 0xCB, ID13 = 0xAC, ID14 = 0x0D,
                                        ID15 = 0x8E, ID16 = 0x2F, ID17 = 0xD0, ID18 = 0x71, ID19 = 0xF2, ID20 = 0x53, ID21 = 0x34,
                                        ID22 = 0x95, ID23 = 0x16, ID24 = 0xB7, ID25 = 0x98, ID26 = 0x39, ID27 = 0xBA, ID28 = 0x1B, ID_IGNORE = 0xFF };
        
        enum class ValueId : uint16_t {
            Current = 0x0200, // 15: 0,1A
            Voltage = 0x0210, // 15: 0,01V
            Cells   = 0x0300, // 15: Cellformat
            Temp1   = 0x0400, // 15: °C
            Temp2   = 0x0410, // 15: °C
            Rpm     = 0x0500, // 15: Rpm
            Speed   = 0x0830, // 15: Knoten 0,001Kn
        };

        template<SensorId ID, template<typename> typename Uart, typename Timer, typename ProviderList = Meta::List<>>
        struct Sensor;
        
        template<SensorId ID, template<typename> typename Uart, typename Timer, typename... Providers>
        struct Sensor<ID, Uart, Timer, Meta::List<Providers...>> {
            inline static constexpr External::Tick<Timer> mResponseDelay{2_ms};
            //            std::integral_constant<uint16_t, mResponseDelay.value>::_;
            
            using providerList = Meta::List<Providers...>;
            inline static constexpr auto numberOfProviders = sizeof...(Providers);
//            std::integral_constant<uint8_t, numberOfProviders>::_;
            
            using index_type = etl::uint_ranged_circular<uint8_t, 0, numberOfProviders - 1>;
            
            enum class State : uint8_t {Init, Request, ReplyWait, Reply, WaitReplyComplete};
            
            struct ProtocollAdapter {
                static inline bool process(const std::byte b) {
                    switch(mState) {
                    case State::Init: 
                        if (b == 0x7e_B) {
                            mState = State::Request;
                        }
                        break;
                    case State::Request: 
                        if (b == std::byte{ID}) {
                            mRequests = mRequests + 1;
                            mStateTicks.reset();
                            mState = State::ReplyWait;
                        }
                        else {
                            mState = State::Init;
                        }
                        break;
                    default:
                        break;
                    }
                    return true;
                }
                inline static uint8_t requests() {
                    return mRequests;
                }
            private:
                static inline volatile uint8_t mRequests{};
            };
            
            using uart = Uart<ProtocollAdapter>;   
            static_assert(uart::sendQLength >= 16);
            
            inline static constexpr bool useInterrupts = uart::useInterrupts;
            using tick_type = std::conditional_t<useInterrupts, volatile External::Tick<Timer>, External::Tick<Timer>>;
            using state_type = std::conditional_t<useInterrupts, volatile State, State>;
            
            static inline void init() {
                uart::template init<AVR::HalfDuplex>();
            }
            static inline void periodic() {
            }
            static inline void ratePeriodic() {
                const auto lastState = mState;
                ++mStateTicks;
                switch(mState) {
                case State::ReplyWait:
                    mStateTicks.on(mResponseDelay, []{
                        mState = State::Reply;    
                        uart::template rxEnable<false>();
                    });
                    break;
                case State::Reply:
                    reply();
                    mState = State::WaitReplyComplete;
                    break;
                case State::WaitReplyComplete:
                    if (uart::isIdle()) {
                        uart::template rxEnable<true>();
                        mState = State::Init;
                    }
                    break;
                default:
                    break;
                }
                if (lastState != mState) {
                    mStateTicks.reset();
                }
            }
        private:
            struct CheckSum {
                void operator+=(const std::byte b) {
                    mValue += uint8_t(b);
                    mValue += mValue >> 8;
                    mValue &= 0x00ff;
                }
                std::byte value() const {
                    return etl::nth_byte<0>(0xff - mValue);
                }
            private:
                uint16_t mValue{};
            };
            inline static void reply() {
                CheckSum cs;
                stuffResponse(0x10_B, cs);
                
                Meta::visitAt<providerList>(mActualProvider, [&]<typename P>(Meta::Wrapper<P>){
                                                auto id = ValueId(uint16_t(P::valueId) + mActualProvider);
                                                stuff(id, cs);
                                                stuff(P::value(), cs);
                              });
                stuff(cs);
                
                ++mActualProvider;
            }
            inline static void stuff(const CheckSum& cs) {
                uart::put(cs.value());
            }
            inline static void stuff(const uint32_t b, CheckSum& cs) {
                stuffResponse(etl::nth_byte<0>(b), cs);
                stuffResponse(etl::nth_byte<1>(b), cs);
                stuffResponse(etl::nth_byte<2>(b), cs);
                stuffResponse(etl::nth_byte<3>(b), cs);
            }
            inline static void stuff(const ValueId b, CheckSum& cs) {
                using t = std::underlying_type<ValueId>::type;
                stuffResponse(etl::nth_byte<0>(static_cast<t>(b)), cs);
                stuffResponse(etl::nth_byte<1>(static_cast<t>(b)), cs);
            }
            inline static void stuffResponse(const std::byte b, CheckSum& cs) {
                if (b == 0x7e_B) {
                    cs += 0x7d_B;
                    uart::put(0x7d_B);
                    cs += 0x5d_B;
                    uart::put(0x5e_B);
                }
                else {
                    cs += b;
                    uart::put(b);
                }
            }
        private:            
            inline static index_type mActualProvider;
            inline static tick_type mStateTicks;
            inline static state_type mState{State::Init};
        };
    }
}
