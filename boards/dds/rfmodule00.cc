#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/spi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>
#include <mcu/internals/syscfg.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/apa102.h>
#include <external/solutions/series01/sppm_in.h>
#include <external/solutions/rc/busscan.h>
#include <external/solutions/series01/cppm_out.h>
#include <external/solutions/rc/rf.h>

#include <mcu/pgm/pgmarray.h>

#include <std/chrono>
#include <std/bit>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

#ifndef NDEBUG
namespace xassert {
    etl::StringBuffer<160> ab;
    etl::StringBuffer<10> aline;
    bool on{false};
}
#endif

namespace External {
    namespace Multiprotocol {
        template<typename Dev, typename Timer>
        struct Generator {
            
//            using usart = AVR::Usart<CN, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<128>>;

            static inline constexpr External::Tick<Timer> msendTicks{500_ms};
            
            inline static constexpr uint16_t sbus_min = 172;
            inline static constexpr uint16_t sbus_max = 1811;
            
            inline static constexpr uint16_t sbus_mid = (sbus_max + sbus_min) / 2;
            
            using value_type = etl::uint_ranged<uint16_t, sbus_min, sbus_max>;
            
            inline static void init() {
                output[0] = std::byte{'M'};
                output[1] = std::byte{'P'};
                output[2] = 0x01_B; // status
                output[3] = 24_B;
                output[4] = 0x06_B;
                output[5] = 0x00_B;
                output[6] = 0x01_B;
                output[7] = 0x01_B;
                output[8] = 0x00_B;
                output[9] = 0x17_B; // AETR
                output[10] = 0x00_B;
                output[11] = 0x00_B;
                output[12] = std::byte{'W'};
                output[13] = std::byte{'M'};
                output[14] = std::byte{'\0'};
                output[19] = std::byte{8<<4 | 0x01};
                output[20] = std::byte{'4'};
                output[21] = std::byte{'0'};
                output[22] = std::byte{'/'};
                output[23] = std::byte{'5'};
                output[24] = std::byte{'0' + ch};
            }

            inline static void periodic() {
            }
            
            inline static void ratePeriodic() { 
                ++mSendTick;
                mSendTick.on(msendTicks, []{
                    output[24] = std::byte{'0' + ch};
                    for(const std::byte& b : output) {
                        Dev::put(b);
                    }
                });
            }
//            private:
            static inline std::array<std::byte, 28> output{};                 
            static inline External::Tick<Timer> mSendTick{};
            static inline uint8_t ch{0};
        };  

        
        template<auto N, typename Timer, typename Dbg = void>
        struct ProtocollAdapter {
            enum class State : uint8_t {Undefined, Data, GotStart, GotProto, GotSub};

            using data_t = std::array<uint16_t, 16>; 
            using value_type = etl::uint_ranged_NaN<uint16_t, 172, 1810>;
            using mapped_type = etl::uint_ranged_NaN<uint16_t, 480, 1504>;
            using channel_t = etl::uint_ranged_NaN<uint8_t, 0, data_t::size() - 1>;
            
            static inline value_type value(const channel_t ch) {
                if (const uint8_t chi = ch.toInt(); ch) {
                    return mChannels[chi];
                }
                return value_type{};
            }
            static inline mapped_type valueMapped(const channel_t ch) {
                if (const uint8_t chi = ch.toInt(); ch) {
                    return mChannels[chi];
                }
                return mapped_type{};
            }

            static_assert(Timer::frequency >= 1000_Hz);
            
            static inline uint8_t tc{0};
            static inline void ratePeriodic() {
                if (++tc > 5) {
                    mState = State::Undefined;
                }
            }
            
            static inline bool process(const std::byte b) {
                ++cc;
                tc = 0;
                switch(mState) {
                case State::Undefined:
                    if (b == 0x54_B) {
                        mState = State::GotStart;
                    }
                    break;
                case State::GotStart:
                    if ((b & 0x0F_B) == 0x05_B) { // Corona
                        mState = State::GotProto;
                    }
                    break;
                case State::GotProto:
                    if (((b >> 4) & 0x07_B) == 0x00_B) { // subproto 0
                        mState = State::GotSub;
                    }
                    break;
                case State::GotSub:
                    if (const int8_t v = static_cast<int8_t>(b); (v >= 0) && (v <= 40)) { 
                        mState = State::Data;
                        mIndex.setToBottom();
                        rfChannel = v;
                    }
                    break;
                case State::Data:
                    mData[mIndex] = std::to_integer(b);
                    if (mIndex.isTop()) {
                        mState = State::Undefined;
                        decode();
                        ++mPackages;
                    }
                    else {
                        ++mIndex;
                    }
                    break;
                }
                return true;
            }
            inline static uint16_t packages() {
                return mPackages;
            }
            inline static void resetStats() {
                mPackages = 0;
            }
//        private:
            static inline void decode() {
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
            }

            using MesgType = std::array<uint8_t, 23>;
            inline static data_t mChannels;
            inline static State mState{State::Undefined};
            inline static MesgType mData; 
            inline static etl::index_type_t<MesgType> mIndex;
            inline static uint16_t mPackages{};
            inline static uint16_t cc{};
            
            inline static uint8_t rfChannel{0};
        };
    }
}


namespace  {
    constexpr auto fRtc = 1000_Hz;
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

//using dbgPin = Pin<Port<A>, 6>;

//using a0Pin = Pin<Port<A>, 0>; // usart0
//using a1Pin = Pin<Port<A>, 1>;
//using a2Pin = Pin<Port<A>, 2>;

using a5Pin = Pin<Port<A>, 5>;

using dataPin = Pin<Port<C>, 0>;
using dataAct = ActiveHigh<dataPin, Output>;

using clkPin = Pin<Port<C>, 2>;
using clkAct = ActiveHigh<clkPin, Output>;
using clkSig = SinglePulse<clkAct>;

using fupPin = Pin<Port<D>, 0>;
using fupAct = ActiveHigh<fupPin, Output>;
using fupSig = SinglePulse<fupAct>;

using rstPin = Pin<Port<D>, 1>;
using rstAct = ActiveHigh<rstPin, Output>;
using rstSig = SinglePulse<rstAct>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>; 
using tdev = Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<16>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<tdev>;

using tca0Position = AVR::Portmux::Position<AVR::Component::Tca<0>, Portmux::Default>;
using cppm = External::Ppm::Cppm<tca0Position, std::integral_constant<uint8_t, 16>, AVR::UseInterrupts<true>>;


using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; 
using mpm_pa = External::Multiprotocol::ProtocollAdapter<0, systemTimer>;
using mpm = Usart<usart2Position, mpm_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using mpm_out = External::Multiprotocol::Generator<mpm, systemTimer>;

using lut0Position = AVR::Portmux::Position<AVR::Component::Ccl<0>, Portmux::Alt1>;
using lut0 = Ccl::SimpleLut<0, Ccl::Input::Mask, Ccl::Input::Mask, Ccl::Input::Usart<2>>;    

using portmux = Portmux::StaticMapper<Meta::List<tca0Position, lut0Position>>;

namespace External {
    namespace DDS {
        
        template<typename Timer, AVR::Concepts::ActivatableOut DPin, AVR::Concepts::Pulseable WClkSig, AVR::Concepts::Pulseable FUpdSig, AVR::Concepts::Pulseable ResetSig>
        struct AD9851 {
            static inline constexpr uint64_t ad_c = uint64_t{1} << 32;
            static inline constexpr uint64_t ad_sysclk = 30'000'000 * 6; // 180MHz bei 30MHz Quarz
            static inline constexpr uint16_t fOffset = 2'000;
            static inline constexpr External::Tick<systemTimer> mChangeTicks{100_ms};
            
            struct SetBaseFrequency {
                static inline void once() {
                    put(mData);
                    FUpdSig::pulse();
                }
                static inline std::array<std::byte, 4> mData;
            };
            struct SetUpperFrequency {
                static inline void once() {
                    put(mData);
                    FUpdSig::pulse();
                }
                static inline std::array<std::byte, 4> mData;
            };

        private:            
            struct FreqWord {
                static inline constexpr uint32_t dw(const uint32_t f) {
                    const uint32_t v = (f * ad_c) / ad_sysclk;
                    return v;                                        
                }
                constexpr FreqWord(const uint32_t f) :
                    data{etl::nth_byte<3>(dw(f)), etl::nth_byte<2>(dw(f)), etl::nth_byte<1>(dw(f)), etl::nth_byte<0>(dw(f))}
                {}
                constexpr FreqWord(auto it) :
                    data{*it++, *it++, *it++, *it++}
                {}
                const std::array<std::byte, 4> data{};
            } __attribute__((packed));
            
            struct ChannelData {
                constexpr ChannelData(const uint32_t f, const uint8_t b, const uint8_t ch) : 
                    f0{f}, f1{f + fOffset}, band{b}, ch{ch} {}
                
                ChannelData(const AVR::Pgm::Ptr<ChannelData>& ptr) :
                    f0{AVR::Pgm::ByteRange{ptr}.begin()}, f1{AVR::Pgm::ByteRange<ChannelData, sizeof(f0)>{ptr}.begin()},
                    band{pgm_read_byte(ptr.raw() + 8)}, ch{pgm_read_byte(ptr.raw() + 9)}
                {}
                const FreqWord f0;
                const FreqWord f1;
                const uint8_t band;
                const uint8_t ch;
            } __attribute__((packed));
            
            struct Generator {
                constexpr auto operator()() {
                    return []<auto... II>(std::index_sequence<II...>){
                        return std::array<ChannelData, RC::channels.size()>{ChannelData{RC::channels[II].mFreq, (uint8_t)RC::channels[II].mBand, (uint8_t)RC::channels[II].mNumber}...};
                    }(std::make_index_sequence<RC::channels.size()>{});
                }
            };
            
            using PgmChData = AVR::Pgm::Util::Converter<Generator>::pgm_type;

            enum class State : uint8_t {Undefined, Init, Ready};
            
        public:
            
            static inline bool ready() {
                return mState == State::Ready;
            }
            static inline void init() {
                DPin::init();
                WClkSig::init();
                FUpdSig::init();
                ResetSig::init();
            }  
            
            using index_type = PgmChData::ranged_type ;
            
            static inline void channel(const index_type& i) {
                etl::copy(SetBaseFrequency::mData, PgmChData::value(i).f0.data);
                etl::copy(SetUpperFrequency::mData, PgmChData::value(i).f1.data);
            }
            
            static inline void periodic() {
            }
            
            static inline void ratePeriodic() {
                const auto oldState = mState;
                ++mStateTick;
                switch (mState) {
                case State::Undefined:
                    mStateTick.on(mChangeTicks, []{
                        mState = State::Init;
                    });
                    break;
                case State::Init:
                    mStateTick.on(mChangeTicks, []{
                        mState = State::Ready;
                    });
                    break;
                case State::Ready:
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch (mState) {
                    case State::Undefined:
                        break;
                    case State::Init:
                        ResetSig::pulse();
                        setSerialMode();
                        break;
                    case State::Ready:
                        break;
                    }
                }
            }
        private:
            
            // evtl SPI (mit Interrupt) mit Double Clk und Buffered Mode?
            
            static inline void put(std::byte b) {
                for(uint8_t i{}; i < 8; ++i) {
                    if (std::any(b & 0x01_B)) {
                        DPin::activate();
                    }
                    else {
                        DPin::inactivate();
                    }
                    b >>= 1;
                    WClkSig::pulse();
                }
            }
            // Reihenfolge gegenüber parallel Mode vertauscht.
            static inline void put(const std::array<std::byte, 4>& a) {
                put(a[3]);
                put(a[2]);
                put(a[1]);
                put(a[0]);
                put(0x01_B); // refclock 6x
            }
            static inline void setSerialMode() {
                DPin::inactivate();
                WClkSig::pulse();
                FUpdSig::pulse();
            }
            static inline External::Tick<systemTimer> mStateTick;
            static inline State mState{State::Undefined};
//            static inline index_type mIndex;
        };
    }
}

using ad9851 = External::DDS::AD9851<systemTimer, dataAct, clkSig, fupSig, rstSig>;

template<typename MCU = DefaultMcuType>
struct Fsm {
    using ch_t = cppm::channel_t;
    using rv_t = cppm::ranged_type;
    
    enum class State : uint8_t {Undefined, Init, Set, Run};
    
    static inline constexpr External::Tick<systemTimer> mInitTicks{500_ms};
    static inline constexpr External::Tick<systemTimer> mChangeTicks{100_ms};
    static inline constexpr External::Tick<systemTimer> mDebugTicks{500_ms};
    
    static inline void init() {
//        dbgPin::dir<Output>();
        a5Pin::dir<Output>();
        ad9851::init();
        tdev::init<BaudRate<9600>>();
        mpm::init<BaudRate<100000>>();
        mpm::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>();
        mpm_out::init();
        lut0::init(0x0f_B); // route TXD (inverted) to lut0-out 
//        lut0::enable();
//        lut0::on();
        
//        mpm::txPinDisable();
        
        cppm::set(ch_t{0}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{1}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{2}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{3}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{4}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{5}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{6}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{7}, rv_t{cppm::ocMedium});
        
        etl::outl<terminal>("dds10"_pgm);
    }
    static inline void periodic() {
//        dbgPin::toggle();
        tdev::periodic();
        ad9851::periodic();
        mpm::periodic();
        mpm_out::periodic();
    }
    static inline void ratePeriodic() {
        ad9851::ratePeriodic();
        mpm_pa::ratePeriodic();
        mpm_out::ratePeriodic();
        const auto oldState = mState;
        ++mStateTick;
        ++mChangeTick;
        (++mDebugTick).on(mDebugTicks, []{
            etl::outl<terminal>("rf: "_pgm, mpm_pa::rfChannel, " v0: "_pgm, mpm_pa::value(0).toInt());
            mpm_out::ch = mpm_pa::rfChannel;
            ad9851::channel(ad9851::index_type{mpm_pa::rfChannel});
        });
        switch(mState) {
        case State::Undefined:
            mStateTick.on(mInitTicks, []{
               mState = State::Init; 
            });
            break;
        case State::Init:
            mStateTick.on(mInitTicks, []{
               mState = State::Set; 
            });
            break;
        case State::Set:
            if (ad9851::ready()) {
                mState = State::Run;
            }
            break;
        case State::Run:
        {
            const auto v = mpm_pa::value(0);
            const auto vv = etl::scale(v.toInt(), etl::Intervall{172u, 1810u}, etl::Intervall{2000u,4000u});
//            cppm::set(ch_t{0}, cppm::ranged_type(vv));
        }
            mChangeTick.on(mChangeTicks, []{
                cppm::set(ch_t{0}, vv);
                vv += 100;
                if (vv.isTop()) {
                    vv.setToBottom();
                }
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                break;
            case State::Set:
                ad9851::channel(ad9851::index_type{0});
                break;
            case State::Run:
                cppm::init();
                break;
            }
        } 
    }
private:
    static inline rv_t vv{cppm::ocMedium};    
    static inline State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline External::Tick<systemTimer> mChangeTick;
};

using fsm = Fsm<>;

using isrRegistrar = IsrRegistrar<cppm::CmpHandler<ad9851::SetBaseFrequency>, cppm::OvfHandler<ad9851::SetUpperFrequency>>;

int main() {
    portmux::init();
    ccp::unlock([]{
         clock::template init<Project::Config::fMcuMhz>();
    });
    systemTimer::init();
    
    fsm::init();
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        while(true) {
            fsm::periodic();                        
            systemTimer::periodic([&]{
                fsm::ratePeriodic();                        
            });
        }
    }    
}
 
ISR(TCA0_OVF_vect) {
    a5Pin::on();
    isrRegistrar::isr<AVR::ISR::Tca<0>::Ovf>();
}
ISR(TCA0_CMP1_vect) {
    a5Pin::off();
    isrRegistrar::isr<AVR::ISR::Tca<0>::Cmp<1>>();
}
