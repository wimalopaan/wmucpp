#pragma once

#include <external/solutions/rc/rf.h>

namespace External {
    namespace DDS {
        
        template<typename Timer, AVR::Concepts::ActivatableOut DPin, AVR::Concepts::Pulseable WClkSig, AVR::Concepts::Pulseable FUpdSig, AVR::Concepts::Pulseable ResetSig>
        struct AD9851 {
            static inline constexpr uint64_t ad_c = uint64_t{1} << 32;
            static inline constexpr uint64_t ad_sysclk = 30'000'000 * 6; // 180MHz bei 30MHz Quarz
            static inline constexpr uint16_t fOffset = 2'000;
            static inline constexpr External::Tick<Timer> mChangeTicks{100_ms};
            
            struct SetBaseFrequency { 
                static inline void once() { // Interrupt
                    if (mState == State::Ready) {
                        put(mData);
                        FUpdSig::pulse();
                    }
                }
                static inline volatile std::array<std::byte, 4> mData;
            };
            struct SetUpperFrequency {
                static inline void once() { // Interrupt
                    if (mState == State::Ready) {
                        put(mData);
                        FUpdSig::pulse();
                    }
                }
                static inline volatile std::array<std::byte, 4> mData;
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

            template<External::RC::Band B>            
            struct Generator {
                constexpr auto operator()() {
                    constexpr uint8_t chNumber = []{
                        uint8_t n{0};
                        for(const auto& c: RC::channels) {
                            if (c.mBand == B) {
                                ++n;
                            }
                        }
                        return n;
                    }();
                    constexpr uint8_t chOffset = []{
                        for(uint8_t i{0}; const auto& c: RC::channels) {
                            if (c.mBand == B) {
                                return i;
                            }
                            ++i;
                        }
                        return uint8_t{0};
                    }();
                    return []<auto... II>(std::index_sequence<II...>){
                        return std::array<ChannelData, chNumber>{ChannelData{RC::channels[II + chOffset].mFreq, 
                                        (uint8_t)RC::channels[II + chOffset].mBand, (uint8_t)RC::channels[II + chOffset].mNumber}...};
                    }(std::make_index_sequence<chNumber>{});
                }
            };
            
            using ChData27MHz = AVR::Pgm::Util::Converter<Generator<External::RC::Band::_27MHz>>::pgm_type;
            using ChData35AMHz = AVR::Pgm::Util::Converter<Generator<External::RC::Band::_35AMHz>>::pgm_type;
            using ChData35BMHz = AVR::Pgm::Util::Converter<Generator<External::RC::Band::_35BMHz>>::pgm_type;
            using ChData40MHz = AVR::Pgm::Util::Converter<Generator<External::RC::Band::_40MHz>>::pgm_type;
            using ChData41MHz = AVR::Pgm::Util::Converter<Generator<External::RC::Band::_41MHz>>::pgm_type;
            using ChData72MHz = AVR::Pgm::Util::Converter<Generator<External::RC::Band::_72MHz>>::pgm_type;

            enum class State : uint8_t {Undefined, Init, Ready, ReadyOff};
            
            enum class Event : uint8_t {None, RFon, RFoff};
            static inline Event mEvent{Event::None};
            
            static inline Event event() {
                Event e{Event::None};
                using std::swap;
                swap(e, mEvent);
                return e;
            }
            
        public:
            static inline bool ready() {
                return (mState == State::Ready) || (mState == State::ReadyOff);
            }
            static inline void init() {
                DPin::init();
                WClkSig::init();
                FUpdSig::init();
                ResetSig::init();
            }  
            
            static inline void rfOn(const bool b) {
                if (b) {
                    mEvent = Event::RFon;
                }                
                else {
                    mEvent = Event::RFoff;
                }
            }
            
            using rfChannel_t = etl::uint_ranged_NaN<uint16_t, External::RC::minChannelNumber, External::RC::maxChannelNumber>;
            
            static inline rfChannel_t channel(const External::RC::Band band, const uint8_t i) {
                switch(band) {
                case External::RC::Band::_27MHz:
                    if (i < ChData27MHz::size()) {
                        etl::copy(SetBaseFrequency::mData, ChData27MHz::value(i).f0.data);
                        etl::copy(SetUpperFrequency::mData, ChData27MHz::value(i).f1.data);
                        return rfChannel_t{ChData27MHz::value(i).ch};
                    }
                    break;
                case External::RC::Band::_35AMHz:
                    if (i < ChData35AMHz::size()) {
                        etl::copy(SetBaseFrequency::mData, ChData35AMHz::value(i).f0.data);
                        etl::copy(SetUpperFrequency::mData, ChData35AMHz::value(i).f1.data);
                        return rfChannel_t{ChData35AMHz::value(i).ch};
                    }
                    break;
                case External::RC::Band::_35BMHz:
                    if (i < ChData35BMHz::size()) {
                        etl::copy(SetBaseFrequency::mData, ChData35BMHz::value(i).f0.data);
                        etl::copy(SetUpperFrequency::mData, ChData35BMHz::value(i).f1.data);
                        return rfChannel_t{ChData35BMHz::value(i).ch};
                    }
                    break;
                case External::RC::Band::_40MHz:
                    if (i < ChData40MHz::size()) {
                        etl::copy(SetBaseFrequency::mData, ChData40MHz::value(i).f0.data);
                        etl::copy(SetUpperFrequency::mData, ChData40MHz::value(i).f1.data);
                        return rfChannel_t{ChData40MHz::value(i).ch};
                    }
                    break;
                case External::RC::Band::_41MHz:
                    if (i < ChData41MHz::size()) {
                        etl::copy(SetBaseFrequency::mData, ChData41MHz::value(i).f0.data);
                        etl::copy(SetUpperFrequency::mData, ChData41MHz::value(i).f1.data);
                        return rfChannel_t{ChData41MHz::value(i).ch};
                    }                    
                    break;
                case External::RC::Band::_72MHz:
                    if (i < ChData72MHz::size()) {
                        etl::copy(SetBaseFrequency::mData, ChData72MHz::value(i).f0.data);
                        etl::copy(SetUpperFrequency::mData, ChData72MHz::value(i).f1.data);
                        return rfChannel_t{ChData72MHz::value(i).ch};
                    }
                    break;
                }
                return {};
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
                        mState = State::ReadyOff;
                    });
                    break;
                case State::ReadyOff:
                    if (event() == Event::RFon) {
                        mState = State::Ready;
                    }
                    break;
                case State::Ready:
                    if (event() == Event::RFoff) {
                        mState = State::ReadyOff;
                    }
                    break;
                }
                if (oldState != mState) {
                    mStateTick.reset();
                    switch (mState) {
                    case State::Undefined:
                        break;
                    case State::Init:
                    case State::ReadyOff:
                        ResetSig::pulse();
                        break;
                    case State::Ready:
                        ResetSig::pulse();
                        setSerialMode();
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
            static inline void put(const volatile std::array<std::byte, 4>& a) {
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
            static inline External::Tick<Timer> mStateTick;
            static inline State mState{State::Undefined};
        };
    }
}
