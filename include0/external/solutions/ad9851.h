#pragma once



namespace External {
    namespace DDS {
        
        template<typename Timer, AVR::Concepts::ActivatableOut DPin, AVR::Concepts::Pulseable WClkSig, AVR::Concepts::Pulseable FUpdSig, AVR::Concepts::Pulseable ResetSig>
        struct AD9851 {
            static inline constexpr uint64_t ad_c = uint64_t{1} << 32;
            static inline constexpr uint64_t ad_sysclk = 30'000'000 * 6; // 180MHz bei 30MHz Quarz
            static inline constexpr uint16_t fOffset = 2'000;
            static inline constexpr External::Tick<Timer> mChangeTicks{100_ms};
            
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
                constexpr ChannelData(const uint32_t f) : 
                    f0{f}, f1{f + fOffset}{}
                
                ChannelData(const AVR::Pgm::Ptr<ChannelData>& ptr) :
                    f0{AVR::Pgm::ByteRange{ptr}.begin()}, f1{AVR::Pgm::ByteRange<ChannelData, sizeof(f0)>{ptr}.begin()}
                {}
                const FreqWord f0;
                const FreqWord f1;
            } __attribute__((packed));
            
            struct Generator {
                constexpr auto operator()() {
                    return []<auto... II>(std::index_sequence<II...>){
                        return std::array<ChannelData, RC::channels.size()>{ChannelData{RC::channels[II].mFreq}...};
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
//                mIndex = i;
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
            static inline External::Tick<Timer> mStateTick;
            static inline State mState{State::Undefined};
//            static inline index_type mIndex;
        };
    }
}

