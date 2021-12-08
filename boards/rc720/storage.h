#pragma once

namespace Storage {
    inline static constexpr uint8_t NAdresses = 5;
    inline static constexpr uint8_t NChannels = 8;
    
    enum class Mode : uint8_t {
        Graupner8K, // 2 long
        Graupner4K, // 2 long
        Robbe,  // 1 short
        CP8, // 1 long (nur Aktivierung zu kurzen Impulsen)
        CP16, // 1 long (nur Aktivierung zu kurzen Impulsen)
        XXX // 2 short
    }; 
    
    template<typename Channel>
    struct SwitchConfig {
        using channel_type = Channel;
        channel_type& passThru() {
            return mPassThruChannel;
        }
    private:
        channel_type mPassThruChannel{};
    };
    
    struct ChannelIndex {
        using channel_type = etl::uint_ranged<uint8_t, 0, NChannels - 1>;
        using addr_type = etl::uint_ranged<uint8_t, 0, NAdresses - 1>;
        addr_type address;
        channel_type channel;
    };
    

    template<typename Channel, typename AddressR, typename Bus>
    struct ApplDataBus final : public EEProm::DataBase<ApplDataBus<Channel, AddressR, Bus>> {
        static_assert(External::Bus::isSBus<Bus>::value || External::Bus::isIBus<Bus>::value);
//                Channel::_;
        using Address = etl::uint_ranged_NaN<typename AddressR::value_type, AddressR::Lower, AddressR::Upper>;
//        Address::_;
        using value_type = SwitchConfig<Channel>;

        Channel& passThru(const ChannelIndex& i) {
            return AValues[i.address][i.channel].passThru();               
        }
        
        value_type& sw(const ChannelIndex& i) {
            return AValues[i.address][i.channel];    
        }
        uint8_t& magic() {
            return mMagic;
        }
        void clear() {
            for(auto& adr : AValues) {
                for(auto& v : adr) {
                    v = value_type{};
                }
            }
            for(auto& v : mMpxModes) {
                v = Mode::Graupner8K;
            }
            for(auto& v : mMpxOffsets) {
                v = 200;
            }
            for(auto& v : mPulseOffsets) {
                v = 200;
            }
        }
        Channel& channel() {
            return mChannel;
        }
        Address& address() {
            return mAddress;
        }

        void pulseOffset(uint8_t addressOffset, uint8_t v) {
            if (addressOffset < mMpxModes.size()) {
                if constexpr(External::Bus::isSBus<Bus>::value) {
                    mPulseOffsets[addressOffset] = 40 * (v + 1); // 20 - 640
                }
                else {
                    mPulseOffsets[addressOffset] = 20 * (v + 1); // 20 - 640
                }
            }
        }        
        uint16_t pulseOffset(uint8_t addressOffset) {
                return mPulseOffsets[addressOffset];
        }
        
        void mpxOffset(uint8_t addressOffset, uint8_t v) {
            if (addressOffset < mMpxModes.size()) {
                if constexpr(External::Bus::isSBus<Bus>::value) {
                    mMpxOffsets[addressOffset] = 40 * (v + 1); // 20 - 640
                }
                else {
                    mMpxOffsets[addressOffset] = 20 * (v + 1); // 20 - 640
                }
            }
        }
        uint16_t mpxOffset(uint8_t addressOffset) {
                return mMpxOffsets[addressOffset];
        }

        void mpxMode(uint8_t addressOffset, uint8_t v) {
            if (addressOffset < mMpxModes.size()) {
                if (v == 0) {
                    mMpxModes[addressOffset] = Mode::Graupner8K;
                }
                else if (v == 1) {
                    mMpxModes[addressOffset] = Mode::Graupner4K;
                }
                else if (v == 2) {
                    mMpxModes[addressOffset] = Mode::Robbe;
                }
                else if (v == 3) {
                    mMpxModes[addressOffset] = Mode::CP8;
                }
                else if (v == 4) {
                    mMpxModes[addressOffset] = Mode::CP16;
                }
                else if (v == 5) {
                    mMpxModes[addressOffset] = Mode::XXX;
                }
                else {
                    mMpxModes[addressOffset] = Mode::Graupner8K;
                }
            }
        }
        Mode mpxMode(uint8_t addressOffset) {
            if (addressOffset < mMpxModes.size()) {
                return  mMpxModes[addressOffset];
            }
            return Mode::Graupner8K;
        }
    private:
        std::array<Mode, NAdresses> mMpxModes {};
        std::array<uint16_t, NAdresses> mMpxOffsets{};
        std::array<uint16_t, NAdresses> mPulseOffsets{};
        uint8_t mMagic;
        Channel mChannel;
        Address mAddress;
        std::array<std::array<value_type, NChannels>, NAdresses> AValues;
    };

}

