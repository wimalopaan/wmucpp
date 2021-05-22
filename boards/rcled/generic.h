#pragma once

#include <cstdint>

#include <external/ibus/ibus2.h>

#include <etl/meta.h>

namespace External {
    template<typename BusDevs, typename ActorList> struct Digital;
    template<typename BusDevs, typename... Actors>
    struct Digital<BusDevs, Meta::List<Actors...>> {
        using actors_list = Meta::List<Actors...>;
        inline static constexpr uint8_t numberOfActors = Meta::size_v<actors_list>;
        
        using Actor = Meta::nth_element<0, actors_list>;
        
        using PA = BusDevs::servo_pa;
        using nvm = BusDevs::eeprom;
        
        using channel_t = PA::channel_t;
        using value_t = PA::value_type;
        
        using protocol_t = BusDevs::BusParam::proto_type;
        
        using addr_t = RCSwitch::addr_t;
        using index_t = RCSwitch::index_t;
        using mode_t = protocol_t::mode_t;
        using param_t = RCSwitch::param_t;
        using pvalue_t = protocol_t::pvalue_t;

        static inline void init() {}
    
        static inline bool periodic() {
            if (!nvm::data().mChannel) return true;

            const channel_t ch = nvm::data().mChannel.toInt();
            const auto cv = PA::valueMapped(ch);
            
            if (!cv) return true;
    
            const addr_t addr = protocol_t::toAddress(cv);
            const index_t index = protocol_t::toIndex(cv);
            const mode_t mode = protocol_t::toMode(cv);
                            
            if (protocol_t::isControlMessage(cv)) { // control
                const param_t param = protocol_t::toParameter(cv);
                const pvalue_t value = protocol_t::toParameterValue(cv);
                
                if (param == protocol_t::broadCast) {
                    if (value == protocol_t::bCastOff) {
                        lastOnIndex = lastindex_t{};
                        lastOnAddr = lastaddr_t{};
                        (Actors::off(), ...);
                    }
                }
                else if (lastOnIndex && lastOnAddr) {
                    const index_t lastOn{lastOnIndex.toInt()};
                    const uint8_t adrIndex = lastOnAddr.toInt() - nvm::data().mAddress.toInt();
                    if (adrIndex < numberOfActors) {
                        mReceivedControl = true;
                        if (param == protocol_t::reset) {
                            if (value == protocol_t::bCastReset) {
                                Meta::visitAt<actors_list>(adrIndex, [&]<typename A>(Meta::Wrapper<A>){
                                                               A::reset();
                                                           });
                            }
                        }
                        else {
                            Meta::visitAt<actors_list>(adrIndex, [&]<typename A>(Meta::Wrapper<A>){
                                                           A::set(lastOn, param, value);
                                                       });
                        }
                    }
                }
            }
            else { // command
                if (!nvm::data().mAddress) return true;
                mReceivedControl = false;                
                const uint8_t adrIndex = addr.toInt() - nvm::data().mAddress.toInt();
                if (adrIndex < numberOfActors) {
                    Meta::visitAt<actors_list>(adrIndex, [&]<typename A>(Meta::Wrapper<A>){
                                                   A::on(index, mode);
                                               });
                    if (mode != protocol_t::off) {
                        lastOnIndex = index;
                        lastOnAddr = addr;
                    }
                    else {
                        lastOnAddr = lastaddr_t{};
                        lastOnIndex = lastindex_t{};
                    }
                }
            }
            return true;                
        }
//        static inline uint16_t lchv;
//        static inline mode_t lmv;
//        static inline pvalue_t lpv;
//        static inline param_t lpp;
    private:
        static inline bool mReceivedControl{false};
        using lastindex_t = etl::uint_ranged_NaN<uint8_t, index_t::Lower, index_t::Upper>; 
        static inline lastindex_t lastOnIndex;
        using lastaddr_t = etl::uint_ranged_NaN<uint8_t, 0, addr_t::Upper>; 
        static inline lastaddr_t lastOnAddr;
    };
}
