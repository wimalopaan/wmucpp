#pragma once

namespace RCSwitch {
    template<typename BusParam, typename PA, typename ActorList, typename NVM = void>
    struct MultiAdapter;
    
    template<typename BusParam, typename PA, typename... Actors, typename NVM>
    struct MultiAdapter<BusParam, PA, Meta::List<Actors...>, NVM> {
        using channel_t = PA::channel_t;
        using value_t = PA::value_type;
        
        using protocol_t = BusParam::proto_type;
//        protocol_t::_;
        
        using addr_t = RCSwitch::addr_t;
        using index_t = RCSwitch::index_t;
        using mode_t = protocol_t::mode_t;
        
        using param_t = RCSwitch::param_t;
        using pvalue_t = protocol_t::pvalue_t;
        
        inline static constexpr uint8_t nActors = sizeof...(Actors);
        
        template<typename T>
        inline static constexpr bool isLearnCode(const T& v) {
            return protocol_t::isLearnCode(v);
        }
        inline static void init() {
        }
        static inline void channel(const channel_t c) {
            if (c) {
                mChannel = c;
            }
        }
        
        static inline void address(const addr_t a) {
            mAddr = a;
        }
        inline static bool ratePeriodic() {
            const auto cv = PA::valueMapped(mChannel);
            const addr_t addr = protocol_t::toAddress(cv);
            const index_t index = protocol_t::toIndex(cv);
            const mode_t mode = protocol_t::toMode(cv);
            
            if (protocol_t::isControlMessage(cv)) { // control (parameters)
                const param_t param = protocol_t::toParameter(cv);
                const pvalue_t value = protocol_t::toParameterValue(cv);
                
                if (param == protocol_t::broadCast) {
                    if (value == protocol_t::bCastOff) {
                        lastOnIndex = lastindex_t{};
                        lastOnAddr = lastaddr_t{};
                        off(Meta::List<Actors...>{});                    
                    }
                }
                else if (lastOnIndex && lastOnAddr) {
                    index_t lastOn{lastOnIndex.toInt()};
                    if ((lastOnAddr.toInt() - mAddr) < nActors) {
                        mReceivedControl = true;
                        etl::uint_ranged<uint8_t, 0, (nActors - 1)> lastAddrOffset{(uint8_t)(lastOnAddr.toInt() - mAddr)};
                        if (param == protocol_t::passThruChannel) {
                            if ((value >= 1) && (value <= 16)) {
                                channel_t v{value};
                                NVM::data().passThru({lastAddrOffset, lastOn}) = v.toInt() - 1;
                                NVM::data().change();
                            }
                            else {
                                //                            decltype(NVM::data()[lastOnIndex].passThru())::_;
                                NVM::data().passThru({lastAddrOffset, lastOn}).setNaN();
                                NVM::data().change();
                            }
                        }
                        else if (param == protocol_t::reset) {
                            if (value == protocol_t::bCastReset) {
                                using sw_t = std::remove_cvref_t<decltype(NVM::data().sw({}))>;
                                NVM::data().sw({lastAddrOffset, lastOn}) = sw_t{};
                                NVM::data().change();
                            }
                        }
                        else if (param == protocol_t::timeMpxMode) {
                            NVM::data().mpxMode(lastAddrOffset, value);
                            NVM::data().change();
                        }
                        else if (param == protocol_t::timeMpxOffsetIncrement) {
                            NVM::data().mpxOffset(lastAddrOffset, value);
                            NVM::data().change();
                        }
                        else if (param == protocol_t::testMode) {
                            NVM::data().pulseOffset(lastAddrOffset, value);
                            NVM::data().change();
                        }
                    }
                }
            }
            else { // command (execute)
                mReceivedControl = false;
                update(Meta::List<Actors...>{}, mode, index, addr);                    
            }
            return true;                
        }
        inline static bool receivedControl() {
            return mReceivedControl;
        }
    private:
        template<typename F, typename... AA>
        inline static void off(Meta::List<F, AA...>) {
            for(auto& sw : F::switches()) {
                sw = F::SwState::Off;
            }
        }
        template<typename F, typename... AA>
        inline static void update(Meta::List<F, AA...>, const mode_t mode, const index_t index, const addr_t address) {
            update<F>(mode, index, address);
            if constexpr(sizeof...(AA) > 0) {
                update(Meta::List<AA...>{}, mode, index, address);
            }
        }
        
        template<typename Actor>
        static inline void update(const mode_t mode, const index_t index, const addr_t address) {
            //                Actor::_;
            if ((Actor::address + mAddr) == address) {
                if (mode != protocol_t::off) {
                    lastOnIndex = index;
                    lastOnAddr = address;
                    if (mode == protocol_t::on) {
                        Actor::switches()[index] = Actor::SwState::Steady;
                    }
                    else if (mode == protocol_t::blink1) {
                        Actor::switches()[index] = Actor::SwState::Blink1;
                    }
                    else if (mode == protocol_t::blink2) {
                        Actor::switches()[index] = Actor::SwState::Blink2;
                    }
                    else {
                        if constexpr(!protocol_t::isLowResolution) {
                            if (mode == protocol_t::config) {
                                Actor::switches()[index] = Actor::SwState::Off;
                            }
                        }
                    }
                }
                else {
                    lastOnAddr = lastaddr_t{};
                    lastOnIndex = lastindex_t{};
                    Actor::switches()[index] = Actor::SwState::Off;
                }
            }
        }
        static inline bool mReceivedControl{false};
        using lastindex_t = etl::uint_ranged_NaN<uint8_t, index_t::Lower, index_t::Upper>; 
        using lastaddr_t = etl::uint_ranged_NaN<uint8_t, 0, addr_t::Upper>; 
        static inline lastindex_t lastOnIndex;
        static inline channel_t mChannel{9};
        static inline addr_t mAddr{0};
        static inline lastaddr_t lastOnAddr; 
    };
    
}
