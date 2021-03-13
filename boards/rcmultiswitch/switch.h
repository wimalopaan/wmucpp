#pragma once

//template<typename PA, typename Actor, typename Out = void>
template<typename BusDevs>
struct Digital {
    using PA = BusDevs::servo_pa;
    using Actor = BusDevs::sw;
    using Out = BusDevs::out;
    
    using channel_t = PA::channel_t;
    using value_t = PA::value_type;
    
    using protocol_t = BusDevs::BusParam::proto_type;
    
    using addr_t = RCSwitch::addr_t;
    using index_t = RCSwitch::index_t;
    using mode_t = protocol_t::mode_t;
    using param_t = RCSwitch::param_t;
    using pvalue_t = protocol_t::pvalue_t;
    
    using blink_index_t = Out::blink_index_t;
    
    using tick_t = Out::tick_t;
    inline static constexpr auto blinkMax = tick_t::max();

    inline static constexpr auto pwmMax = Out::pwmMax;
    
    static inline void init(const channel_t c) {
        mChannel = c;
    }

    static inline void channel(const channel_t c) {
        if (c) {
            mChannel = c;
        }
    }

    static inline void address(const addr_t a) {
        mAddr = a;
    }
    
    template<typename T>
    inline static constexpr bool isLearnCode(const T& v) {
        return protocol_t::isLearnCode(v);
    }
    static inline bool periodic() {
        const auto cv = PA::valueMapped(mChannel);

        if (!cv) return true;
        
        const addr_t addr = protocol_t::toAddress(cv);
        const index_t index = protocol_t::toIndex(cv);
        const mode_t mode = protocol_t::toMode(cv);
                        
        if (protocol_t::isControlMessage(cv)) { // control
            const param_t param = protocol_t::toParameter(cv);
            const pvalue_t value = protocol_t::toParameterValue(cv);
            
            lpv = value;
            lpp = param;
            
            if (param == protocol_t::broadCast) {
                if (value == protocol_t::bCastOff) {
                    lastOnIndex = lastindex_t{};
                    for(auto& s : Actor::switches()) {
                        s = Actor::SwState::Off;
                    }                        
                }
            }
            else if (lastOnIndex) {
                index_t lastOn{lastOnIndex.toInt()};
                if (param == protocol_t::reset) {
                    if (value == protocol_t::bCastReset) {
                        Out::reset(lastOn);
                    }
                }
                else if (param == protocol_t::pwm) {
                    Actor::switches()[lastOn] = Actor::SwState::Steady;
                    const uint8_t pwm = ((uint16_t)value * pwmMax) / pvalue_t::Upper;
                    Out::pwm(lastOn, pwm);
                }
                else if (param == protocol_t::blink1Intervall) {
                    Actor::switches()[lastOn] = Actor::SwState::Blink1;
                    const uint16_t intervall = ((uint32_t)value * tick_t::max()) / pvalue_t::Upper;
                    Out::intervall2(lastOn, tick_t::fromRaw(intervall), blink_index_t{0});
                    Out::duration(lastOn, tick_t::fromRaw(intervall / 2), blink_index_t{0});
                }
                else if (param == protocol_t::blink1Duration) {
                    Actor::switches()[lastOn] = Actor::SwState::Blink1;
                    const uint16_t intervall = Out::intervall(lastOn, blink_index_t{0}).value;
                    const uint16_t duration = ((uint32_t)value * intervall) / pvalue_t::Upper; 
                    Out::duration(lastOn, tick_t::fromRaw(duration), blink_index_t{0});
                }
                else if (param == protocol_t::blink2Intervall) {
                    Actor::switches()[lastOn] = Actor::SwState::Blink2;
                    const uint16_t intervall = ((uint32_t)value * tick_t::max()) / pvalue_t::Upper;
                    Out::intervall2(lastOn, tick_t::fromRaw(intervall), blink_index_t{1});
                    Out::duration(lastOn, tick_t::fromRaw(intervall / 2), blink_index_t{1});
                }
                else if (param == protocol_t::blink2Duration) {
                    Actor::switches()[lastOn] = Actor::SwState::Blink2;
                    const uint16_t intervall = Out::intervall(lastOn, blink_index_t{1}).value;
                    const uint16_t duration = ((uint32_t)value * intervall) / pvalue_t::Upper; 
                    Out::duration(lastOn, tick_t::fromRaw(duration), blink_index_t{1});
                }
                else if (param == protocol_t::testMode) {
                    Actor::testMode(value);
                }
            }
        }
        else { // command
            if (addr != mAddr) {
                return false;
            }
            lmv = mode;
            if (mode != protocol_t::off) {
                lastOnIndex = index.toInt();
                if (mode == protocol_t::on) {
                    Actor::switches()[index] = Actor::SwState::Steady;
                }
                else if (mode == protocol_t::blink1) {
                    Actor::switches()[index] = Actor::SwState::Blink1;
                }
                else if (mode == protocol_t::blink2) {
                    Actor::switches()[index] = Actor::SwState::Blink2;
                }
            }
            else {
                lastOnIndex = lastindex_t{};
                Actor::switches()[index] = Actor::SwState::Off;
            }
        }
        return true;                
    }
    static inline mode_t lmv;
    static inline pvalue_t lpv;
    static inline param_t lpp;
//        private: 
    using lastindex_t = etl::uint_ranged_NaN<uint8_t, index_t::Lower, index_t::Upper>; 
    static inline lastindex_t lastOnIndex;
    static inline channel_t mChannel{14}; // ch 16
    static inline addr_t    mAddr{0};
};
