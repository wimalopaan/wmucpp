#define NDEBUG

#define LEARN_DOWN
#define SCALE_ADC_PWM

#ifndef NDEBUG
static unsigned int assertKey{1234};
#endif

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
#include <mcu/internals/syscfg.h>
#include <mcu/internals/event.h>
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
#include <external/solutions/rc/busscan.h>
 
#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

namespace xassert {
    etl::StringBuffer<81> ab;
    etl::StringBuffer<10> aline;
    bool on{false};
}

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace IBus::Switch {
    template<typename Param, typename PA, typename Sensor, typename ActorList, typename NVM>
    struct GeneralSwitch;
    
    template<typename Param, typename PA, typename Sensor, typename... Actors, typename NVM>
    struct GeneralSwitch<Param, PA, Sensor, Meta::List<Actors...>, NVM> {
        using actor_list = Meta::List<Actors...>;
        using channel_t = PA::channel_t;
        using value_t = PA::value_type;
        
        using protocol_t = Param::proto_type;
        using bus_t = Param::bus_t;
//        protocol_t::_;
        
        using addr_t = RCSwitch::addr_t;
        using index_t = RCSwitch::index_t;
        using param_t = RCSwitch::param_t;
        using mode_t = protocol_t::mode_t;
        using pvalue_t = protocol_t::pvalue_t;
        
        static inline void init(const channel_t c = 0) {
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
        static inline bool ratePeriodic() {
            const auto cv = PA::valueMapped(mChannel);
            
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
                        Meta::visit<actor_list>([]<typename A>(const Meta::Wrapper<A>){
                                                    A::stop();
                                                });
                        gotBCastOff = true;
                    }
                }
                else if (lastOnIndex) {
                    index_t lastOn{lastOnIndex.toInt()};
                    mReceivedControl = true;
                    gotBCastOff = false;
                    if (param == protocol_t::reset) {
                        if (value == protocol_t::bCastReset) {
                            Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                          A::reset();
                                                      });
                        }
                    }
                    else if (param == protocol_t::timeMpxMode) { // FrSky: phys. Sensor-Id
                        // Why??
                        Meta::visit<actor_list>([]<typename A>(const Meta::Wrapper<A>){
                                                    A::stop();
                                                });
                        
                        if ((value >= 1) && (value < External::SPort::sensor_ids.size())) {
                            if constexpr(External::Bus::isSBus<bus_t>::value) {
                                const External::SPort::SensorId id = External::SPort::sensor_ids[value - 1];
                                Sensor::ProtocollAdapter::id(id);
                                NVM::data().physicalId(id);
                                NVM::data().change();
                            }
                        }
                    }
                    else if (param == protocol_t::motorRampTime) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setRamp(value);
                                                  });
                    }
                    else if (param == protocol_t::pwm1) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setPwmForward(value);
                                                  });
                    }
                    else if (param == protocol_t::pwm2) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setPwmBackward(value);
                                                  });
                    }
                    else if (param == protocol_t::offCurr1) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setOffCurrentForward(value);
                                                  });
                    }
                    else if (param == protocol_t::offCurr2) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setOffCurrentBackward(value);
                                                  });
                    }
                    else if (param == protocol_t::passThruChannel) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setPassThru(value);
                                                  });
                    }
                    else if (param == protocol_t::testMode) {
                        if ((value >= 1) && (value <= 8)) {
                            Meta::visit<actor_list>([&]<typename A>(const Meta::Wrapper<A>&) {
                                                        //                                                        A::setTest(mode_t(value - 1));
                                                    });
                        }   
                    }
                    else if (param == protocol_t::adcSensitivity) {
                    }
                    else if (param == protocol_t::cutOffSensitivity) {
                        if ((value >= 1) && (value <= 10)) {
                            Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                          A::setStartOffCurrentFactor(value);
                                                      });
                        }
                    }
                }
            }
            else { // command
                mReceivedControl = false;
                if (addr != mAddr) {
                    return false;
                }
                if (mode != protocol_t::off) {
                    lastOnIndex = index.toInt();
                }
                else {
                    lastOnIndex = lastindex_t{};
                }
                Meta::visitAt<actor_list>(index.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                              if (mode == protocol_t::off) {
                                                  A::stop();
                                              }
                                              else if (mode == protocol_t::forward) {
                                                  if (!gotBCastOff) { // parameter selection
                                                      A::startForward();
                                                  }
                                              }
                                              else if (mode == protocol_t::backward) {
                                                  A::startBackward();
                                              }
                                          });
            }
            return true;                
        }
        inline static bool receivedControl() {
            return mReceivedControl;
        }
    private: 
        static inline bool gotBCastOff{false};
        static inline bool mReceivedControl{false};
        using lastindex_t = etl::uint_ranged_NaN<uint8_t, 0, Meta::size_v<actor_list> - 1>; 
        static inline lastindex_t lastOnIndex;
        static inline channel_t mChannel{14};
        static inline addr_t    mAddr{0};
    };
}

namespace Storage {
    inline static constexpr uint8_t NChannels = 4;
    
    template<typename Timer, typename Channel, typename PWM_T>
    struct Output {
        inline static constexpr auto defaultCurrentCheckTimeout = 200_ms;
        
        void currentCheckTO(const auto& t){
            currentCheckTimeout = t;
            rampCyles = ccTimeoutToRampCycles(External::Tick<Timer>{t});
        }
        
        auto currentCheckTO() {
            return currentCheckTimeout.time();
        }
        
        External::Tick<Timer> currentCheckTimeout{defaultCurrentCheckTimeout};
        
        External::Tick<Timer> ccTimeoutToRampCycles(const External::Tick<Timer>& ccTo) {
            return ccTo / (PWM_T::Upper - PWM_T::Lower + 1);
        }
        
        External::Tick<Timer> rampCyles = ccTimeoutToRampCycles(currentCheckTimeout);
        //        static_assert(rampCyles.value >= 1);
        //        std::integral_constant<uint16_t, rampCyles.value>::_;
        
        PWM_T pwmForward{PWM_T::Upper / 4};
        PWM_T pwmBackward{PWM_T::Upper / 4};
        
        uint16_t forwardRefC{std::numeric_limits<uint16_t>::max()};
        uint16_t backwardRefC{std::numeric_limits<uint16_t>::max()};
        
        uint16_t forwardOffC{std::numeric_limits<uint16_t>::max()};
        uint16_t backwardOffC{std::numeric_limits<uint16_t>::max()};
        
        etl::uint_ranged<uint8_t, 1, 10> startOffCFactor{1};
        
        Channel passThru{};
    };
    
    template<typename Timer, typename Channel, typename AddressR, typename PWM_T>
    struct ApplData final : public EEProm::DataBase<ApplData<Timer, Channel, AddressR, PWM_T>> {
//                Channel::_;
        using Address = etl::uint_ranged_NaN<typename AddressR::value_type, AddressR::Lower, AddressR::Upper>;
//                Address::_;
        using value_type = Output<Timer, Channel, PWM_T>;
        
        uint8_t& magic() {
            return mMagic;
        }
        
        void clear() {
            for(value_type& v : mData) {
                v = value_type{};
            }
            mChannel = 14;
            mAddress = 0;
            mPhysId = External::SPort::SensorId::ID3;
        }
        
        Channel& channel() {
            return mChannel;
        }
        Address& address() {
            return mAddress;
        }
        void physicalId(const External::SPort::SensorId id) {
            mPhysId = id;
        }
        External::SPort::SensorId physicalId() {
            return mPhysId;
        }
        
        using index_type = etl::uint_ranged<uint8_t, 0, NChannels - 1>;
        
//        value_type& output(const index_type i) {
//            return mData[i];
//        }
        
        template<index_type I>
        value_type& output() {
            return mData[I];
        }        
    private:
        uint8_t mMagic;
        Channel mChannel;
        Address mAddress;
        std::array<value_type, NChannels> mData;
        External::SPort::SensorId mPhysId{External::SPort::SensorId::ID1};
    };
}

namespace  {
    constexpr auto fRtc = 1000_Hz;

    //    constexpr uint16_t Ri = 2700;
    constexpr uint16_t Ri = 3000;
    
    constexpr uint16_t Ro = 2100;
    constexpr uint16_t R0 = 1500;
    constexpr uint16_t R1 = 2100;
    constexpr uint16_t R2 = 3900;
    constexpr uint16_t R3 = 8200;
}

template<typename Param, typename Timer, uint8_t N, typename PWM, typename ADC, 
         typename InA, typename InB, typename EndPin, typename PA, typename NVM, typename Term>
struct ChannelFsm {
    template<auto ID, typename State>
    struct StateProvider {
        inline static constexpr auto ibus_type = ID;
        inline static constexpr auto valueId = ID;
        inline static constexpr auto Id = ID;
        
        static inline void init() {}
        
        static inline uint16_t value() {
    #ifdef FULL_STATE
            return 10000 + uint16_t(mValue) + (mOcf ? 1000 : 0) + (mOcb ? 100 : 0);
    #else
            return uint16_t(mValue) + 1000;
    #endif
        }
        static inline void set(const State s) {
            if (s != mValue) {
                mValue = s;
                mChanged = true;
            }
        }
        [[nodiscard]] static inline bool changingParameters() {
            mChanged = false;   
            return (mValue>= State::SettingsBegin) && (mValue <= State::SettingsEnd);
        }
        
        [[nodiscard]] static inline bool resetted() {
            mChanged = false;   
            return mValue == State::Reset;
        }
        static inline void error(const bool b) {
            if (mError != b) {
                mError = b;
                mChanged = true;
            }
        }
        [[nodiscard]] static inline bool oc() {
            mChanged = false;   
            return (mValue == State::OffForwardOC) || (mValue == State::OffBackwardOC);
        }
        [[nodiscard]] static inline bool end() {
            mChanged = false;   
            return (mValue== State::OffForwardEnd) || (mValue == State::OffBackwardEnd);
        }
        [[nodiscard]] static inline bool error() {
            mChanged = false;   
            return mError;
        }
        [[nodiscard]] static inline bool changed() {
            return mChanged;
        }
    private:
        static inline bool mError{false};
        static inline State mValue{State::Init};
        static inline bool mChanged{false};
    };

    using protocol_t = Param::proto_type;
    inline static constexpr auto stateProviderId = Param::stateProviderId;
//    using bus_t = protocol_t::bus_t;
    
    using pvalue_t = protocol_t::pvalue_t;
    
    using adc_index_t = typename ADC::index_type;
    static inline constexpr adc_index_t adci{N};
    
    inline static constexpr double VRef = ADC::VRef;
    using value_type = typename ADC::value_type;
    using reso_type = typename ADC::mcu_adc_type::reso_type;
    
    using pwm_value_t = typename PWM::value_type;
    
    using pa_value_t = typename PA::value_type;
    using channel_t = typename PA::channel_t;
    
    using storage_t = std::remove_cvref_t<decltype(NVM::data())>;
    using data_t = storage_t::value_type;
    using data_index_t = storage_t::index_type;
    
    using endButton = External::Button<EndPin, Timer, External::Tick<Timer>{100_ms}, External::Tick<Timer>{300_ms}>;
    
    inline static constexpr data_index_t data_index{N};
    
    enum class State : uint8_t {Init = 0, 
                                Off = 1, OffWait, Reset,
                                Forward = 10, Backward, ForwardWait, BackwardWait, 
                                SettingsBegin = 20, SetForwardPwm = SettingsBegin, SetBackwardPwm, SetParameter, SetParameterWait,  
                                SettingsEnd = SetParameterWait, 
                                ForwardPwmSetPause = 30, BackwardPwmSetPause, 
                                OffForwardOC = 40, OffBackwardOC, OffForwardEnd, OffBackwardEnd,
                                ForwardPassThru = 50, BackwardPassThru, ForwardPassThruWait, BackwardPassThruWait};
    
    enum class Event : uint8_t {None, Stop, 
                                Forward, Backward, 
                                SetFowardPwm, SetBackwardPwm, 
                                SetParameter, Reset
                               };
    
    using stateP = StateProvider<stateProviderId, State>;

    static inline constexpr uint16_t pa_mid = (pa_value_t::Upper + pa_value_t::Lower) / 2;
    static inline constexpr uint16_t pa_half = (pa_value_t::Upper - pa_value_t::Lower) / 2;
    static inline constexpr uint16_t pa_hysterese = pa_half / 10;
    
    static inline constexpr External::Tick<Timer> offTimeout{300_ms};
    static inline constexpr External::Tick<Timer> setPwmTimeout{2000_ms};
    
private:
    static inline auto& pwmForward() {
        return data().pwmForward;        
    }
    static inline auto& pwmBackward() {
        return data().pwmBackward;        
    }
    static inline auto& forwardOffCurr() {
        return data().forwardOffC;        
    }
    static inline auto forwardOffCurr(const uint16_t pwm) {
        if (forwardOffCurr() == std::numeric_limits<uint16_t>::max()) {
            return forwardOffCurr();
        }
        else {
            if (forwardOffCurr() > forwardStartOffCurr()) {
                const uint16_t cv1 = forwardOffCurr() - forwardStartOffCurr();
                const uint16_t cv2 = ((((uint32_t)pwm - pa_mid) * cv1) / pa_half) + forwardStartOffCurr(); 
                return cv2;
            }
            else {
                return forwardOffCurr();
            }
        }
    }
    static inline auto backwardOffCurr(const uint16_t pwm) {
        if (backwardOffCurr() == std::numeric_limits<uint16_t>::max()) {
            return backwardOffCurr();
        }
        else {
            if (backwardOffCurr() > backwardStartOffCurr()) {
                const uint16_t cv1 = backwardOffCurr() - backwardStartOffCurr();
                const uint16_t cv2 = ((((uint32_t)pa_mid - pwm) * cv1) / pa_half) + backwardStartOffCurr(); 
                return cv2;
            }
            else {
                return backwardOffCurr();
            }
        }
    }
    static inline auto& backwardOffCurr() {
        return data().backwardOffC;        
    }
    static inline auto& forwardRefCurr() {
        return data().forwardRefC;        
    }
    static inline auto& backwardRefCurr() {
        return data().backwardRefC;        
    }
    static inline auto forwardStartOffCurr() {
        return (forwardOffCurr() * data().startOffCFactor) / 10;   
    }
    static inline auto backwardStartOffCurr() {
        return (backwardOffCurr() * data().startOffCFactor) / 10;   
    }
    inline static auto& data() {
//        std::integral_constant<uint8_t, data_index>::_;
        return NVM::data().template output<data_index>();
    }
    inline static void change() {
        NVM::data().change();
    }
    inline static auto& ccTimeout() {
        return data().currentCheckTimeout;
    }
    inline static auto& rampCycles() {
        return data().rampCyles;
    }
    inline static auto& passThru() {
        return data().passThru;
    }

public:
    inline static void init() {
        InA::template dir<Output>();
        InB::template dir<Output>();
        off();
        PWM::template on<N>();
        endButton::init();
    }
    
    inline static void reset() {
        stop();
        off();
        data() = data_t{};
        stateP::error(false);
        change();
        mEvent = Event::Reset;
    }
    
    inline static void setRamp(const pvalue_t v) {
        auto push = [](auto nv) {
            data().currentCheckTO(nv);
            mEvent = Event::SetParameter;
            change();
        };
        if ((v > 0) && (v <= pvalue_t::Upper)) {
            const auto nv = data().defaultCurrentCheckTimeout * v.toInt();
            push(nv);
        }
        else {
            push(data().defaultCurrentCheckTimeout);
        }
    }
    
    inline static void setPassThru(const auto v) {
        if ((v > 0) && (v <= (channel_t::Upper + 1))) {
            const auto c = v - 1;
            passThru() = c;        
            change();
            mEvent = Event::SetParameter;
        }           
        else {
            if (passThru()) {
                passThru() = channel_t{};
                change();
                mEvent = Event::SetParameter;
            }
        }
    }
    
    inline static void setPwmForward(const pvalue_t& v) {
        mEvent = Event::SetFowardPwm;
        mEventValue = v;
    }
    
    inline static void setPwmBackward(const pvalue_t& v) {
        mEvent = Event::SetBackwardPwm;
        mEventValue = v;
    }
    
    inline static void setOffCurrentForward(const pvalue_t& v) {
        if (v.isTop()) {
            forwardOffCurr() = std::numeric_limits<uint16_t>::max();
            change();
            mEvent = Event::SetParameter;
        }
        else {
            const auto n = forwardRefCurr() + ((forwardRefCurr() * v.toInt()) >> 3);
            forwardOffCurr() = n;
            change();
            mEvent = Event::SetParameter;
            mEventValue = v;
            if (n >= ADC::value_type::Upper) {
                stateP::error(true);
            }    
            else {
                stateP::error(false);
            }
        }
    }
    
    inline static void setOffCurrentBackward(const pvalue_t& v) {
        if (v.isTop()) {
            backwardOffCurr() = std::numeric_limits<uint16_t>::max();
            change();
            mEvent = Event::SetParameter;
        }
        else {
            const auto n = backwardRefCurr() + ((backwardRefCurr() * v.toInt()) >> 3);
            backwardOffCurr() = n;
            change();
            mEvent = Event::SetParameter;
            mEventValue = v;
            if (n >= ADC::value_type::Upper) {
                stateP::error(true);
            }    
            else {
                stateP::error(false);
            }
        }
    }
    
    inline static void setStartOffCurrentFactor(const pvalue_t& v) {
        data().startOffCFactor.set(v);
        mEvent = Event::SetParameter;
    }
    
    inline static void stop() {
        mEvent = Event::Stop;
    }
    
    inline static void startForward() {
        mEvent = Event::Forward;
    }
    
    inline static void startBackward() {
        mEvent = Event::Backward;
    }
    
    inline static Event event() {
        Event e{Event::None};
        std::swap(e, mEvent);
        return e;
    }
    
    inline static void periodic() {
    }
    
    inline static void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        ++mRampTick;
        endButton::periodic();
        const auto e = event();
        const auto buttenEvent = endButton::event();
        updateAdc();
        switch(mState) {
        case State::Init:
            mState = State::Off;
            break;
        case State::OffWait:
            mStateTick.on(offTimeout, []{
                mState = State::Off;
            });
            break;
        case State::Off:
            if (auto pt = passThru()) {
                if (auto pv = PA::value(pt)) {
                    if (auto rv = pv.toInt(); rv > (pa_mid + pa_hysterese)) {
                        mState = State::ForwardPassThruWait;
                    } 
                    else if (rv < (pa_mid - pa_hysterese)) {
                        mState = State::BackwardPassThruWait;
                    }
                }         
            }
            else {
                if (e == Event::Forward) {
                    mState = State::ForwardWait;
                }
                else if (e == Event::Backward) {
                    mState = State::BackwardWait;
                }
            }
            if (e == Event::SetFowardPwm) {
                mState = State::SetForwardPwm;
            }
            else if (e == Event::SetBackwardPwm) {
                mState = State::SetBackwardPwm;
            }
            else if (e == Event::SetParameter) {
                mState = State::SetParameter;
            }
            else if (e == Event::Reset) {
                mState = State::Reset;
            }
            break;
        case State::SetForwardPwm:
            if (e != Event::None) {
                if (e == Event::SetFowardPwm) {
                    updateForwardPwm();
                    mStateTick.on(setPwmTimeout, []{
                        mState = State::ForwardPwmSetPause;
                    });
                }
                else {
                    mState = State::OffWait;                
                }
            }
            else {
                mStateTick.on(setPwmTimeout, []{
                    mState = State::OffWait;
                });
            }
            break;
        case State::ForwardPwmSetPause:
            if (e != Event::None) {
                if (e != Event::SetFowardPwm) {
                    mState = State::OffWait;
                }
            }
            break;
        case State::SetBackwardPwm:
            if (e != Event::None) {
                if (e == Event::SetBackwardPwm) {
                    updateBackwardPwm();
                    mStateTick.on(setPwmTimeout, []{
                        mState = State::BackwardPwmSetPause;
                    });
                }
                else {
                    mState = State::OffWait;                
                }
            }
            else {
                mStateTick.on(setPwmTimeout, []{
                    mState = State::OffWait;
                });
            }
            break;
        case State::BackwardPwmSetPause:
            if (e != Event::None) {
                if (e != Event::SetBackwardPwm) {
                    mState = State::OffWait;
                }
            }
            break;
        case State::Reset:
            if (e != Event::None) {
                if (e == Event::Reset) {
                }
                else {
                    mState = State::OffWait;                
                }
            }
            else {
                mStateTick.on(setPwmTimeout, []{
                    mState = State::SetParameterWait;
                });
            }
            break;
        case State::SetParameter:
            if (e != Event::None) {
                if (e == Event::SetParameter) {
                }
                else {
                    mState = State::OffWait;                
                }
            }
            else {
                mStateTick.on(setPwmTimeout, []{
                    mState = State::SetParameterWait;
                });
            }
            break;
        case State::SetParameterWait:
            if (e != Event::None) {
                if (e == Event::SetParameter) {
                    mState = State::SetParameter;                
                }
                else {
                    mState = State::OffWait;                
                }
            }
            else {
                mStateTick.on(setPwmTimeout, []{
                    mState = State::OffWait;
                });
            }
            break;
        case State::ForwardWait:
            mRampTick.on(rampCycles(), []{
                if (++mRampPwm <= pwmForward()) {
                    updatePwm(mRampPwm);
                }
            });
            mStateTick.on(ccTimeout(), []{
                mState = State::Forward; 
            });
            if (e == Event::Forward) {
            }
            else if (e == Event::Stop) {
                mState = State::OffWait;
            }
            else if (e == Event::SetFowardPwm) {
                mState = State::SetForwardPwm;
            }
            else if (e == Event::SetBackwardPwm) {
                mState = State::SetBackwardPwm;
            }
            else if (e != Event::None) {
                mState = State::OffWait;
            }
            break;
        case State::Forward:
            if (mActualAdc >= forwardOffCurr()) {
                mState = State::OffForwardOC;
            }
            if (buttenEvent == endButton::Press::Long) {
                mState = State::OffForwardEnd;
            }
            if (e == Event::Forward) {
            }
            else if (e == Event::Stop) {
                mState = State::OffWait;
            }
            else if (e == Event::SetFowardPwm) {
                mState = State::SetForwardPwm;
            }
            else if (e == Event::SetBackwardPwm) {
                mState = State::SetBackwardPwm;
            }
            else if (e != Event::None) {
                mState = State::OffWait;
            }
            break;
        case State::ForwardPassThruWait:
            if (auto pt = passThru()) {
                if (auto pv = PA::value(pt)) {
                    if (auto rv = pv.toInt(); rv < (pa_mid + pa_hysterese)) {
                        mState = State::OffWait;
                    } 
                    else {
                        pwm_value_t pvs(((uint32_t)rv - pa_mid) * pwmForward() / pa_half); 
                        updatePwm(pvs);
                    }
                    mStateTick.on(ccTimeout(), []{
                        mState = State::ForwardPassThru;
                    });
                }         
            }
            else {
                mState = State::OffWait;
            }
            break;
        case State::ForwardPassThru:
            if (auto pt = passThru()) {
                if (auto pv = PA::value(pt)) {
                    lrv = pv.toInt();
                    if (auto rv = pv.toInt(); rv < (pa_mid + pa_hysterese)) {
                        mState = State::OffWait;
                    } 
                    else {
                        if (buttenEvent == endButton::Press::Long) {
                            mState = State::OffForwardEnd;
                        }
                        else {
                            //                            pwm_value_t::_;
                            if (mActualAdc >= forwardOffCurr(rv)) {
                                mState = State::OffForwardOC;
                            }
                            pwm_value_t pvs(((uint32_t)rv - pa_mid) * pwmForward() / pa_half); 
                            updatePwm(pvs);
                        }
                    }
                }         
                else {
                    mState = State::OffWait;
                }
            }
            else {
                mState = State::OffWait;
            }
            break;
        case State::OffForwardOC:
            if (auto pt = passThru()) {
                if (auto pv = PA::value(pt)) {
                    if (auto rv = pv.toInt(); (rv > (pa_mid - pa_hysterese)) && (rv < (pa_mid + pa_hysterese))) {
                        mState = State::OffWait;
                    } 
                }
            }
            else {
                if (e == Event::Stop) {
                    mState = State::OffWait;
                }
            }
            break;
        case State::OffForwardEnd:
            if (e == Event::Backward) {
                mState = State::BackwardWait;
            }
            break;
        case State::BackwardWait:
            mRampTick.on(rampCycles(), []{
                if (++mRampPwm <= pwmBackward()) {
                    updatePwm(mRampPwm);
                }
            });
            mStateTick.on(ccTimeout(), []{
                mState = State::Backward; 
            });
            if (e == Event::Backward) {
            }
            else if (e == Event::Stop) {
                mState = State::OffWait;
            }
            else if (e != Event::None) {
                mState = State::OffWait;
            }
            break;
        case State::Backward:
            if (mActualAdc >= backwardOffCurr()) {
                mState = State::OffBackwardOC;
            }
            if (buttenEvent == endButton::Press::Long) {
                mState = State::OffBackwardEnd;
            }
            if (e == Event::Backward) {
            }
            else if (e == Event::SetBackwardPwm) {
                mState = State::SetBackwardPwm;
            }
            else if (e != Event::None) {
                mState = State::OffWait;
            }
            break;
        case State::BackwardPassThruWait:
            if (auto pt = passThru()) {
                if (auto pv = PA::value(pt)) {
                    if (auto rv = pv.toInt(); rv > (pa_mid - pa_hysterese)) {
                        mState = State::OffWait;
                    } 
                    else {
                        pwm_value_t pvs(((uint32_t)pa_mid - rv) * pwmBackward() / pa_half); 
                        updatePwm(pvs);
                    }
                }         
                mStateTick.on(ccTimeout(), []{
                    mState = State::BackwardPassThru;
                });
            }
            else {
                mState = State::OffWait;
            }
            break;
        case State::BackwardPassThru:
            if (auto pt = passThru()) {
                if (auto pv = PA::value(pt)) {
                    if (auto rv = pv.toInt(); rv > (pa_mid - pa_hysterese)) {
                        mState = State::OffWait;
                    } 
                    else {
                        if (buttenEvent == endButton::Press::Long) {
                            mState = State::OffBackwardEnd;
                        }
                        else {
                            if (mActualAdc >= backwardOffCurr(rv)) {
                                mState = State::OffBackwardOC;
                            }
                            pwm_value_t pvs(((uint32_t)pa_mid - rv) * pwmBackward() / pa_half); 
                            updatePwm(pvs);
                        }
                    }
                }         
            }
            else {
                mState = State::OffWait;
            }
            break;
        case State::OffBackwardOC:
            if (auto pt = passThru()) {
                if (auto pv = PA::value(pt)) {
                    if (auto rv = pv.toInt(); (rv > (pa_mid - pa_hysterese)) && (rv < (pa_mid + pa_hysterese))) {
                        mState = State::OffWait;
                    } 
                }
            }
            else {
                if (e == Event::Stop) {
                    mState = State::OffWait;
                }
            }
            break;
        case State::OffBackwardEnd:
            if (e == Event::Forward) {
                mState = State::ForwardWait;
            }
            break;
        }
        if (oldState != mState) {
            stateP::set(mState);
            mStateTick.reset();
            mRampTick.reset();
            switch(mState) {
            case State::Init:
                etl::outl<Term>("S i"_pgm);
                break;
            case State::OffWait:
                etl::outl<Term>("S ow"_pgm);
                off();
                break;
            case State::Off:
                etl::outl<Term>("S o"_pgm);
                updatePwm(pwm_value_t{0});
                off();
                break;
            case State::SetForwardPwm:
                forward();
                updateForwardPwm();
                etl::outl<Term>("S sf"_pgm);
                break;
            case State::ForwardPwmSetPause:
                etl::outl<Term>("S sfp"_pgm);
                off();
                break;
            case State::Reset:
                etl::outl<Term>("S reset"_pgm);
                off();
                break;
            case State::SetParameter:
                etl::outl<Term>("S param"_pgm);
                off();
                break;
            case State::SetParameterWait:
                etl::outl<Term>("S paramW"_pgm);
                off();
                break;
            case State::SetBackwardPwm:
                backward();
                updateBackwardPwm();
                etl::outl<Term>("S sb"_pgm);
                break;
            case State::BackwardPwmSetPause:
                etl::outl<Term>("S sbp"_pgm);
                off();
                break;
            case State::ForwardWait:
                etl::outl<Term>("S fw"_pgm);
                updatePwm(pwm_value_t{0});
                mRampPwm.set(0);
                forward();
                break;
            case State::Forward:
                etl::outl<Term>("S f"_pgm);
                forward();
                break;
            case State::ForwardPassThruWait:
                etl::outl<Term>("S ftw"_pgm);
                forward();
                break;
            case State::ForwardPassThru:
                etl::outl<Term>("S ft"_pgm);
                forward();
                break;
            case State::OffForwardOC:
                etl::outl<Term>("S ocf: "_pgm, mActualAdc);
                off();
                break;
            case State::OffForwardEnd:
                etl::outl<Term>("S ofe: "_pgm, mActualAdc);
                off();
                break;
            case State::BackwardWait:
                etl::outl<Term>("S bw"_pgm);
                updatePwm(pwm_value_t{0});
                mRampPwm.set(0);
                backward();
                break;
            case State::Backward:
                etl::outl<Term>("S b"_pgm);
                backward();
                break;
            case State::BackwardPassThruWait:
                etl::outl<Term>("S btw"_pgm);
                backward();
                break;
            case State::BackwardPassThru:
                etl::outl<Term>("S bt"_pgm);
                backward();
                break;
            case State::OffBackwardOC:
                etl::outl<Term>("S ocb"_pgm);
                off();
                break;
            case State::OffBackwardEnd:
                etl::outl<Term>("S obe: "_pgm, mActualAdc);
                off();
                break;
            }
        }
    }
    
    static inline void debug() {
        if constexpr(N == 0) {
            etl::out<Term>("Ch"_pgm, N, " St: "_pgm, uint8_t(mState), " c: "_pgm, mActualAdc, " lrv: "_pgm, lrv);
            //            etl::out<Term>(" to:"_pgm, ccTimeout().value, " rc: "_pgm, rampCycles().value);
            //            etl::out<Term>(" fpw: "_pgm, pwmForward(), " bpw: "_pgm, pwmBackward());
            //            etl::out<Term>(" frc: "_pgm, forwardRefCurr(), " foc: "_pgm, forwardOffCurr(), " sfoc: "_pgm, forwardStartOffCurr());
            etl::outl<Term>(" foc: "_pgm, forwardOffCurr(), " sfoc: "_pgm, forwardStartOffCurr());
            //            etl::outl<Term>(" brc: "_pgm, backwardRefCurr(), " boc: "_pgm, backwardOffCurr());
        }
    }
    
    static inline ADC::value_type adcValue() {
        return mActualAdc;
    }
    
private:
    inline static uint16_t lrv{};
    
    inline static void updatePwm(pwm_value_t p) {
        mActualPwm = p;
        PWM::template pwm<N>(p);
    }
    
    inline static ADC::value_type updateAdc() {
#ifdef SCALE_ADC_PWM
        return mActualAdc = ADC::value(adci, mActualPwm);
#else
        return mActualAdc = ADC::value(adci);
#endif
    }
    
    inline static void updateForwardPwm() {
        updateAdc();
        pwmForward().set(((uint16_t)pwm_value_t::Upper * (mEventValue - pvalue_t::Lower)) / (pvalue_t::Upper - pvalue_t::Lower));
        updatePwm(pwmForward());
        forwardOffCurr() = defaultOC(mActualAdc);
        forwardRefCurr() = mActualAdc;
        change();
    }
    inline static void updateBackwardPwm() {
        updateAdc();
        pwmBackward().set(((uint16_t)pwm_value_t::Upper * (mEventValue - pvalue_t::Lower)) / (pvalue_t::Upper - pvalue_t::Lower));
        updatePwm(pwmBackward());
        backwardOffCurr() = defaultOC(mActualAdc);
        backwardRefCurr() = mActualAdc;
        change();
    }
    
    inline static uint16_t defaultOC(const uint16_t& i) {
        return (3 * i) / 2;
    }
    
    inline static void forward() {
        InA::on();
        InB::off();
    }
    inline static void backward() {
        InA::off();
        InB::on();
    }
    inline static void off() {
        InA::off();
        InB::off();
    }
    
    static inline pwm_value_t mRampPwm{};
    static inline pwm_value_t mActualPwm{};
    static inline ADC::value_type mActualAdc{};
    
    static inline Event mEvent{Event::None};
    inline static pvalue_t mEventValue{};
    
    inline static External::Tick<Timer> mRampTick;
    inline static External::Tick<Timer> mStateTick;
    inline static State mState{State::Init};
};
template<typename Devs, typename ChList>
struct GlobalFsm;

template<typename Devs, typename... Chs>
struct GlobalFsm<Devs, Meta::List<Chs...>> {
    using bus_type = Devs::bus_type;
    using devs = bus_type::devs;
    
    using Sensor = Devs::sensor;
    using Servo = Devs::servo;
    using Adc = Devs::adcController;
    
    static inline constexpr uint8_t sensorUartNumber = Sensor::uart::number;
    using lut3 = Ccl::SimpleLut<3, Ccl::Input::Mask,  Ccl::Input::Mask, Ccl::Input::Usart<sensorUartNumber>>;
    
    using sigrow = AVR::SigRow<>;
    using syscfg = AVR::Cpu::SysCfg<>;
    using channel_list = Meta::List<Chs...>;
    static_assert(Meta::size_v<channel_list> <= 4, "too much channels");
    static_assert(Meta::is_set_v<channel_list>, "channels must be different");
    
    using adi_t = Adc::index_type;
    
    using servo_pa = Devs::servo_pa;
    using sensor_pa = Devs::sensor::uart::protocoll_adapter_type;
    
    using ch_t = servo_pa::channel_t;
    
    enum class State : uint8_t {Undefined, StartWait, SearchChannel, AfterSearch, 
                                InitRun, Run,  
                                ShowAddress, ShowAddressWait, LearnTimeout,
                                MasterReset, MasterResetWait, MasterResetWait2};
    
    using Timer = Devs::systemTimer;
    
    static constexpr External::Tick<typename Devs::systemTimer> learnTimeoutTicks{4000_ms};
    static constexpr External::Tick<typename Devs::systemTimer> scanTimeoutTicks{50_ms};
    static constexpr External::Tick<typename Devs::systemTimer> waitTimeoutTicks{3000_ms};
    static constexpr External::Tick<typename Devs::systemTimer> signalTimeoutTicks{500_ms};
    static constexpr External::Tick<typename Devs::systemTimer> eepromTimeout{2000_ms};
    static constexpr External::Tick<typename Devs::systemTimer> debugTimeout{500_ms};
    static constexpr External::Tick<typename Devs::systemTimer> masterResetTimeout{4000_ms};
    
    using blinker = External::Blinker2<typename devs::led1, typename Devs::systemTimer, 100_ms, 2000_ms>;
    using blinker2 = External::Blinker2<typename devs::led2, typename Devs::systemTimer, 100_ms, 2000_ms>;
    using bl_count_t = blinker::count_type;
    using count_t = blinker2::count_type;
    
    using NVM = Devs::eeprom;
    using Term = Devs::terminal;
    using PWM = Devs::pwm;
    using AS = Devs::analogSwitch;
    using SW = Devs::gswitch;
    
    using jumper = devs::jp1;
    using jp1pin = devs::JP1Pin;
    
    inline static auto& data() {
        return NVM::data();
    }
    
    inline static void init(const bool inverted) {
        mInverted = inverted;
        NVM::init();
        if (data().magic() != 42) {
            data().magic() = 42;
            data().clear();
            data().change();
            etl::outl<Term>("e init"_pgm);
        }
        blinker::init();
        blinker2::init();
        
        if constexpr(External::Bus::isIBus<bus_type>::value) {
            lut3::init(std::byte{0x00}); // low on lut3-out 
            Sensor::init();
            Sensor::uart::txOpenDrain();
            Servo::template init<BaudRate<115200>>();
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            lut3::init(std::byte{0x0f}); // route TXD (inverted) to lut3-out 
            Sensor::init();
            Sensor::uart::txPinDisable();
            
            sensor_pa::id(data().physicalId());
            
            Servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
        
        Adc::mcu_adc_type::nsamples(4); 
        Adc::init();
        PWM::init();
        PWM::template prescale<6>();
        PWM::template off<4>();
        PWM::template off<5>();
        (Chs::init(), ...);
        AS::init();
        jumper::init();
//        jumper::pin_type::template pullup<true>();
//        jp1pin::template pullup<true>();
        
    }
    inline static void periodic() {
        NVM::saveIfNeeded([&]{
            etl::outl<Term>("ep s"_pgm);
        });
        Adc::periodic();
        Servo::periodic();
        Sensor::periodic();
        (Chs::periodic(), ...);
    }
    inline static void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        ++mEepromTick;
        ++mDebugTick;

        servo_pa::ratePeriodic();
        Sensor::ratePeriodic();
        SW::ratePeriodic();
        AS::ratePeriodic();

        etl::circular_call(Chs::ratePeriodic ...); // verhindert Zeitüberlauf in der Hauptschleife
        //        (Chs::ratePeriodic(), ...);
        
        blinker::ratePeriodic();
        blinker2::ratePeriodic();
        switch(mState) {
        case State::Undefined:
            mState = State::StartWait;
            break;
        case State::StartWait:
            if (jumper::isActive()) {
                mState = State::MasterReset;
            }
            else {
                mStateTick.on(waitTimeoutTicks, []{
                    blinker::off();
                    mState = State::SearchChannel;
                });
            }
            break;
        case State::MasterReset:
            mStateTick.on(masterResetTimeout, []{
                mState = State::MasterResetWait;
            });
            break;
        case State::MasterResetWait:
            if (!jumper::isActive()) {
                mState = State::MasterResetWait2;
            }
            break;
        case State::MasterResetWait2:
            mStateTick.on(waitTimeoutTicks, []{
                mState = State::StartWait;
            });
            break;
        case State::SearchChannel:
            if (search()) {
                mState = State::AfterSearch;
            }
            mStateTick.on(learnTimeoutTicks, []{
                mState = State::LearnTimeout;
            });
            break;
        case State::AfterSearch:
            mStateTick.on(signalTimeoutTicks, []{
                mState = State::ShowAddress;
            });
            break;
        case State::LearnTimeout:
            etl::outl<Term>("timeout ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            if (NVM::data().channel() && NVM::data().address()) {
                SW::channel(NVM::data().channel());
                SW::address(typename SW::addr_t{NVM::data().address().toInt()});
            }
            mState = State::InitRun;
            break;
        case State::ShowAddress:
            etl::outl<Term>("learned ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            blinker::onePeriod(bl_count_t(data().address().toInt() + 1));
            mState = State::ShowAddressWait;
            break;
        case State::ShowAddressWait:
            if (!blinker::isActive()) {
                mState = State::InitRun;
            }
            break;
        case State::InitRun:
            mState = State::Run;
            break;
        case State::Run:
            Meta::visit<channel_list>([]<typename C>(Meta::Wrapper<C>){
                                          if (C::stateP::changed()) {
                                              if ((C::stateP::oc())) {
                                                  blinker::steady();
                                              }
                                              else if ((C::stateP::end())) {
                                                  blinker::blink(bl_count_t{1});
                                              }
                                              else if ((C::stateP::error())) {
                                                  blinker::blink(bl_count_t{2});
                                              }
                                              else if ((C::stateP::changingParameters())) {
                                                  blinker::blink(bl_count_t{4});
                                              }
                                              else if ((C::stateP::resetted())) {
                                                  blinker::blink(bl_count_t{6});
                                              }
                                              else {
                                                  blinker::off();
                                              }
                                          }
                                      });                            
            
            mEepromTick.on(eepromTimeout, []{
//                etl::outl<Term>("exp"_pgm);
                NVM::data().expire();
            });
            mDebugTick.on(debugTimeout, debug);
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::MasterReset:
                etl::outl<Term>("MR"_pgm);
                blinker::steady();
                blinker2::steady();
                data().clear();
                data().change();
                break;
            case State::MasterResetWait:
                etl::outl<Term>("MRW"_pgm);
                blinker::off();
                blinker2::off();
                break;
            case State::MasterResetWait2:
                break;
            case State::StartWait:
                etl::outl<Term>("SW"_pgm);
                if constexpr(External::Bus::isIBus<bus_type>::value) {
                    etl::outl<Term>("rcQ: IBus"_pgm);
                    blinker2::blink(count_t{1});
                }
                else if constexpr(External::Bus::isSBus<bus_type>::value) {
                    etl::outl<Term>("rcQ: SBus"_pgm);
                    if (mInverted) {
                        blinker2::blink(count_t{2});
                        Servo::rxInvert(true);
                    }
                    else {
                        blinker2::blink(count_t{3});
                    }
                }
                else {
                    static_assert(std::false_v<Term>, "wrong bus");    
                }
                etl::outl<Term>("sig: "_pgm, sigrow::id()[0], sigrow::id()[1], sigrow::id()[2], syscfg::revision());
                break;
            case State::SearchChannel:
                etl::outl<Term>("SC"_pgm);
                break;
            case State::AfterSearch:
                etl::outl<Term>("AS"_pgm);
                break;
            case State::ShowAddress:
                etl::outl<Term>("SA"_pgm);
                break;
            case State::ShowAddressWait:
                etl::outl<Term>("SAW"_pgm);
                break;
            case State::LearnTimeout:
                etl::outl<Term>("LW"_pgm);
                break;
            case State::InitRun:
                etl::outl<Term>("IR"_pgm);
                break;
            case State::Run:
                etl::outl<Term>("RU"_pgm);
                break;
            }
        }
    }
private:
    static inline void debug() {
#ifndef NDEBUG
        if (xassert::on) {
//            etl::outl<Term>("aassert: "_pgm, assertKey);
            etl::outl<Term>("a: "_pgm, assertKey, " : "_pgm, xassert::ab);
            xassert::on = false;
            assertKey = 0;
        }
#endif
        (Chs::debug(),...);
        //                etl::circular_call(Chs::debug...);        
//        etl::outl<Term>("id: "_pgm, (uint8_t)NVM::data().physicalId());
//        etl::outl<Term>("es: "_pgm, AS::states());
        const auto v = servo_pa::valueMapped(NVM::data().channel());
        etl::outl<Term>("ch: "_pgm, v.toInt(), " A: "_pgm, protocol_t::toAddress(v), " F: "_pgm, protocol_t::toIndex(v), " M: "_pgm, protocol_t::toMode(v));
        etl::outl<Term>("ch: "_pgm, v.toInt(), " A: "_pgm, protocol_t::toAddress(v), " P: "_pgm, protocol_t::toParameter(v), " V: "_pgm, protocol_t::toParameterValue(v));
    }
    using protocol_t = typename SW::protocol_t;
    using addr_t = RCSwitch::addr_t;
    
    static inline bool search() {
        if (learnChannel < 16) {
            if (const auto lc = servo_pa::valueMapped(learnChannel.toRangedNaN()); lc && SW::isLearnCode(lc)) {
                etl::outl<Term>("ch: "_pgm, learnChannel.toInt(), " : "_pgm, lc.toInt());
                if (const auto pv = protocol_t::toParameterValue(lc).toInt(); (pv >= 1) && ((pv - 1) <= RCSwitch::addr_t::Upper)) {
                    const uint8_t addr = pv - 1;
                    SW::channel(learnChannel.toRangedNaN());
                    SW::address(addr_t(addr));
                    NVM::data().channel() = learnChannel;
                    NVM::data().address() = addr;
                    NVM::data().change();
                    return true;
                }
            }   
        }
#ifdef LEARN_DOWN
        --learnChannel;
#else
        ++learnChannel;
#endif
        return false;
    }
#ifdef LEARN_DOWN
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{ch_t::Upper};
#else
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{0};
#endif
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mEepromTick;
    static inline External::Tick<Timer> mDebugTick;
    static inline State mState{State::Undefined};
    static inline bool mInverted{false};
};

template<typename ADC, uint8_t Ch, uint16_t Ro, uint16_t... Rs>
struct AnalogSwitch {
    using rlist = Meta::NList<Rs...>::list_type;
    
    static inline constexpr auto limits = []{
        std::array<std::pair<uint16_t, uint16_t>, (1 << Meta::size_v<rlist>)> vv;
        auto parallel = [](const double a, const double b){
            double z = a * b;
            double n = a + b;
            if (n != 0) {
                return z / n; 
            }
            return 0.0;
        };
        
        for(uint8_t i = 0; i < vv.size(); ++i) {
            double r0 = 1000'000'000.0;
            double r1 = 1000'000'000.0;
            double r2 = 1000'000'000.0;
            double r3 = 1000'000'000.0;
            if (i & 0x01) {
                r0 = Meta::nth_element<0, rlist>::value;
            }
            if (i & 0x02) {
                r1 = Meta::nth_element<1, rlist>::value;
            }
            if (i & 0x04) {
                r2 = Meta::nth_element<2, rlist>::value;
            }
            if (i & 0x08) {
                r3 = Meta::nth_element<3, rlist>::value;
            }
            const auto r = parallel(parallel(parallel(r0, r1), r2), r3);

            const double Vref = 4.096;
            const double amax = 4095;
            const double vmax = 5.0;
            const double p = 0.01;

            const double v = vmax * r / (r + Ro); 
            const double vl = v * (1.0 - p);
            const double al = vl * amax / Vref;
            const double vh = v * (1.0 + p);
            const double ah = vh * amax / Vref;
            
            vv[i].first = std::min(amax, al);
            vv[i].second= std::min(amax, ah);
        }
        return vv;
    }();
    static inline constexpr auto overlap = []{
        for(uint8_t i = 1; i < limits.size(); ++i) {
            for(uint8_t n = (i + 1); n < limits.size(); ++n) {
                if ((limits[i].first <= limits[n].first) && (limits[i].second >= limits[n].first)) {
                    return true;
                }                
                if ((limits[i].first <= limits[n].second) && (limits[i].second >= limits[n].second)) {
                    return true;
                }                
            }
        }
        return false;
    }();
    static_assert(!overlap);

    template<uint8_t SW>
    struct Switch {
        static_assert(SW < Meta::size_v<rlist>);
        
        static inline constexpr std::byte mask{1 << SW};
        
        static inline void init(){}
        
        static inline bool isActive() {
            return std::any(sstates & mask);
        }

    };

    static inline void init(){}

    static inline void ratePeriodic(){
        const auto v = ADC::value(typename ADC::index_type{Ch});
        for(uint8_t i{0}; i < limits.size(); ++i) {
            if (const auto vi = v.toInt(); (limits[i].first <= vi) && (vi <= limits[i].second)) {
                sstates = std::byte{i};                        
            }
        }
    }

    static inline std::byte states() {
        return sstates;
    }
private:
    static inline std::byte sstates{};
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;

    using servoPosition = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Servo / DBG
    using scanDevPosition = servoPosition;
    
    using sensorPosition = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Sensor
    using scanTermPosition = sensorPosition;
#ifdef NDEBUG
    using scan_term_dev = void;
#else
    using scan_term_dev = Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#endif
  
    using ppmDevPosition = void;
    using evrouter = void;
    
    using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
    using pwm = PWM::DynamicPwm8Bit<tcaPosition>;

    using JP1Pin  = Pin<Port<A>, 5>; 
      
    using led2Pin  = Pin<Port<F>, 4>; 
    using scanLedPin = AVR::ActiveHigh<led2Pin, Output>;
    
    using led1Pin  = Pin<Port<D>, 3>; 
    using assertPin = led1Pin;
    
    using pwmPin1 = Pin<Port<A>, 0>; 
    using pwmPin2 = Pin<Port<A>, 1>; 
    using pwmPin3 = Pin<Port<A>, 2>; 
    using pwmPin4 = Pin<Port<A>, 3>; 
    
    using inA1Pin = Pin<Port<F>, 5>; 
    using inB1Pin = Pin<Port<D>, 0>; // HW03: Brücke auf Platine 
    using inA2Pin = Pin<Port<D>, 4>; 
    using inB2Pin = Pin<Port<D>, 6>; 
    using inA3Pin = Pin<Port<C>, 3>; 
    using inB3Pin = Pin<Port<C>, 2>; 
    using inA4Pin = Pin<Port<A>, 6>; 
    using inB4Pin = Pin<Port<A>, 7>; 
    
    //using dgbPin = Pin<Port<C>, 0>; // tx 
    using ibusPin = Pin<Port<C>, 1>; // rx 
    
    using csf1Pin = Pin<Port<F>, 2>; // ADC 18 (avr128da32)
    using csf2Pin = Pin<Port<D>, 7>; // ADC 7
    using csf3Pin = Pin<Port<D>, 5>; // ADC 5
    using csf4Pin = Pin<Port<D>, 1>; // ADC 1 
    using endSPin = Pin<Port<D>, 2>; // ADC 2 // HW03: falsch angeschlossen
    
    using daisyChain= Pin<Port<A>, 4>; 
    
    template<typename T>
    struct PseudoTimer {
        inline static constexpr auto intervall = T::intervall * 4;
    };
    using pseudoTimer = PseudoTimer<systemTimer>;

//    using eeprom = EEProm::Controller<Storage::ApplData<pseudoTimer, uint8_t, 
//                                      RCSwitch::addr_t, typename pwm::value_type>>;
            
    using adc = Adc<Component::Adc<0>, AVR::Resolution<12>, Vref::V4_096>;
    using adcController = External::Hal::AdcController<adc, Meta::NList<18, 7, 5, 1, 2, 0x42>>; // 42 = temp
    
    using analogSwitch = AnalogSwitch<adcController, 4, Ro, R0, R1, R2, R3>;
    using sw0 = analogSwitch::Switch<0>;
    using sw1 = analogSwitch::Switch<1>;
    using sw2 = analogSwitch::Switch<2>;
    using sw3 = analogSwitch::Switch<3>;
    
    using led1 = AVR::ActiveHigh<led1Pin, Output>;
    using led2 = AVR::ActiveHigh<led2Pin, Output>;
    
    using jp1 = AVR::ActiveLow<JP1Pin, Input>;
    
    static inline void init() {
        using portmux = Portmux::StaticMapper<Meta::List<servoPosition, sensorPosition>>; // tcaPosition
        portmux::init();
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDa32<MCU>) {
                clock::template init<Project::Config::fMcuMhz>();
            }
            else if constexpr(AVR::Concepts::At01Series<MCU>) {
                clock::template prescale<1>();
            }
            else {
                static_assert(std::false_v<MCU>);
            }
        });
        systemTimer::init(); 
    }
    static inline void periodic() {}
};

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    using bus_type = External::Bus::IBusIBus<Devs>;

    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::High>;
        inline static constexpr auto stateProviderId = IBus2::Type::type::FLIGHT_MODE;
    };
    
    using systemTimer = Devs::systemTimer;
    using pseudoTimer = Devs::pseudoTimer;
    using pwm = typename Devs::pwm;

    using adc = Devs::adc;
    
    using analogSwitch = Devs::analogSwitch;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, 
    AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using channel_t = servo_pa::channel_t;
   
    using eeprom = EEProm::Controller<Storage::ApplData<pseudoTimer, channel_t, 
                                      RCSwitch::addr_t, typename pwm::value_type>>;
    
    #ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
    #else
    using terminal = etl::basic_ostream<void>;
    #endif

//    using eeprom = Devs::eeprom;    
    
    using adcController = Devs::adcController;

    using ch0 = ChannelFsm<BusParam, pseudoTimer, 0, pwm, adcController, typename Devs::inA1Pin, typename Devs::inB1Pin, typename Devs::sw0, servo_pa, eeprom, terminal>;
    using ch1 = ChannelFsm<BusParam, pseudoTimer, 1, pwm, adcController, typename Devs::inA2Pin, typename Devs::inB2Pin, typename Devs::sw1, servo_pa, eeprom, terminal>;
    using ch2 = ChannelFsm<BusParam, pseudoTimer, 2, pwm, adcController, typename Devs::inA3Pin, typename Devs::inB3Pin, typename Devs::sw2, servo_pa, eeprom, terminal>;
    using ch3 = ChannelFsm<BusParam, pseudoTimer, 3, pwm, adcController, typename Devs::inA4Pin, typename Devs::inB4Pin, typename Devs::sw3, servo_pa, eeprom, terminal>;

    using ch_list = Meta::List<ch0, ch1, ch2, ch3>;
    
    using vn1 = External::AnalogConverter<ch0, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;
    using vn2 = External::AnalogConverter<ch1, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;
    using vn3 = External::AnalogConverter<ch2, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;
    using vn4 = External::AnalogConverter<ch3, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;

    template<typename Sensor>
    struct CProvider {
        inline static constexpr auto ibus_type = IBus2::Type::type::BAT_CURR;
        inline static void init() {
        }
        inline static uint16_t value() {
            return Sensor::value();
        }
    };
    
    using cp1 = CProvider<vn1>;
    using cp2 = CProvider<vn2>;
    using cp3 = CProvider<vn3>;
    using cp4 = CProvider<vn4>;
    
    using sp1 = ch0::stateP;
    using sp2 = ch1::stateP;
    using sp3 = ch2::stateP;
    using sp4 = ch3::stateP;

    template<typename DC>
    struct IBusThrough {
        inline static void init() {
            DC::template dir<Output>();
        }
        inline static void on() {
            DC::on();
        }
        inline static void off() {
            DC::off();
        }
    };
    using ibt = IBusThrough<typename Devs::daisyChain>;
    
    using sensor = IBus2::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<115200>, 
    Meta::List<cp1, cp2, cp3, cp4, sp1, sp2, sp3, sp4>, systemTimer, ibt
    //                          , etl::NamedFlag<true>
    //                           , etl::NamedFlag<true>
    >;
    
    using gswitch = IBus::Switch::GeneralSwitch<BusParam, servo_pa, sensor, Meta::List<ch0, ch1, ch2, ch3>, eeprom>;
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    using bus_type = External::Bus::SBusSPort<Devs>;

    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::Low>;
        inline static constexpr auto stateProviderId = External::SPort::ValueId::DIY;
    };
    
    using systemTimer = Devs::systemTimer;
    using pseudoTimer = Devs::pseudoTimer;
    using pwm = typename Devs::pwm;

    using adc = Devs::adc;
    
    using analogSwitch = Devs::analogSwitch;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, 
    AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using channel_t = servo_pa::channel_t;
    
    using eeprom = EEProm::Controller<Storage::ApplData<pseudoTimer, channel_t, 
                                      RCSwitch::addr_t, typename pwm::value_type>>;
    
#ifndef NDEBUG
using terminal = etl::basic_ostream<servo>;
#else
using terminal = etl::basic_ostream<void>;
#endif
    
//    using eeprom = Devs::eeprom;    
    
    using adcController = Devs::adcController;  
 
    using ch0 = ChannelFsm<BusParam, pseudoTimer, 0, pwm, adcController, typename Devs::inA1Pin, typename Devs::inB1Pin, typename Devs::sw0, servo_pa, eeprom, terminal>;
    using ch1 = ChannelFsm<BusParam, pseudoTimer, 1, pwm, adcController, typename Devs::inA2Pin, typename Devs::inB2Pin, typename Devs::sw1, servo_pa, eeprom, terminal>;
    using ch2 = ChannelFsm<BusParam, pseudoTimer, 2, pwm, adcController, typename Devs::inA3Pin, typename Devs::inB3Pin, typename Devs::sw2, servo_pa, eeprom, terminal>;
    using ch3 = ChannelFsm<BusParam, pseudoTimer, 3, pwm, adcController, typename Devs::inA4Pin, typename Devs::inB4Pin, typename Devs::sw3, servo_pa, eeprom, terminal>;
    
    using ch_list = Meta::List<ch0, ch1, ch2, ch3>;
    
    using vn1 = External::AnalogConverter<ch0, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<10,1>>;
    using vn2 = External::AnalogConverter<ch1, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<10,1>>;
    using vn3 = External::AnalogConverter<ch2, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<10,1>>;
    using vn4 = External::AnalogConverter<ch3, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<10,1>>;

    template<typename PA>
    using sensorUsart = AVR::Usart<typename Devs::sensorPosition, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
    
    template<typename Sensor>
    struct CProvider {
        inline static constexpr auto valueId = External::SPort::ValueId::Current;
        inline static uint32_t value() {
            return Sensor::value();
        }
    };
    
    using cp1 = CProvider<vn1>;
    using cp2 = CProvider<vn2>;
    using cp3 = CProvider<vn3>;
    using cp4 = CProvider<vn4>;
    
    using sp1 = ch0::stateP;
    using sp2 = ch1::stateP;
    using sp3 = ch2::stateP;
    using sp4 = ch3::stateP;
    
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, 
    Meta::List<cp1, cp2, cp3, cp4, sp1, sp2, sp3, sp4>>;

    using gswitch = IBus::Switch::GeneralSwitch<BusParam, servo_pa, sensor, Meta::List<ch0, ch1, ch2, ch3>, eeprom>;
};


template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isIBus<BusSystem>::value || External::Bus::isSBus<BusSystem>::value) {
            using terminal = devs::terminal;
            using systemTimer = devs::systemTimer;
            using gfsm = GlobalFsm<devs, typename devs::ch_list>;
            
            gfsm::init(inverted);
            etl::outl<terminal>("avr128da32_04"_pgm);
            while(true) {
                devs::servo::periodic();
                gfsm::periodic(); 
                systemTimer::periodic([&]{
                    gfsm::ratePeriodic();
                });
            }
        }
    }
};

using devices = Devices<>;
using scanner = External::Scanner<devices, Application>;

int main() {
    scanner::run();
}

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
//    using terminal = etl::basic_ostream<devices::scan_term_dev>;
//    devices::assertPin::dir<AVR::Output>();
//    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    xassert::ab.clear();
    xassert::ab.insertAt(0, expr);
    etl::itoa(line, xassert::aline);
    xassert::ab.insertAt(20, xassert::aline);
    xassert::ab.insertAt(30, file);
    xassert::on = true;
//    while(true) {
//        devices::assertPin::toggle();
//    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
