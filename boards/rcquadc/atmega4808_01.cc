#define NDEBUG

//#define USE_IBUS

#define USE_SBUS

#ifdef USE_SBUS
# define USE_INVERTED_SBUS
# define USE_SPORT
#endif

#define LEARN_DOWN

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

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace IBus::Switch {
    template<typename PA, typename Sensor, typename ActorList, typename NVM>
    struct GeneralSwitch;
    
    template<typename PA, typename Sensor, typename... Actors, typename NVM>
    struct GeneralSwitch<PA, Sensor, Meta::List<Actors...>, NVM> {
        using actor_list = Meta::List<Actors...>;
        using channel_t = PA::channel_t;
        using value_t = PA::value_type;
        
        using protocol_t = Protocol1;
        
        using addr_t = Protocol1::addr_t;
        using index_t = Protocol1::index_t;
        using mode_t = Protocol1::mode_t;
        using param_t = Protocol1::param_t;
        using pvalue_t = Protocol1::pvalue_t;
        
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
            return Protocol1::isLearnCode(v);
        }
        static inline bool ratePeriodic() {
            const auto cv = PA::valueMapped(mChannel);
            
            if (!cv) return true;
            
            const addr_t addr = Protocol1::toAddress(cv);
            const index_t index = Protocol1::toIndex(cv);
            const mode_t mode = Protocol1::toMode(cv);
            
            if (Protocol1::isControlMessage(cv)) { // control
                const param_t param = Protocol1::toParameter(cv);
                const pvalue_t value = Protocol1::toParameterValue(cv);
                
                if (param == Protocol1::broadCast) {
                    if (value == Protocol1::bCastOff) {
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
                    if (param == Protocol1::reset) {
                        if (value == Protocol1::bCastReset) {
                            Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                          A::reset();
                                                      });
                        }
                    }
                    else if (param == Protocol1::timeMpxMode) { // FrSky: phys. Sensor-Id
                        if ((value >= 1) && (value < External::SPort::sensor_ids.size())) {
#ifdef USE_SBUS
                            const External::SPort::SensorId id = External::SPort::sensor_ids[value - 1];
                            Sensor::ProtocollAdapter::id(id);
                            NVM::data().physicalId(id);
                            NVM::data().change();
#endif
                        }
                    }
                    else if (param == Protocol1::motorRampTime) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setRamp(value);
                                                  });
                    }
                    else if (param == Protocol1::pwm1) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setPwmForward(value);
                                                  });
                    }
                    else if (param == Protocol1::pwm2) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setPwmBackward(value);
                                                  });
                    }
                    else if (param == Protocol1::offCurr1) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setOffCurrentForward(value);
                                                  });
                    }
                    else if (param == Protocol1::offCurr2) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setOffCurrentBackward(value);
                                                  });
                    }
                    else if (param == Protocol1::passThruChannel) {
                        Meta::visitAt<actor_list>(lastOn.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                                      A::setPassThru(value);
                                                  });
                    }
                    else if (param == Protocol1::testMode) {
                        if ((value >= 1) && (value <= 8)) {
                            Meta::visit<actor_list>([&]<typename A>(const Meta::Wrapper<A>&) {
                                                        //                                                        A::setTest(mode_t(value - 1));
                                                    });
                        }   
                    }
                    else if (param == Protocol1::adcSensitivity) {
                    }
                    else if (param == Protocol1::cutOffSensitivity) {
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
                if (mode != Protocol1::off) {
                    lastOnIndex = index.toInt();
                }
                else {
                    lastOnIndex = lastindex_t{};
                }
                Meta::visitAt<actor_list>(index.toInt(), [&]<typename A>(const Meta::Wrapper<A>&) {
                                              if (mode == Protocol1::off) {
                                                  A::stop();
                                              }
                                              else if (mode == Protocol1::forward) {
                                                  if (!gotBCastOff) { // parameter selection
                                                      A::startForward();
                                                  }
                                              }
                                              else if (mode == Protocol1::backward) {
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
        //        Channel::_;
        using Address = etl::uint_ranged_NaN<typename AddressR::value_type, AddressR::Lower, AddressR::Upper>;
        //        Address::_;
        using value_type = Output<Timer, Channel, PWM_T>;
        
        enum class AdcSensitivity : uint8_t {Low, Mid, High};
        
        uint8_t& magic() {
            return mMagic;
        }
        
        void clear() {
            for(value_type& v : mData) {
                v = value_type{};
            }
            mChannel = 14;
            mAddress = 0;
            mAdcSens = AdcSensitivity::Low;
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
        
        value_type& output(const index_type i) {
            return mData[i];
        }
    private:
        uint8_t mMagic;
        Channel mChannel;
        Address mAddress;
        AdcSensitivity mAdcSens{AdcSensitivity::Low};
        std::array<value_type, NChannels> mData;
        External::SPort::SensorId mPhysId{External::SPort::SensorId::ID1};
    };
}

namespace  {
#ifdef USE_HOTT
    constexpr auto fRtc = 500_Hz;
#endif
#ifdef USE_SBUS
    constexpr auto fRtc = 1000_Hz;
#endif
#ifdef USE_IBUS
    constexpr auto fRtc = 1000_Hz;
    //    constexpr auto fRtc = 2000_Hz;
#endif
#ifdef USE_PPM
    constexpr auto fRtc = 128_Hz;
#endif
    //    constexpr uint16_t Ri = 2700;
    constexpr uint16_t Ri = 3000;
}

#ifdef USE_IBUS
template<typename Sensor>
struct CProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::BAT_CURR;
    inline static void init() {
    }
    inline static uint16_t value() {
        return Sensor::value();
    }
};
#endif
#ifdef USE_SPORT
template<typename Sensor>
struct CProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::Current;
    inline static uint32_t value() {
        return Sensor::value();
    }
};
#endif

template<typename Timer, uint8_t N, typename PWM, typename ADC, 
         typename InA, typename InB, typename EndPin, typename PA, typename NVM, typename Term>
struct ChannelFsm {
    using protocol_t = IBus::Switch::Protocol1;
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
                                Off = 1, OffWait, 
                                Forward = 10, Backward, ForwardWait, BackwardWait, 
                                SettingsBegin = 20, SetForwardPwm = SettingsBegin, SetBackwardPwm, SetParameter, SetParameterWait,  
                                SettingsEnd = SetParameterWait, 
                                ForwardPwmSetPause = 30, BackwardPwmSetPause, 
                                OffForwardOC = 40, OffBackwardOC, OffForwardEnd, OffBackwardEnd,
                                ForwardPassThru = 50, BackwardPassThru, ForwardPassThruWait, BackwardPassThruWait};
    
    enum class Event : uint8_t {None, Stop, 
                                Forward, Backward, 
                                SetFowardPwm, SetBackwardPwm, 
                                SetParameter,
                               };
#ifdef USE_IBUS
    struct StateProvider {
        inline static constexpr auto ibus_type = IBus::Type::type::FLIGHT_MODE;

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
            return (mState >= State::SettingsBegin) && (mState <= State::SettingsEnd);
        }
        static inline void error(const bool b) {
            if (mError != b) {
                mError = b;
                mChanged = true;
            }
        }
        [[nodiscard]] static inline bool oc() {
            mChanged = false;   
            return (mState == State::OffForwardOC) || (mState == State::OffBackwardOC);
        }
        [[nodiscard]] static inline bool end() {
            mChanged = false;   
            return (mState == State::OffForwardEnd) || (mState == State::OffBackwardEnd);
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
#endif
#ifdef USE_SBUS
    struct StateProvider {
        inline static constexpr auto valueId = External::SPort::ValueId::DIY;
        static inline void init() {}
        static inline uint32_t value() {
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
            return (mState >= State::SettingsBegin) && (mState <= State::SettingsEnd);
        }
        static inline void error(const bool b) {
            if (mError != b) {
                mError = b;
                mChanged = true;
            }
        }
        [[nodiscard]] static inline bool oc() {
            mChanged = false;   
            return (mState == State::OffForwardOC) || (mState == State::OffBackwardOC);
        }
        [[nodiscard]] static inline bool end() {
            mChanged = false;   
            return (mState == State::OffForwardEnd) || (mState == State::OffBackwardEnd);
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
#endif
    static inline constexpr uint16_t pa_mid = (pa_value_t::Upper + pa_value_t::Lower) / 2;
    static inline constexpr uint16_t pa_half = (pa_value_t::Upper - pa_value_t::Lower) / 2;
    static inline constexpr uint16_t pa_hysterese = 10;
    
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
        if (backwardOffCurr() > backwardStartOffCurr()) {
            const uint16_t cv1 = backwardOffCurr() - backwardStartOffCurr();
            const uint16_t cv2 = ((((uint32_t)pa_mid - pwm) * cv1) / pa_half) + backwardStartOffCurr(); 
            return cv2;
        }
        else {
            return backwardOffCurr();
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
        return NVM::data().output(data_index);
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
        StateProvider::error(false);
        change();
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
        }
        else {
            const auto n = forwardRefCurr()+ ((forwardRefCurr() * v.toInt()) >> 3);
            forwardOffCurr() = n;
            change();
            mEvent = Event::SetParameter;
            mEventValue = v;
            if (n >= ADC::value_type::Upper) {
                StateProvider::error(true);
            }    
            else {
                StateProvider::error(false);
            }
        }
    }
    
    inline static void setOffCurrentBackward(const pvalue_t& v) {
        if (v.isTop()) {
            backwardOffCurr() = std::numeric_limits<uint16_t>::max();
        }
        else {
            const auto n = backwardRefCurr() + ((backwardRefCurr() * v.toInt()) >> 3);
            backwardOffCurr() = n;
            change();
            mEvent = Event::SetParameter;
            mEventValue = v;
            if (n >= ADC::value_type::Upper) {
                StateProvider::error(true);
            }    
            else {
                StateProvider::error(false);
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
            StateProvider::set(mState);
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
        return mActualAdc = ADC::value(adci, mActualPwm);
//        return mActualAdc = ADC::value(adci);
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

template<typename Timer, typename PWM, typename NVM, typename ChList, typename Led, typename Adc, 
         typename Servo, typename Baud, 
         typename Sensor, 
         typename SW, 
         typename Term = void>
struct GlobalFsm;

template<typename Timer, typename PWM, typename NVM, typename Led, typename... Chs, typename Adc, 
         typename Servo, auto baud, 
         typename Sensor, 
         typename SW, 
         typename Term>
struct GlobalFsm<Timer, PWM, NVM, Meta::List<Chs...>, Led, Adc, Servo, BaudRate<baud>, Sensor, SW, Term> {
    using channel_list = Meta::List<Chs...>;
    static_assert(Meta::size_v<channel_list> <= 4, "too much channels");
    static_assert(Meta::is_set_v<channel_list>, "channels must be different");
    
    using adi_t = Adc::index_type;
    
    using protocoll_adapter_t = Servo::protocoll_adapter_type;
    
    using ch_t = protocoll_adapter_t::channel_t;
    
    enum class State : uint8_t {Undefined, StartWait, SearchChannel, AfterSearch, 
                                InitRun, Run,  
                                ShowAddress, ShowAddressWait, LearnTimeout,
                                Debug1};
    
    static constexpr External::Tick<Timer> learnTimeoutTicks{4000_ms};
    static constexpr External::Tick<Timer> scanTimeoutTicks{50_ms};
    static constexpr External::Tick<Timer> waitTimeoutTicks{3000_ms};
    static constexpr External::Tick<Timer> signalTimeoutTicks{500_ms};
    static constexpr External::Tick<Timer> eepromTimeout{2000_ms};
    static constexpr External::Tick<Timer> debugTimeout{500_ms};
    
    using blinker = External::Blinker2<Led, Timer, 100_ms, 2000_ms>;
    using bl_count_t = blinker::count_type;
    
    inline static auto& data() {
        return NVM::data();
    }
    
    inline static void init() {
        NVM::init();
        if (data().magic() != 42) {
            data().magic() = 42;
            data().clear();
            data().change();
        }
        blinker::init();
#ifdef USE_IBUS
        Servo::template init<BaudRate<baud>>();
#endif
#ifdef USE_SBUS
        Servo::template init<AVR::BaudRate<baud>, FullDuplex, true, 1>(); // 8E2
# ifdef USE_INVERTED_SBUS
        Servo::rxInvert(true);
# endif
#endif
        Sensor::init();
        Adc::mcu_adc_type::nsamples(6); 
        Adc::init();
        PWM::init();
        PWM::template prescale<6>();
        (Chs::init(), ...);
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
#ifdef USE_SBUS
        Servo::protocoll_adapter_type::ratePeriodic();
#endif
        Sensor::ratePeriodic();
        SW::ratePeriodic();
        
        const auto oldState = mState;
        ++mStateTick;
        ++mEepromTick;
        
        etl::circular_call(Chs::ratePeriodic ...); // verhindert Zeitüberlauf in der Hauptschleife
        //        (Chs::ratePeriodic(), ...);
        
        blinker::ratePeriodic();
        switch(mState) {
        case State::Undefined:
            mState = State::StartWait;
            blinker::steady();
            break;
        case State::StartWait:
            mStateTick.on(waitTimeoutTicks, []{
                blinker::off();
                mState = State::SearchChannel;
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
                                          if (C::StateProvider::changed()) {
                                              if ((C::StateProvider::oc())) {
                                                  blinker::steady();
                                              }
                                              else if ((C::StateProvider::end())) {
                                                  blinker::blink(bl_count_t{1});
                                              }
                                              else if ((C::StateProvider::error())) {
                                                  blinker::blink(bl_count_t{2});
                                              }
                                              else if ((C::StateProvider::changingParameters())) {
                                                  blinker::blink(bl_count_t{4});
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
            mStateTick.match(debugTimeout, []{
                mState = State::Debug1;
            });
            break;
        case State::Debug1:
            mState = State::Run;
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::StartWait:
                etl::outl<Term>("rcQ mega4808 01"_pgm);
                break;
            case State::SearchChannel:
            case State::AfterSearch:
            case State::ShowAddress:
            case State::ShowAddressWait:
            case State::LearnTimeout:
            case State::InitRun:
                break;
            case State::Run:
                break;
            case State::Debug1:
                (Chs::debug(),...);
                //                etl::circular_call(Chs::debug...);
                break;
            }
        }
    }
private:
    using protocol_t = typename SW::protocol_t;
    using addr_t = typename protocol_t::addr_t;
    
    static inline bool search() {
        if (learnChannel < 16) {
            if (const auto lc = protocoll_adapter_t::valueMapped(learnChannel.toRangedNaN()); lc && SW::isLearnCode(lc)) {
                etl::outl<Term>("ch: "_pgm, learnChannel.toInt(), " : "_pgm, lc.toInt());
                if (const auto pv = protocol_t::toParameterValue(lc).toInt(); (pv >= 1) && ((pv - 1) <= SW::protocol_t::addr_t::Upper)) {
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
    static inline State mState{State::Undefined};
};

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Servo / DBG
#ifdef USE_IBUS
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Sensor
#endif

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using pwm = PWM::DynamicPwm8Bit<tcaPosition>;
#ifdef USE_IBUS
using portmux = Portmux::StaticMapper<Meta::List<usart1Position, usart2Position>>;
#endif
#ifdef USE_SBUS
using portmux = Portmux::StaticMapper<Meta::List<usart1Position>>;
#endif

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

#ifdef USE_SBUS
using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
#endif
#ifdef USE_IBUS
using servo_pa = IBus::Servo::ProtocollAdapter<0>;
#endif

using servo = Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, 
AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifndef NDEBUG
using terminal = etl::basic_ostream<servo>;
#else
using terminal = etl::basic_ostream<void>;
#endif

using ledPin  = Pin<Port<D>, 2>; 

using dbgPin  = Pin<Port<D>, 4>; 

using pwmPin1 = Pin<Port<A>, 0>; 
using pwmPin2 = Pin<Port<A>, 1>; 
using pwmPin3 = Pin<Port<A>, 2>; 
using pwmPin4 = Pin<Port<A>, 3>; 

using end1Pin = Pin<Port<F>, 5>; 
using end2Pin = Pin<Port<D>, 1>; 
using end3Pin = Pin<Port<A>, 4>; 
using end4Pin = Pin<Port<A>, 5>; 

using end1 = ActiveLow<end1Pin, Input>;
using end2 = ActiveLow<end2Pin, Input>;
using end3 = ActiveLow<end3Pin, Input>;
using end4 = ActiveLow<end4Pin, Input>;

using inA1Pin = Pin<Port<F>, 3>; 
using inB1Pin = Pin<Port<F>, 1>; 
using inA2Pin = Pin<Port<D>, 3>; 
using inB2Pin = Pin<Port<D>, 6>; 
using inA3Pin = Pin<Port<C>, 3>; 
using inB3Pin = Pin<Port<C>, 2>; 
using inA4Pin = Pin<Port<A>, 6>; 
using inB4Pin = Pin<Port<A>, 7>; 

using dgbPin = Pin<Port<C>, 0>; // tx 
using ibusPin = Pin<Port<C>, 1>; // rx 

using csf1Pin = Pin<Port<F>, 2>; // ADC 12
using csf2Pin = Pin<Port<D>, 7>; // ADC 7
using csf3Pin = Pin<Port<D>, 5>; // ADC 5
using csf4Pin = Pin<Port<D>, 0>; // ADC 0 

#ifdef USE_SPORT
using rxtxPin = Pin<Port<F>, 0>; // rxtx 
#else
using sensorUartPin = Pin<Port<F>, 0>; // tx 
#endif

using daisyChain= Pin<Port<F>, 4>; 

#ifdef USE_DAISY
struct IBusThrough {
    inline static void init() {
        daisyChain::template dir<Output>();
    }
    inline static void on() {
        daisyChain::on();
    }
    inline static void off() {
        daisyChain::off();
    }
};
using ibt = IBusThrough;
#else
using ibt = void;
#endif

template<typename T>
struct PseudoTimer {
    inline static constexpr auto intervall = T::intervall * 4;
};
using pseudoTimer = PseudoTimer<systemTimer>;

using eeprom = EEProm::Controller<Storage::ApplData<pseudoTimer, servo_pa::channel_t, IBus::Switch::Protocol1::addr_t, pwm::value_type>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<12, 7, 5, 0, 0x1e>>; // 1e = temp

using ch0 = ChannelFsm<pseudoTimer, 0, pwm, adcController, inA1Pin, inB1Pin, end1, servo_pa, eeprom, terminal>;
using ch1 = ChannelFsm<pseudoTimer, 1, pwm, adcController, inA2Pin, inB2Pin, end2, servo_pa, eeprom, terminal>;
using ch2 = ChannelFsm<pseudoTimer, 2, pwm, adcController, inA3Pin, inB3Pin, end3, servo_pa, eeprom, terminal>;
using ch3 = ChannelFsm<pseudoTimer, 3, pwm, adcController, inA4Pin, inB4Pin, end4, servo_pa, eeprom, terminal>;

#ifdef USE_SPORT
using vn1 = External::AnalogConverter<ch0, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<10,1>>;
using vn2 = External::AnalogConverter<ch1, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<10,1>>;
using vn3 = External::AnalogConverter<ch2, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<10,1>>;
using vn4 = External::AnalogConverter<ch3, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<10,1>>;
#endif
#ifdef USE_IBUS
using vn1 = External::AnalogConverter<ch0, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;
using vn2 = External::AnalogConverter<ch1, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;
using vn3 = External::AnalogConverter<ch2, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;
using vn4 = External::AnalogConverter<ch3, std::ratio<0,1>, std::ratio<Ri,1900>, std::ratio<100,1>>;
#endif

using cp1 = CProvider<vn1>;
using cp2 = CProvider<vn2>;
using cp3 = CProvider<vn3>;
using cp4 = CProvider<vn4>;

using sp1 = ch0::StateProvider;
using sp2 = ch1::StateProvider;
using sp3 = ch2::StateProvider;
using sp4 = ch3::StateProvider;

#ifdef USE_SPORT
template<typename PA>
using sensorUsart = External::SoftSerial::Usart<Meta::List<rxtxPin, rxtxPin>, Component::Tcb<0>, 
PA, AVR::BaudRate<57600>,
AVR::ReceiveQueueLength<0>,
AVR::SendQueueLength<64>,
etl::NamedFlag<true>,
etl::NamedFlag<false>
//                                            ,dbg1
>;
using sensor = External::SPort::Sensor<External::SPort::SensorId::ID3, sensorUsart, systemTimer, 
Meta::List<cp1, cp2, cp3, cp4, sp1, sp2, sp3, sp4>>;

#else
using sensor = IBus::Sensor<usart2Position, AVR::Usart, AVR::BaudRate<115200>, 
Meta::List<cp1, cp2, cp3, cp4, sp1, sp2, sp3, sp4>, systemTimer, ibt
//                          , etl::NamedFlag<true>
//                           , etl::NamedFlag<true>
>;
#endif

using gswitch = IBus::Switch::GeneralSwitch<servo_pa, sensor, Meta::List<ch0, ch1, ch2, ch3>, eeprom>;

using led = AVR::ActiveHigh<ledPin, Output>;
#ifdef USE_IBUS
using gfsm = GlobalFsm<systemTimer, pwm, eeprom, Meta::List<ch0, ch1, ch2, ch3>, 
led, adcController, servo, BaudRate<115200>, sensor, gswitch, terminal>;
#endif
#ifdef USE_SBUS
using gfsm = GlobalFsm<systemTimer, pwm, eeprom, Meta::List<ch0, ch1, ch2, ch3>, 
led, adcController, servo, BaudRate<100000>, sensor, gswitch, terminal>;
#endif
#ifdef USE_SPORT
using isrRegistrar = IsrRegistrar<sensor::uart::StartBitHandler, sensor::uart::BitHandler>;
#endif

int main() {
    portmux::init();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();
    gfsm::init();
    
    dbgPin::dir<Output>();
    
    {
#ifdef USE_SPORT
        etl::Scoped<etl::EnableInterrupt<>> ei;
#endif
        while(true) {
            gfsm::periodic();
            systemTimer::periodic([&]{
                dbgPin::on();
                gfsm::ratePeriodic();
                dbgPin::off();
                alarmTimer::periodic([&](const auto&){
                });
            });
        }
    }
}

#ifdef USE_SPORT
ISR(PORTF_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<rxtxPin::name_type>>();
}

ISR(TCB0_INT_vect) {
    isrRegistrar::isr<AVR::ISR::Tcb<0>::Capture>();
}
#endif

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
        led::activate();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
