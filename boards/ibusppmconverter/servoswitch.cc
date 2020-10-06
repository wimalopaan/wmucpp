//#define NDEBUG -> board.h

//#define DEBUG2 // RX/TX change -> full duplex

#define USE_SBUS
//#define USE_IBUS

#define LEARN_DOWN // start at highest channel number downwards

#include "board.h"

namespace Storage2 {
    inline static constexpr uint8_t NChannels = 5;

    enum class FollowMode : uint8_t {Off, CopyPositions, OwnPositions};
    
    template<typename ModeType, typename PPM>
    struct ChannelConfig {
        using ch_index_t = etl::uint_ranged<uint8_t, 0, NChannels - 1>;
        using fo_index_t = etl::uint_ranged_NaN<uint8_t, 0, NChannels - 1>;
        
        using ppm_value_t = typename PPM::ranged_type;
        
        constexpr ChannelConfig() {
            std::fill(std::begin(mPositions), std::end(mPositions), PPM::ocMedium);
        } 
        void addFollower(const IBus::Switch::Protocol1::index_t& i) {
            for(const auto& v : mFollower) {
                if (v && (v.toInt() == i)) {
                    return;
                }
            }
            for(auto& v: mFollower) {
                if (i && !v) {
                    v = i.toInt();
                    break;
                }
            }
        }
        void removeFollower(const IBus::Switch::Protocol1::index_t& i) {
            for(auto& v: mFollower) {
                if (v && i && (v.toInt() == i.toInt())) {
                    v.setNaN();
                }
            }
        }
        template<typename Term>
        void print() const {
            etl::out<Term>("F["_pgm);
            for(const auto& f: mFollower) {
                etl::out<Term>(" "_pgm, f.toInt());
            }            
            etl::outl<Term>("]"_pgm);
        }
        FollowMode followMode() const {
            return mFollow;
        }
        void followMode(const FollowMode m){
            mFollow = m;
        }
        bool isFollowMode(const FollowMode& m) const {
            return mFollow == m;
        }
        void forFollowers(const auto& func) {
            for (const auto& follower : mFollower) {
                if (follower) {
                    func(follower);
                }
            }
        }
        const auto& positions() const {
            return mPositions;
        }
        auto& position(const ModeType& m) {
            return mPositions[m];
        }
        fo_index_t followMaster() const {
            return mFollowIndex;
        }
        void followMaster(const fo_index_t& i) {
            mFollowIndex = i;
        }
        void removeMaster() {
            mFollowIndex.setNaN();
        }
        using increment_t = etl::uint_ranged<uint8_t, 1, 100>;
        void increment(const increment_t& i) {
            mIncrement = i;
        }
        increment_t increment() const{
            return mIncrement;
        }
    private:
        std::array<ppm_value_t, ModeType::Upper + 1> mPositions;        
        increment_t mIncrement{50};
        FollowMode mFollow{FollowMode::Off};
        fo_index_t mFollowIndex;
        std::array<fo_index_t, NChannels> mFollower;
    };
    
    template<typename Channel, typename AddressR, typename ModeType, typename PPM>
    struct ApplData final : public EEProm::DataBase<ApplData<Channel, AddressR, ModeType, PPM>> {
//                Channel::_;
        using Address = etl::uint_ranged_NaN<typename AddressR::value_type, AddressR::Lower, AddressR::Upper>;
//        Address::_;
        using mode_t = ModeType;
        using value_type = ChannelConfig<ModeType, PPM>;
        using fo_index_t = typename value_type::fo_index_t;
        using ch_index_t = typename value_type::ch_index_t;
        using increment_t = typename value_type::increment_t;
        using follow_t = FollowMode;
        
        constexpr uint8_t& magic() {
            return mMagic;
        }
        constexpr void clear() {
            for(auto& v : AValues) {
                v = value_type{};
            }
        }
        constexpr Channel& channel() {
            return mChannel;
        }
        constexpr Address& address() {
            return mAddress;
        }
        constexpr void addFollower(const ch_index_t& i, const fo_index_t& follower) {
            AValues[i].addFollower(follower);
        }
        constexpr void removeFollower(const ch_index_t& i, const fo_index_t& follower) {
            AValues[i].removeFollower(follower);
        }
        constexpr const auto& positions(const ch_index_t& i) const {
            return AValues[i].positions();
        }
        constexpr increment_t increment(const ch_index_t i) const {
            return AValues[i].increment();
        }
        constexpr value_type& channel(const etl::uint_ranged<uint8_t, 0, NChannels - 1>& ch) {
            return AValues[ch];
        }
        template<typename Term>
        constexpr void print() {
            for(const auto& ch : AValues) {
                ch.template print<Term>();
            }
        }
    private:
        uint8_t mMagic;
        Channel mChannel;
        Address mAddress;
        std::array<value_type, NChannels> AValues;
    };
}

template<typename PA, typename SW, typename NVM, typename Timer, typename FsmList, typename RELD, typename LED = void, typename Term = void>
struct GFSM;

template<typename PA, typename SW, typename NVM, typename Timer, typename... Fsms, typename RELD, typename LED, typename Term>
struct GFSM<PA, SW, NVM, Timer, Meta::List<Fsms...>, RELD, LED, Term> {
    using ch_t = PA::channel_t;
    
    enum class State : uint8_t {Undefined, StartWait, SearchChannel, AfterSearch, InitRun, Run, 
                                ShowAddress, ShowAddressWait, LearnTimeout,
//                                Test1
                               };
    
    static constexpr auto intervall = Timer::intervall;
    
    inline static constexpr auto learnTimeout = 4000_ms;
    inline static constexpr auto scanTimeout = 50_ms;
    
    static_assert(learnTimeout > (scanTimeout * (ch_t::Upper + 1) * 2), "wrong learn timeout");
    
    static constexpr External::Tick<Timer> learnTimeoutTicks{learnTimeout};
    static constexpr External::Tick<Timer> scanTimeoutTicks{scanTimeout};
    static constexpr External::Tick<Timer> waitTimeoutTicks{3000_ms};
    static constexpr External::Tick<Timer> reloadTimeoutTicks{20_ms};
    static constexpr External::Tick<Timer> signalTimeoutTicks{500_ms};

    using blinker = External::SimpleBlinker<LED, Timer, 300_ms>;
    
    static inline void init() {
        blinker::init();
    }

    static inline void periodic() {
        switch(mState) {
        case State::Run:
            (Fsms::periodic(), ...);
            break;
        case State::Undefined:
        case State::StartWait:
        case State::SearchChannel:
        case State::AfterSearch:
        case State::InitRun:
        case State::ShowAddress:
        case State::ShowAddressWait:
        case State::LearnTimeout:
//        case State::Test1:
        default:
            break;
        }
    }
    
    static inline void ratePeriodic() {
        const auto oldState = mState;
        blinker::ratePeriodic();
        ++stateTicks;
        switch(mState) {
        case State::Undefined:
            mState = State::StartWait;
            blinker::steady();
            break;
        case State::StartWait:
            stateTicks.on(waitTimeoutTicks, []{
                blinker::off();
                mState = State::SearchChannel;
            });
            break;
        case State::SearchChannel:
            if (search()) {
                mState = State::AfterSearch;
            }
            stateTicks.on(learnTimeoutTicks, []{
                mState = State::LearnTimeout;
            });
            break;
        case State::AfterSearch:
            stateTicks.on(signalTimeoutTicks, []{
                mState = State::ShowAddress;
            });
            break;
        case State::LearnTimeout:
            etl::outl<Term>("timeout ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            if (NVM::data().channel() && NVM::data().address()) {
                SW::channel(NVM::data().channel());
                SW::address(typename SW::addr_t{NVM::data().address().toInt()});
            }
            else {
                etl::outl<Term>("using ch: "_pgm, SW::channel().toInt(), " adr: "_pgm, SW::address().toInt());
            }
            mState = State::InitRun;
            break;
        case State::ShowAddress:
            etl::outl<Term>("learned ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            blinker::blink(NVM::data().address().toInt() + 1);
            mState = State::ShowAddressWait;
            break;
        case State::ShowAddressWait:
            if (!blinker::isActive()) {
                mState = State::InitRun;
            }
            break;
        case State::Run:
            SW::ratePeriodic();
            (Fsms::ratePeriodic(), ...);
            stateTicks.on(reloadTimeoutTicks, []{
                RELD::reload();
            });
            if (SW::receivedControl()) {
                blinker::steady();                
            }
            else {
                blinker::off();                
            }
            break;
        case State::InitRun:
            (Fsms::init(), ...);
            mState = State::Run;
            break;
//        case State::Test1:
//            if (!SW::receivedControl()) {
//                mState = State::Run;
//            }
//            break;
        }
        if (oldState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Run:
                break;
            case State::Undefined:
            case State::StartWait:
                etl::outl<Term>("swait"_pgm);
                break;
            case State::SearchChannel:
                etl::outl<Term>("search"_pgm);
                break;
            case State::AfterSearch:
            case State::InitRun:
            case State::ShowAddress:
            case State::ShowAddressWait:
            case State::LearnTimeout:
//            case State::Test1:
            default:
                break;
            }
        }
    }
private:
    using protocol_t = typename SW::protocol_t;
    using addr_t = typename protocol_t::addr_t;
    
    static inline bool search() {
        if (const auto lc = PA::valueMapped(learnChannel.toRangedNaN()); lc && SW::isLearnCode(lc)) {
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
    static inline State mState{State::Undefined};
    inline static External::Tick<Timer> stateTicks;
};

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using ppmA = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>>;

using tcb0Position = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using ppmB = External::Ppm::PpmOut<tcb0Position>;

using tcb1Position = Portmux::Position<Component::Tcb<1>, Portmux::Default>;
using ppmC = External::Ppm::PpmOut<tcb1Position>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcb0Position, tcb1Position>>;

#ifdef USE_SBUS
using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
#endif
#ifdef USE_IBUS
using servo_pa = IBus::Servo::ProtocollAdapter<0>;
#endif

#ifndef DEBUG2
using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;
#else
using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<512>>;
#endif

template<typename PPM, uint8_t Channel = 0>
struct Adapter {
    using ppm_t = PPM;
    using ppm_index_t = PPM::index_type;
    
    inline static constexpr auto ocMax =  PPM::ocMax;
    inline static constexpr auto ocMin =  PPM::ocMin;
    inline static constexpr auto ocMedium =  PPM::ocMedium;
    
    template<bool start = true>
    inline static void init() {
        PPM::template init<start>();
    }
    using ranged_type = PPM::ranged_type;    
    
//    ranged_type::_;
    
    inline static void ppmRaw(const auto v) {
        PPM::ppmRaw(ppm_index_t{Channel}, v);
    }
    
    inline static void ppm_async(const auto v) {
        PPM::ppm(ppm_index_t{Channel}, v);
    }
};

template<typename PPM, typename PA, typename NVM, typename Timer, etl::uint_ranged<uint8_t, 0, Storage2::NChannels - 1> Channel>
struct FSM {
    inline static constexpr auto& appData = NVM::data();
    inline static constexpr auto& chData = appData.channel(Channel);
    
    using storage_t = std::remove_cvref_t<decltype(appData)>; 
    
    using mode_t = storage_t::mode_t;
    using ch_config_t = storage_t::value_type;
    using ch_index_t = storage_t::ch_index_t;
    using fo_index_t = storage_t::fo_index_t;
    using increment_t = storage_t::increment_t;
    using ppm_value_t = PPM::ranged_type;
    
    enum class State : uint8_t {Init, Steady, Run, Test};
    
    
    static inline void reset() {
        chData = ch_config_t{};
        jump(chData.positions()[0]);
    }
    
//    template<bool start = true>
    static inline void init() {
//        PPM::template init<start>();
        mState = State::Init;
    }

    static inline void setTest(const mode_t&) {
    }

    static constexpr External::Tick<Timer> waitTimeoutTicks{20_ms};
//    std::integral_constant<uint16_t, waitTimeoutTicks.value.toInt()>::_;
    
    static inline void ratePeriodic() {
        ++stateTicks;
        const auto oldState = mState;
        switch(mState) {
        case State::Init:
            if (chData.isFollowMode(Storage2::FollowMode::CopyPositions) && chData.followMaster()) {
                mActualPosition = mTargetPosition = appData.positions(ch_index_t{chData.followMaster().toInt()})[0];
            }
            else {
                mActualPosition = mTargetPosition = chData.positions()[0];
            }            
            PPM::ppmRaw(mActualPosition);
            PPM::init();
            mState = State::Steady;
            break;
        case State::Steady:
            if (mTargetPosition != mActualPosition) {
                mState = State::Run;
            }
            break;
        case State::Run:
            if (mActualPosition == mTargetPosition) {
                mState = State::Steady;
            }
            stateTicks.on(waitTimeoutTicks, []{
                if (mTargetPosition > (mActualPosition + increment)) {
                    mActualPosition += increment;
                }
                else if (mTargetPosition < (mActualPosition - increment)) {
                    mActualPosition -= increment;
                }
                else {
                    mActualPosition = mTargetPosition;
                }
                PPM::ppmRaw(mActualPosition);
            });
            break;
        case State::Test:
            break;
        }
        if (oldState != mState) {
            stateTicks.reset();
        }
    }
    static inline void periodic() {
    }
    static inline void setIncrement(const auto& increment) {
//        decltype(increment)::_;
        const auto i = etl::scaleTo<increment_t>(increment);
        chData.increment(i);
        appData.change();
    }
    template<typename T, auto L, auto U>
    static inline void setPosition(const mode_t& p, const etl::uint_ranged<T, L, U>& v) {
        ppm_value_t pv = etl::scaleTo<ppm_value_t>(v);          
        chData.position(p) = pv;
        appData.change();
        jump(pv);
    }
    template<typename T, auto L, auto U>
    static inline void setPosition(const mode_t& p, const etl::uint_ranged_NaN<T, L, U>& v) {
        if (v) {
            ppm_value_t pv = etl::scaleTo<ppm_value_t>(v.toRanged());          
            chData.position(p) = pv;
            appData.change();
            jump(pv);
        }
    }

    static inline void moveToPosition(const mode_t& p) {
        mMode = p;
        if (chData.isFollowMode(Storage2::FollowMode::CopyPositions) && chData.followMaster()) {
            increment = appData.increment(ch_index_t{chData.followMaster().toInt()});
            mTargetPosition = appData.positions(ch_index_t{chData.followMaster().toInt()})[p];
        }
        else {
            increment = chData.increment();
            mTargetPosition = chData.positions()[p];
        }
    }
//private:
    static inline void jump(const ppm_value_t& p) {
        PPM::ppmRaw(p);
        mActualPosition = mTargetPosition = p;        
    } 
    inline static State mState{State::Init};
    inline static External::Tick<Timer> stateTicks;
    inline static ppm_value_t mActualPosition{PPM::ocMedium};
    inline static ppm_value_t mTargetPosition{PPM::ocMedium};
    inline static mode_t mMode;
    inline static etl::uint_ranged<uint8_t, 1, 100> increment{50};
};

using eeprom = EEProm::Controller<Storage2::ApplData<servo_pa::channel_t, IBus::Switch::Protocol1::addr_t, IBus::Switch::Protocol1::mode_t, ppmA>>;

using ppmCh1 = Adapter<ppmA, 0>;
using ppmCh2 = Adapter<ppmA, 1>;
using ppmCh3 = Adapter<ppmA, 2>;
using fsm1 = FSM<ppmCh1, servo_pa, eeprom, systemTimer, eeprom::data_t::ch_index_t{0}>;
using fsm2 = FSM<ppmCh2, servo_pa, eeprom, systemTimer, eeprom::data_t::ch_index_t{1}>;
using fsm3 = FSM<ppmCh3, servo_pa, eeprom, systemTimer, eeprom::data_t::ch_index_t{2}>;
using fsm4 = FSM<ppmB, servo_pa, eeprom, systemTimer, eeprom::data_t::ch_index_t{3}>;
using fsm5 = FSM<ppmC, servo_pa, eeprom, systemTimer, eeprom::data_t::ch_index_t{4}>;

using ibus_switch = IBus::Switch::ServoSwitch<servo_pa, eeprom, Meta::List<fsm1, fsm2, fsm3, fsm4, fsm5>>;

using evch0 = Event::Channel<0, void>;
using evch1 = Event::Channel<1, void>;
using evuser0 = Event::Route<evch0, Event::Users::Tcb<0>>;
using evuser1 = Event::Route<evch1, Event::Users::Tcb<1>>;
using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1>>;

struct Reloader {
    static inline void reload() {
        ppmB::onReload([]{
            evrouter::strobe<0>();
        });
        ppmC::onReload([]{
            evrouter::strobe<1>();
        });
    }
};
using reloader = Reloader;


#ifndef DEBUG2
using led = AVR::ActiveLow<daisyChain, Output>;
using terminal = etl::basic_ostream<void>;
#else
using led = AVR::ActiveHigh<q0Pin, Output>;
using terminal = etl::basic_ostream<servo>;
#endif

using gfsm = GFSM<servo_pa, ibus_switch, eeprom, systemTimer, Meta::List<fsm1, fsm2, fsm3, fsm4, fsm5>, reloader, led, terminal>;

auto& appData = eeprom::data();

int main() {
    wdt::init<ccp>();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    reset::noWatchDog([]{
        uninitialzed::reset();
    });
    
    reset::onWatchDog([]{
        uninitialzed::counter = uninitialzed::counter + 1;        
    });

    eeprom::init();

    if (appData.magic() != 42) {
        appData.clear();
        appData.magic() = 42;
        appData.change();
    }

    evrouter::init();
    portmux::init();
    systemTimer::init();
    
#ifdef USE_IBUS
# ifndef DEBUG2
    servo::init<AVR::BaudRate<115200>, HalfDuplex>();
    servo::txEnable<false>();
# else
    servo::init<AVR::BaudRate<115200>>();
# endif
#endif
#ifdef USE_SBUS
    servo::init<AVR::BaudRate<100000>, HalfDuplex, true, 1>(); // 8E2
    servo::txEnable<false>();
#endif
    
    gfsm::init();
    
    ibus_switch::init();
    
    const auto eepromTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
    while(true) {
        eeprom::saveIfNeeded([&]{});
        servo::periodic();
        gfsm::periodic();
        
        systemTimer::periodic([&]{
            wdt::reset();
            gfsm::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (eepromTimer == t) {
                    appData.print<terminal>();
                    appData.expire();
                }
            });
        });
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
#if defined(DEBUG2)
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
    while(true) {
        daisyChain::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
