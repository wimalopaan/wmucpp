#pragma once

#include <mcu/internals/usart.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>

#include <external/solutions/blinker.h>
#include <external/solutions/series01/sppm_in.h>

namespace External {
    using namespace AVR;
    using namespace std::literals::chrono;
    using namespace External::Units::literals;
    
    struct TermSameAsScanDevice {};
    
    namespace Bus {
        static inline constexpr auto fRtc = 1000_Hz;
        
        template<typename Devs>
        struct SBusSPort {
            using devs = Devs;
//            using servo_pa = External::SBus::Servo::ProtocollAdapter<0, typename Devs::systemTimer>;
//            using periodic_dev = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
//            using terminal_device = periodic_dev;
        };
        template<typename Devs>
        struct IBusIBus {
            using devs = Devs;
//            using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
//            using periodic_dev = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
//            using terminal_device = periodic_dev;
        };
        template<typename Devs>
        struct SumDHott {
            using devs = Devs;
//            using periodic_dev = Hott::Experimental::Sensor<typename Devs::servoPosition, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, typename Devs::systemTimer>;
//            using servo_pa = periodic_dev::ProtocollAdapter;
//            using terminal_device = void;
        };
        template<typename Devs>
        struct Ppm {
            using devs = Devs;
//            using periodic_dev = Hott::Experimental::Sensor<typename Devs::servoPosition, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, typename Devs::systemTimer>;
//            using servo_pa = periodic_dev::ProtocollAdapter;
//            using terminal_device = void;
        };
        template<typename Devs>
        struct NoBus {
            using devs = Devs;
        };

        
        template<typename> struct isSBus : std::false_type {};
        template<typename D> struct isSBus<SBusSPort<D>> : std::true_type {};

        template<typename> struct isIBus : std::false_type {};
        template<typename D> struct isIBus<IBusIBus<D>> : std::true_type {};

        template<typename> struct isSumD : std::false_type {};
        template<typename D> struct isSumD<SumDHott<D>> : std::true_type {};

        template<typename> struct isPpm : std::false_type {};
        template<typename D> struct isPpm<Ppm<D>> : std::true_type {};

        template<typename> struct isNoBus: std::false_type {};
        template<typename D> struct isNoBus<NoBus<D>> : std::true_type {};
    }
    
    template<typename Devs, template<typename> typename App, typename Duplex = AVR::FullDuplex>
    struct Scanner {
        enum class State : uint8_t {Undefined, Init, 
                                    InitSBus, InitSBusInv, InitIBus, InitSumD, InitPpm, 
                                    CheckSBus, CheckSBusInv, CheckIBus, CheckSumD, CheckPpm,
                                    IsSBus, IsSBusInv, IsIBus, IsSumD, IsPpm};
         
        static_assert(std::is_same_v<Duplex, AVR::FullDuplex> || std::is_same_v<Duplex, AVR::HalfDuplex>);
        using devs = Devs;
        using systemTimer = devs::systemTimer;
    
        using ibus_pa = IBus2::Servo::ProtocollAdapter<0>;
        using ibus_test_dev = Usart<typename Devs::scanDevPosition, ibus_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;
        
        using sbus_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
        using sbus_test_dev = Usart<typename Devs::scanDevPosition, sbus_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;
        
        using sumd_pa = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
        using sumd_test_dev = Usart<typename Devs::scanDevPosition, sumd_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;

        using ppm_test_dev = External::Ppm::SinglePpmIn<typename Devs::ppmDevPosition>;
        using evrouter = Devs::evrouter;
        
        using term_dev = devs::scan_term_dev;
        using terminal = etl::basic_ostream<term_dev>;

        using led = std::conditional_t<std::is_same_v<typename Devs::scanLedPin, void>, AVR::ActiveHigh<AVR::NoPin, Output>, typename Devs::scanLedPin>;
        using blinker = External::Blinker2<led, systemTimer, 100_ms, 2000_ms>;
        
        static constexpr External::Tick<systemTimer> initTimeout{100_ms};
        static constexpr External::Tick<systemTimer> checkTimeout{1000_ms};
        
        static constexpr uint8_t minPackages{10};
        
        [[noreturn]] static inline auto run() {
            devs::init();
            blinker::init();
            while(true) {
                periodic();            
                systemTimer::periodic([]{
                    ratePeriodic();
                });
            }
        }   
    private:
        inline static void periodic() {
            devs::periodic();
            if constexpr(!std::is_same_v<term_dev, void>) {
                term_dev::periodic();
            }
            switch(mState) {
            case State::InitIBus:
            case State::CheckIBus:
                ibus_test_dev::periodic();
                break;
            case State::InitSBus:
            case State::InitSBusInv:
            case State::CheckSBus:
            case State::CheckSBusInv:
                sbus_test_dev::periodic();
                break;
            case State::InitSumD:
            case State::CheckSumD:
                sumd_test_dev::periodic();
                break;
            case State::InitPpm:
            case State::CheckPpm:
                if constexpr(!std::is_same_v<typename Devs::ppmDevPosition, void>) {
                    ppm_test_dev::periodic();
                }
                break;
            case State::Init:
            case State::IsIBus:
            case State::IsSBus:
            case State::IsSBusInv:
            case State::IsSumD:
            case State::IsPpm:
            case State::Undefined:
                break;
#ifdef NDEBUG
            default:
                __builtin_unreachable();
                break;
#endif     
            }                    
        }
        inline static void ratePeriodic() {
            blinker::ratePeriodic();
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Undefined:
                mState = State::Init;
                break;
            case State::Init:
                mStateTick.on(initTimeout, []{
                    mState = State::InitIBus;
                });
                break;
            case State::InitIBus:
                ibus_pa::ratePeriodic();
                mStateTick.on(checkTimeout, []{
                    mState = State::CheckIBus;
                });
                break;
            case State::InitSBus:
                sbus_pa::ratePeriodic();
                mStateTick.on(checkTimeout, []{
                    mState = State::CheckSBus;
                });
                break;
            case State::InitSBusInv:
                sbus_pa::ratePeriodic();
                mStateTick.on(checkTimeout, []{
                    mState = State::CheckSBusInv;
                });
                break;
            case State::InitSumD:
                sumd_pa::ratePeriodic();
                mStateTick.on(checkTimeout, []{
                    mState = State::CheckSumD;
                });
                break;
            case State::InitPpm:
                if constexpr(!std::is_same_v<typename Devs::ppmDevPosition, void>) {
                    mStateTick.on(checkTimeout, []{
                        mState = State::CheckPpm;
                    });                    
                }
                else {
                    mState = State::CheckPpm;
                }
                break;
            case State::CheckIBus:
                ibus_pa::ratePeriodic();
                if (ibus_pa::packages() >= minPackages) {
                    mState = State::IsIBus;
                }
                else {
                    mState = State::InitSBusInv;
                }
                break;
            case State::CheckSBus:
                sbus_pa::ratePeriodic();
                if (sbus_pa::packages() >= minPackages) {
                    mState = State::IsSBus;
                }
                else {
                    mState = State::InitSumD;
                }
                break;
            case State::CheckSBusInv:
                sbus_pa::ratePeriodic();
                if (sbus_pa::packages() >= minPackages) {
                    mState = State::IsSBusInv;
                }
                else {
                    mState = State::InitSBus;
                }
                break;
            case State::CheckSumD:
                sumd_pa::ratePeriodic();
                if (sumd_pa::packages() >= minPackages) {
                    mState = State::IsSumD;
                }
                else {
                    mState = State::InitPpm;
                }
                break;
            case State::CheckPpm:
                if constexpr(!std::is_same_v<typename Devs::ppmDevPosition, void>) {
                    if (ppm_test_dev::counter() > 10) {
                        mState = State::IsPpm;
                    }
                    else {
                        mState = State::InitIBus;
                    }
                }
                else {
                    mState = State::InitIBus;
                }
                break;
            case State::IsSBus:
                blinker::off();
                App<Bus::SBusSPort<Devs>>::run(false);
                mState = State::Init;
                break;
            case State::IsSBusInv:
                blinker::off();
                App<Bus::SBusSPort<Devs>>::run(true);
                mState = State::Init;
                break;
            case State::IsIBus:
                blinker::off();
                App<Bus::IBusIBus<Devs>>::run();
                mState = State::Init;
                break;
            case State::IsSumD:
                blinker::off();
                App<Bus::SumDHott<Devs>>::run();
                mState = State::Init;
                break;
            case State::IsPpm:
                blinker::off();
                App<Bus::Ppm<Devs>>::run();
                mState = State::Init;
                break;
#ifdef NDEBUG
            default:
                __builtin_unreachable();
                break;
#endif     
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Undefined:
                    break;
                case State::Init:
                    blinker::steady();
                    if constexpr(!std::is_same_v<term_dev, void>) {
                        term_dev::template init<AVR::BaudRate<115200>>();
                    }
                    etl::outl<terminal>("s: i "_pgm, GITTAG_PGM);
                    break;
                case State::InitIBus:
                    ibus_test_dev::template init<AVR::BaudRate<115200>, Duplex>();
                    ibus_test_dev::txPinDisable();
                    if constexpr(std::is_same_v<Duplex, AVR::FullDuplex>) {
                        ibus_test_dev::rxInvert(false);
                    }
                    else {
                        ibus_test_dev::txInvert(false);
                    }
                    ibus_pa::resetStats();
                    etl::outl<terminal>("s: iib"_pgm);
                    break;
                case State::InitSBus:
                    sbus_test_dev::template init<AVR::BaudRate<100000>, Duplex, true, 1>();
                    sbus_test_dev::txPinDisable();
                    if constexpr(std::is_same_v<Duplex, AVR::FullDuplex>) {
                        sbus_test_dev::rxInvert(false);
                    }
                    else {
                        sbus_test_dev::txInvert(false);
                    }
                    sbus_pa::resetStats();
                    etl::outl<terminal>("s: isb"_pgm);
                    break;
                case State::InitSBusInv:
                    sbus_test_dev::template init<AVR::BaudRate<100000>, Duplex, true, 1>();
                    sbus_test_dev::txPinDisable();
                    if constexpr(std::is_same_v<Duplex, AVR::FullDuplex>) {
                        sbus_test_dev::rxInvert(true);
                    }
                    else {
                        sbus_test_dev::txInvert(true);
                    }
                    sbus_pa::resetStats();
                    etl::outl<terminal>("s: isbi"_pgm);
                    break;
                case State::InitSumD:
                    sumd_test_dev::template init<AVR::BaudRate<115200>, Duplex>();
                    sumd_test_dev::txPinDisable();
                    if constexpr(std::is_same_v<Duplex, AVR::FullDuplex>) {
                        sumd_test_dev::rxInvert(false);
                    }
                    else {
                        sumd_test_dev::txInvert(false);
                    }
                    sumd_pa::resetStats();
                    etl::outl<terminal>("s: isd"_pgm);
                    break;
                case State::InitPpm:
                    if constexpr(!std::is_same_v<typename Devs::ppmDevPosition, void>) {
                        evrouter::init();
                        ppm_test_dev::init();
                        ppm_test_dev::reset();
                    }
                    etl::outl<terminal>("s: ippm"_pgm);
                    break;
                case State::CheckIBus:
                    etl::outl<terminal>("s: cib"_pgm);
                    break;
                case State::CheckSBus:
                    etl::outl<terminal>("s: csb"_pgm);
                    break;
                case State::CheckSBusInv:
                    etl::outl<terminal>("s: csbi"_pgm);
                    break;
                case State::CheckSumD:
                    etl::outl<terminal>("s: csd"_pgm);
                    break;
                case State::CheckPpm:
                    etl::outl<terminal>("s: cppm"_pgm);
                    break;
                case State::IsIBus:
                    etl::outl<terminal>("s: IB"_pgm);
                    break;
                case State::IsSBus:
                    etl::outl<terminal>("s: SB"_pgm);
                    break;
                case State::IsSBusInv:
                    etl::outl<terminal>("s: SBI"_pgm);
                    break;
                case State::IsSumD:
                    etl::outl<terminal>("s: SD"_pgm);
                    break;
                case State::IsPpm:
                    etl::outl<terminal>("s: SPpm"_pgm);
                    break;
#ifdef NDEBUG
                default:
                    __builtin_unreachable();
                    break;
#endif     
                }
            }        
        }
        static inline State mState{State::Undefined};
        static inline External::Tick<systemTimer> mStateTick;
    };

    template<typename Devs, template<typename> typename App, typename PList, typename Duplex = AVR::FullDuplex, bool useTimeout = false>
    struct Scanner2 {
        enum class State : uint8_t {Undefined, Init, 
                                    InitSBus, InitSBusInv, InitIBus, InitSumD, InitPpm, 
                                    CheckSBus, CheckSBusInv, CheckIBus, CheckSumD, CheckPpm,
                                    IsSBus, IsSBusInv, IsIBus, IsSumD, IsPpm};

        static inline constexpr bool checkPpm = Meta::contains_v<PList, Bus::Ppm<Devs>>;
        static inline constexpr bool checkIbus= Meta::contains_v<PList, Bus::IBusIBus<Devs>>;
        static inline constexpr bool checkSbus= Meta::contains_v<PList, Bus::SBusSPort<Devs>>;
        static inline constexpr bool checkSumd= Meta::contains_v<PList, Bus::SumDHott<Devs>>;
        
        static_assert(std::is_same_v<Duplex, AVR::FullDuplex> || std::is_same_v<Duplex, AVR::HalfDuplex>);
        using devs = Devs;
        using systemTimer = devs::systemTimer;
        using scanDevPosition = devs::scanDevPosition;
        
        static inline constexpr bool termOnScanDev = std::is_same_v<typename devs::scan_term_dev, TermSameAsScanDevice>;
      
        using ScanDevSendQueueLength = std::conditional_t<termOnScanDev, AVR::SendQueueLength<128>, AVR::SendQueueLength<0>>;
        
        using ibus_pa = std::conditional_t<checkIbus, IBus2::Servo::ProtocollAdapter<0>, External::Hal::NullProtocollAdapter<>>;
        using ibus_test_dev = std::conditional_t<checkIbus, 
                                                 Usart<scanDevPosition, ibus_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, ScanDevSendQueueLength>,
                                                 void>; 
        
        using sbus_pa = std::conditional_t<checkSbus, External::SBus::Servo::ProtocollAdapter<0, systemTimer>, External::Hal::NullProtocollAdapter<>>;
        using sbus_test_dev = std::conditional_t<checkSbus,
                                                 Usart<scanDevPosition, sbus_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, ScanDevSendQueueLength>,
                                                  void>;
        
        using sumd_pa = std::conditional_t<checkSumd, Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>, External::Hal::NullProtocollAdapter<>>;
        using sumd_test_dev = std::conditional_t<checkSumd,
                                                 Usart<scanDevPosition, sumd_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, ScanDevSendQueueLength>,
                                                 void>;

        using ppm_test_dev = External::Ppm::SinglePpmIn<typename Devs::ppmDevPosition>;
        using evrouter = Devs::evrouter;
        
        
        using term_dev = std::conditional_t<termOnScanDev, void, typename devs::scan_term_dev>;
        using terminal = etl::basic_ostream<term_dev>;

        using led = std::conditional_t<std::is_same_v<typename Devs::scanLedPin, void>, AVR::ActiveHigh<AVR::NoPin, Output>, typename Devs::scanLedPin>;
        using blinker = External::Blinker2<led, systemTimer, 100_ms, 2000_ms>;
        
        static constexpr External::Tick<systemTimer> initTimeout{100_ms};
        static constexpr External::Tick<systemTimer> checkTimeout{1000_ms};
        static constexpr External::Tick<systemTimer> totalTimeout{15000_ms};
        
        static constexpr uint8_t minPackages{10};
        
        [[noreturn]] static inline auto run() {
            devs::init();
            blinker::init();
            while(true) {
                periodic();            
                systemTimer::periodic([]{
                    ratePeriodic();
                });
            }
        }   
    private:
        inline static void periodic() {
            devs::periodic();
            if constexpr(!std::is_same_v<term_dev, void>) {
                term_dev::periodic();
            }
            switch(mState) {
            case State::InitIBus:
            case State::CheckIBus:
                if constexpr(!std::is_same_v<scanDevPosition, void> && checkIbus) {
                    ibus_test_dev::periodic();
                }
                break;
            case State::InitSBus:
            case State::InitSBusInv:
            case State::CheckSBus:
            case State::CheckSBusInv:
                if constexpr(!std::is_same_v<scanDevPosition, void> && checkSbus) {
                    sbus_test_dev::periodic();
                }
                break;
            case State::InitSumD:
            case State::CheckSumD:
                if constexpr(!std::is_same_v<scanDevPosition, void> && checkSumd) {
                    sumd_test_dev::periodic();
                }
                break;
            case State::InitPpm:
            case State::CheckPpm:
                if constexpr(!std::is_same_v<typename Devs::ppmDevPosition, void> && checkPpm) {
                    ppm_test_dev::periodic();
                }
                break;
            case State::Init:
            case State::IsIBus:
            case State::IsSBus:
            case State::IsSBusInv:
            case State::IsSumD:
            case State::IsPpm:
            case State::Undefined:
                break;
#ifdef NDEBUG
            default:
                __builtin_unreachable();
                break;
#endif     
            }                    
        }
        inline static void ratePeriodic() {
            blinker::ratePeriodic();
            const auto oldState = mState;
            ++mStateTick;

            if constexpr(useTimeout) {
                (++mTotalTick).on(totalTimeout, []{
                    blinker::template off<true>();
                    App<Bus::SBusSPort<Devs>>::timeout();
                });
            }            
            switch(mState) {
            case State::Undefined:
                mState = State::Init;
                break;
            case State::Init:
                mStateTick.on(initTimeout, []{
                    mState = State::InitIBus;
                });
                break;
            case State::InitIBus:
                if constexpr(!checkIbus) {
                    mState = State::InitSBus;
                }
                else {
                    ibus_pa::ratePeriodic();
                    mStateTick.on(checkTimeout, []{
                        mState = State::CheckIBus;
                    });
                }
                break;
            case State::InitSBus:
                if constexpr(!checkSbus) {
                    mState = State::InitSumD;
                }
                else {
                    sbus_pa::ratePeriodic();
                    mStateTick.on(checkTimeout, []{
                        mState = State::CheckSBus;
                    });
                }
                break;
            case State::InitSBusInv:
                if constexpr(!checkSbus) {
                    mState = State::InitSumD;
                }
                else {
                    sbus_pa::ratePeriodic();
                    mStateTick.on(checkTimeout, []{
                        mState = State::CheckSBusInv;
                    });
                }
                break;
            case State::InitSumD:
                if constexpr(!checkSumd) {
                    mState = State::InitPpm;
                }
                else {
                    sumd_pa::ratePeriodic();
                    mStateTick.on(checkTimeout, []{
                        mState = State::CheckSumD;
                    });
                }
                break;
            case State::InitPpm:
                if constexpr(!checkPpm) {
                    mState = State::InitIBus;
                }
                else {
                mStateTick.on(checkTimeout, []{
                    mState = State::CheckPpm;
                });                    
                }
                break;
            case State::CheckIBus:
                if constexpr(checkIbus) {
                    ibus_pa::ratePeriodic();
                    if (ibus_pa::packages() >= minPackages) {
                        mState = State::IsIBus;
                    }
                    else {
                        mState = State::InitSBusInv;
                    }
                }
                else {
                    mState = State::InitSBusInv;
                }
                break;
            case State::CheckSBus:
                if constexpr(checkSbus) {
                    sbus_pa::ratePeriodic();
                    if (sbus_pa::packages() >= minPackages) {
                        mState = State::IsSBus;
                    }
                    else {
                        mState = State::InitSumD;
                    }
                }
                else {
                    mState = State::InitSumD;
                }
                break;
            case State::CheckSBusInv:
                if constexpr(checkSbus) {
                    sbus_pa::ratePeriodic();
                    if (sbus_pa::packages() >= minPackages) {
                        mState = State::IsSBusInv;
                    }
                    else {
                        mState = State::InitSBus;
                    }
                }
                else {
                    mState = State::InitSBus;
                }
                break;
            case State::CheckSumD:
                if constexpr(checkSumd) {
                    sumd_pa::ratePeriodic();
                    if (sumd_pa::packages() >= minPackages) {
                        mState = State::IsSumD;
                    }
                    else {
                        mState = State::InitPpm;
                    }
                }
                else {
                    mState = State::InitPpm;
                }
                break;
            case State::CheckPpm:
                if constexpr(checkPpm) {
                    if (ppm_test_dev::counter() > 10) {
                        mState = State::IsPpm;
                    }
                    else {
                        mState = State::InitIBus;
                    }
                }
                else {
                    mState = State::InitIBus;
                }
                break;
            case State::IsSBus:
                blinker::template off<true>();
                if constexpr(termOnScanDev && checkSbus) {
                    while(!sbus_test_dev::isEmpty()) {
                        sbus_test_dev::periodic();
                    }                    
                }
                App<Bus::SBusSPort<Devs>>::run(false);
                mState = State::Init;
                break;
            case State::IsSBusInv:
                blinker::template off<true>();
                if constexpr(termOnScanDev && checkSbus) {
                    while(!sbus_test_dev::isEmpty()) {
                        sbus_test_dev::periodic();
                    }                    
                }
                App<Bus::SBusSPort<Devs>>::run(true);
                mState = State::Init;
                break;
            case State::IsIBus:
                blinker::template off<true>();
                if constexpr(termOnScanDev && checkIbus) {
                    while(!ibus_test_dev::isEmpty()) {
                        ibus_test_dev::periodic();
                    }                    
                }
                App<Bus::IBusIBus<Devs>>::run();
                mState = State::Init;
                break;
            case State::IsSumD:
                blinker::template off<true>();
                if constexpr(termOnScanDev && checkSumd) {
                    while(!sumd_test_dev::isEmpty()) {
                        sumd_test_dev::periodic();
                    }                    
                }
                App<Bus::SumDHott<Devs>>::run();
                mState = State::Init;
                break;
            case State::IsPpm:
                blinker::template off<true>();
                App<Bus::Ppm<Devs>>::run();
                mState = State::Init;
                break;
#ifdef NDEBUG
            default:
                __builtin_unreachable();
                break;
#endif     
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Undefined:
                    break;
                case State::Init:
                    blinker::steady();
                    if constexpr(!std::is_same_v<term_dev, void>) {
                        term_dev::template init<AVR::BaudRate<115200>>();
                        etl::outl<terminal>("s: i "_pgm, GITTAG_PGM);
                    }
                    break;
                case State::InitIBus:
                    if constexpr(!std::is_same_v<scanDevPosition, void> && checkIbus) {
                        ibus_test_dev::template init<AVR::BaudRate<115200>, Duplex>();
                        if constexpr(std::is_same_v<Duplex, AVR::FullDuplex>) {
                            ibus_test_dev::txPinEnable();
                            ibus_test_dev::rxInvert(false);
                        }
                        else {
                            ibus_test_dev::txPinDisable();
                            ibus_test_dev::txInvert(false);
                        }
                        ibus_pa::resetStats();
                        if constexpr(termOnScanDev) {
                            using t = etl::basic_ostream<ibus_test_dev>; 
                            etl::outl<t>("s: iib"_pgm);
                        }
                        else {
                            etl::outl<terminal>("s: iib"_pgm);
                        }      
                    }
                    break;
                case State::InitSBus:
                    if constexpr(!std::is_same_v<scanDevPosition, void> && checkSbus) {
                        sbus_test_dev::template init<AVR::BaudRate<100000>, Duplex, true, 1>();
                        if constexpr(std::is_same_v<Duplex, AVR::FullDuplex>) {
                            sbus_test_dev::txPinEnable();
                            sbus_test_dev::rxInvert(false);
                        }
                        else {
                            sbus_test_dev::txPinDisable();
                            sbus_test_dev::txInvert(false);
                        }
                        sbus_pa::resetStats();
                        if constexpr(termOnScanDev) {
                            using t = etl::basic_ostream<sbus_test_dev>;
                            etl::outl<t>("s: isb"_pgm);
                        }
                        else {
                            etl::outl<terminal>("s: isb"_pgm);
                        }      
                    }
                    break;
                case State::InitSBusInv:
                    if constexpr(!std::is_same_v<scanDevPosition, void> && checkSbus) {
                        sbus_test_dev::template init<AVR::BaudRate<100000>, Duplex, true, 1>();
                        if constexpr(std::is_same_v<Duplex, AVR::FullDuplex>) {
                            sbus_test_dev::txPinEnable();
                            sbus_test_dev::rxInvert(true);
                        }
                        else {
                            sbus_test_dev::txPinDisable();
                            sbus_test_dev::txInvert(true);
                        }
                        sbus_pa::resetStats();
                        if constexpr(termOnScanDev) {
                            using t = etl::basic_ostream<sbus_test_dev>;
                            etl::outl<t>("s: isbi"_pgm);
                        }
                        else {
                            etl::outl<terminal>("s: isbi"_pgm);
                        }      
                    }
                    break;
                case State::InitSumD:
                    if constexpr(!std::is_same_v<scanDevPosition, void> && checkSumd) {
                        sumd_test_dev::template init<AVR::BaudRate<115200>, Duplex>();
                        sumd_test_dev::txPinDisable();
                        if constexpr(std::is_same_v<Duplex, AVR::FullDuplex>) {
                            sumd_test_dev::txPinEnable();
                            sumd_test_dev::rxInvert(false);
                        }
                        else {
                            sumd_test_dev::txPinDisable();
                            sumd_test_dev::txInvert(false);
                        }
                        sumd_pa::resetStats();
                        if constexpr(termOnScanDev) {
                            using t = etl::basic_ostream<sumd_test_dev>;
                            etl::outl<t>("s: isd"_pgm);
                        }
                        else {
                            etl::outl<terminal>("s: isd"_pgm);
                        }      
                    }
                    break;
                case State::InitPpm:
                    if constexpr(!std::is_same_v<typename Devs::ppmDevPosition, void> && checkPpm) {
                        evrouter::init();
                        ppm_test_dev::init();
                        ppm_test_dev::reset();
                        etl::outl<terminal>("s: ippm"_pgm);
                    }
                    break;
                case State::CheckIBus:
                    if constexpr(checkIbus) {
                        if constexpr(termOnScanDev) {
                            using t = etl::basic_ostream<ibus_test_dev>;
                            etl::outl<t>("s: cib"_pgm);
                        }
                        else {
                            etl::outl<terminal>("s: cib"_pgm);
                        }      
                    }
                    break;
                case State::CheckSBus:
                    if constexpr(checkSbus) {
                        if constexpr(termOnScanDev) {
                            using t = etl::basic_ostream<sbus_test_dev>;
                            etl::outl<t>("s: csb"_pgm);
                        }
                        else {
                            etl::outl<terminal>("s: csb"_pgm);
                        }      
                    }
                    break;
                case State::CheckSBusInv:
                    if constexpr(checkSbus) {
                        if constexpr(termOnScanDev) {
                            using t = etl::basic_ostream<sbus_test_dev>;
                            etl::outl<t>("s: csbi"_pgm);
                        }
                        else {
                            etl::outl<terminal>("s: csbi"_pgm);
                        }      
                    }
                    break;
                case State::CheckSumD:
                    if constexpr(checkSumd) {
                        if constexpr(termOnScanDev) {
                            using t = etl::basic_ostream<sumd_test_dev>;
                            etl::outl<t>("s: csd"_pgm);
                        }
                        else {
                            etl::outl<terminal>("s: csd"_pgm);
                        }      
                    }
                    break;
                case State::CheckPpm:
                    if constexpr(checkPpm) {
                        etl::outl<terminal>("s: cppm"_pgm);
                    }
                    break;
                case State::IsIBus:
                    if constexpr(termOnScanDev) {
                        using t = etl::basic_ostream<ibus_test_dev>;
                        etl::outl<t>("s: IB"_pgm);
                    }
                    else {
                        etl::outl<terminal>("s: IB"_pgm);
                    }      
                    break;
                case State::IsSBus:
                    if constexpr(termOnScanDev) {
                        using t = etl::basic_ostream<sbus_test_dev>;
                        etl::outl<t>("s: SB"_pgm);
                    }
                    else {
                        etl::outl<terminal>("s: SB"_pgm);
                    }      
                    break;
                case State::IsSBusInv:
                    if constexpr(termOnScanDev) {
                        using t = etl::basic_ostream<sbus_test_dev>;
                        etl::outl<t>("s: SBI"_pgm);
                    }
                    else {
                        etl::outl<terminal>("s: SBI"_pgm);
                    }      
                    break;
                case State::IsSumD:
                    if constexpr(termOnScanDev) {
                        using t = etl::basic_ostream<sumd_test_dev>;
                        etl::outl<t>("s: SD"_pgm);
                    }
                    else {
                        etl::outl<terminal>("s: SD"_pgm);
                    }      
                    break;
                case State::IsPpm:
                    etl::outl<terminal>("s: SPpm"_pgm);
                    break;
#ifdef NDEBUG
                default:
                    __builtin_unreachable();
                    break;
#endif     
                }
            }        
        }
        static inline State mState{State::Undefined};
        static inline External::Tick<systemTimer> mTotalTick;
        static inline External::Tick<systemTimer> mStateTick;
    };
    
}
