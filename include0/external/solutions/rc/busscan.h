#pragma once

#include <mcu/internals/usart.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>

#include <external/solutions/blinker.h>

namespace External {
    using namespace AVR;
    using namespace std::literals::chrono;
    using namespace External::Units::literals;
    
    namespace Bus {
        static inline constexpr auto fRtc = 1000_Hz;
        
        template<typename Devs>
        struct SBusSPort {
            using devs = Devs;
            using servo_pa = External::SBus::Servo::ProtocollAdapter<0, typename Devs::systemTimer>;
            using periodic_dev = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
            using terminal_device = periodic_dev;
        };
        template<typename Devs>
        struct IBusIBus {
            using devs = Devs;
            using servo_pa = IBus::Servo::ProtocollAdapter<0>;
            using periodic_dev = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
            using terminal_device = periodic_dev;
        };
        template<typename Devs>
        struct SumDHott {
            using devs = Devs;
            using periodic_dev = Hott::Experimental::Sensor<typename Devs::servoPosition, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, typename Devs::systemTimer>;
            using servo_pa = periodic_dev::ProtocollAdapter;
            using terminal_device = void;
        };
        
        template<typename> struct isSBus : std::false_type {};
        template<typename D> struct isSBus<SBusSPort<D>> : std::true_type {};

        template<typename> struct isIBus : std::false_type {};
        template<typename D> struct isIBus<IBusIBus<D>> : std::true_type {};

        template<typename> struct isSumD : std::false_type {};
        template<typename D> struct isSumD<SumDHott<D>> : std::true_type {};
    }
    
    template<typename Devs, template<typename> typename App>
    struct Scanner {
        enum class State : uint8_t {Undefined, Init, 
                                    InitSBus, InitSBusInv, InitIBus, InitSumD,
                                    CheckSBus, CheckSBusInv, CheckIBus, CheckSumD, 
                                    IsSBus, IsSBusInv, IsIBus, IsSumD};
         
        using devs = Devs;
        using systemTimer = devs::systemTimer;
    
        using ibus_pa = IBus::Servo::ProtocollAdapter<0>;
        using ibus_test_dev = Usart<typename Devs::scanDevPosition, ibus_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
        
        using sbus_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
        using sbus_test_dev = Usart<typename Devs::scanDevPosition, sbus_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
        
        using sumd_pa = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
        using sumd_test_dev = Usart<typename Devs::scanDevPosition, sumd_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
        
        using term_dev = devs::scan_term_dev;
        using terminal = etl::basic_ostream<term_dev>;

        using ledPin = std::conditional_t<std::is_same_v<typename Devs::scanLedPin, void>, AVR::NoPin, typename Devs::scanLedPin>;
        using led = AVR::ActiveHigh<typename Devs::scanLedPin, AVR::Output>;
        using blinker = External::Blinker2<led, systemTimer, 100_ms, 2000_ms>;
        
        static constexpr External::Tick<systemTimer> initTimeout{100_ms};
        static constexpr External::Tick<systemTimer> checkTimeout{1000_ms};
        
        static constexpr uint8_t minPackages{10};
        
        [[noreturn]] static inline auto run() {
            devs::init();
            while(true) {
                periodic();            
                systemTimer::periodic([]{
                    ratePeriodic();
                });
            }
        }   
    private:
        inline static void periodic() {
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
            case State::Init:
            case State::IsIBus:
            case State::IsSBus:
            case State::IsSBusInv:
            case State::IsSumD:
            case State::Undefined:
                break;
            }                    
        }
        inline static void ratePeriodic() {
            const auto oldState = mState;
            ++mStateTick;
            ++mCheckTick;
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
                    mState = State::InitIBus;
                }
                break;
            case State::IsSBus:
                App<Bus::SBusSPort<Devs>>::run(false);
                mState = State::Init;
                break;
            case State::IsSBusInv:
                App<Bus::SBusSPort<Devs>>::run(true);
                mState = State::Init;
                break;
            case State::IsIBus:
                App<Bus::IBusIBus<Devs>>::run();
                mState = State::Init;
                break;
            case State::IsSumD:
                App<Bus::SumDHott<Devs>>::run();
                mState = State::Init;
                break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Undefined:
                    break;
                case State::Init:
                    if constexpr(!std::is_same_v<term_dev, void>) {
                        term_dev::template init<AVR::BaudRate<115200>>();
                    }
                    etl::outl<terminal>("s: i "_pgm, GITTAG_PGM);
                    break;
                case State::InitIBus:
                    ibus_test_dev::template init<AVR::BaudRate<115200>>();
                    sbus_test_dev::rxInvert(false);
                    ibus_pa::resetStats();
                    etl::outl<terminal>("s: iib"_pgm);
                    break;
                case State::InitSBus:
                    sbus_test_dev::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>();
                    sbus_test_dev::rxInvert(false);
                    sbus_pa::resetStats();
                    etl::outl<terminal>("s: isb"_pgm);
                    break;
                case State::InitSBusInv:
                    sbus_test_dev::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>();
                    sbus_test_dev::rxInvert(true);
                    sbus_pa::resetStats();
                    etl::outl<terminal>("s: isbi"_pgm);
                    break;
                case State::InitSumD:
                    sumd_test_dev::template init<AVR::BaudRate<115200>>();
                    sbus_test_dev::rxInvert(false);
                    sumd_pa::resetStats();
                    etl::outl<terminal>("s: isd"_pgm);
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
                }
            }        
        }
        static inline State mState{State::Undefined};
        static inline External::Tick<systemTimer> mCheckTick;
        static inline External::Tick<systemTimer> mStateTick;
    };
    
}