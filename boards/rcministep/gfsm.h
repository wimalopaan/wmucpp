#pragma once

#include <mcu/avr.h>

#include <mcu/internals/port.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/event.h>

#include <external/solutions/rc/busscan.h>
#include <external/solutions/tick.h>

namespace Bus {
    using namespace AVR;
    using namespace std::literals::chrono;
    using namespace External::Units::literals;

    template<typename BusDevs, typename MCU = DefaultMcuType>
    struct GFSM;
    
    template<template<typename> typename BD, typename Devices, typename MCU>
    requires(External::Bus::isIBus<typename BD<Devices>::bus_type>::value || External::Bus::isSBus<typename BD<Devices>::bus_type>::value)
    struct GFSM<BD<Devices>, MCU> {
        static inline constexpr bool noAdc = std::is_same_v<MCU, ATTiny412>;
        static inline constexpr bool noTerm= std::is_same_v<MCU, ATTiny412>;
        using BusDevs = BD<Devices>;
        using devices = BusDevs::devs;
        using bus     = BusDevs::bus_type;
        using term_dev = std::conditional_t<noTerm, void, typename devices::term_dev>;
        using adc      = std::conditional_t<noAdc, void, typename devices::adcController>;
        using timer    = devices::systemTimer;
        using stepper  = devices::stepper;
        using servo    = devices::servo;
        using pa       = servo::protocoll_adapter_type;
        
        using terminal = etl::basic_ostream<term_dev>;
//        using adc_i_t  = std::conditional_t<noAdc, void, typename adc::index_type>;
        
        inline static constexpr External::Tick<timer> debugTicks{1000_ms}; 
        
        using speed_type = etl::uint_ranged<uint8_t, 1, 8>;
        
        inline static constexpr void init() {
            if constexpr(External::Bus::isIBus<bus>::value) {
                servo::template init<BaudRate<115200>>();
                servo::template txEnable<false>();
            }
            stepper::init();
            if constexpr(!noAdc) {
                adc::init();
            }
            if constexpr(!std::is_same_v<term_dev, void>) {
                if constexpr(!std::is_same_v<term_dev, servo>) {
                    term_dev::template init<AVR::BaudRate<115200>>();
                }
            }
        }
        inline static constexpr void periodic() {
            stepper::periodic();
            servo::periodic();
            if constexpr(!noAdc) {
                adc::periodic();
            }
            if constexpr(!std::is_same_v<term_dev, void>) {
                term_dev::periodic();
            }
        }
        inline static constexpr void ratePeriodic() {
            (++speedDivider).on(speed, []{
                stepper::ratePeriodic();
            });
            
            if (const auto s = pa::value(0); s) {
                speed = etl::scaleTo<speed_type>(s.toRanged());
            } 
            if (const auto b = pa::value(1); b) {
                stepper::balance(b.toRanged());
            }
            if constexpr(!noAdc) {
                using adc_i_t  = adc::index_type;
                const auto s = adc::value(adc_i_t{0});
                speed = etl::scaleTo<speed_type>(s);
            }
            
            ++stateTick;
            stateTick.on(debugTicks, [&]{
    //           etl::outl<terminal>("bal: "_pgm, balance, " s0: "_pgm, Stepper::mScale0, " s1: "_pgm, Stepper::mScale1, " ds: "_pgm, Stepper::deltaShift); 
//               etl::outl<terminal>(" i0: "_pgm, stepper::index0.toInt(), " i1: "_pgm, stepper::index1.toInt(), " ds: "_pgm, stepper::deltaShift); 
                 etl::outl<terminal>("s: "_pgm, speed); 
               
            });
            
    //        Stepper::balance(balance);
    //        Stepper::shift(balance);
        }
    private:
        inline static speed_type speed{1};
        inline static External::CountDown speedDivider{};
        inline static External::Tick<timer> stateTick{}; 
    };

        template<template<typename> typename BD, typename Devices, typename MCU>
        struct GFSM<BD<Devices>, MCU> {
            using BusDevs = BD<Devices>;
            using devices = BusDevs::devs;
            using bus     = BusDevs::bus_type;
            using term_dev = devices::term_dev;
            using adc      = devices::adcController;
            using timer    = devices::systemTimer;
            using stepper  = devices::stepper;
            
            using terminal = etl::basic_ostream<term_dev>;
            using adc_i_t = adc::index_type;
            
            inline static constexpr External::Tick<timer> debugTicks{1000_ms}; 
            
            using speed_type = etl::uint_ranged<uint8_t, 1, 8>;
            
            inline static constexpr void init() {
                stepper::init();
                adc::init();
            }
            inline static constexpr void periodic() {
                stepper::periodic();
                adc::periodic();
                if constexpr(!std::is_same_v<term_dev, void>) {
                    term_dev::periodic();
                }
            }
            inline static constexpr void ratePeriodic() {
                (++speedDivider).on(speed, []{
                    stepper::ratePeriodic();
                });
                
                
                ++stateTick;
                stateTick.on(debugTicks, [&]{
        //           etl::outl<terminal>("bal: "_pgm, balance, " s0: "_pgm, Stepper::mScale0, " s1: "_pgm, Stepper::mScale1, " ds: "_pgm, Stepper::deltaShift); 
    //               etl::outl<terminal>(" i0: "_pgm, stepper::index0.toInt(), " i1: "_pgm, stepper::index1.toInt(), " ds: "_pgm, stepper::deltaShift); 
                     etl::outl<terminal>("s: "_pgm, speed); 
                   
                });
                
        //        Stepper::balance(balance);
        //        Stepper::shift(balance);
            }
        private:
            inline static speed_type speed{8};
            inline static External::CountDown speedDivider{};
            inline static External::Tick<timer> stateTick{}; 
        };
}

