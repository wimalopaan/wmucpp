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
    requires(External::Bus::isIBus<typename BD<Devices>::bus_type>::value || External::Bus::isSBus<typename BD<Devices>::bus_type>::value || External::Bus::isSumD<typename BD<Devices>::bus_type>::value)
    struct GFSM<BD<Devices>, MCU> {
        using BusDevs = BD<Devices>;
        using devices = BusDevs::devs;
        using bus     = BusDevs::bus_type;
        using term_dev = typename BusDevs::term_dev;
        using timer    = devices::systemTimer;
        using stepper  = devices::stepper;
        using servo    = BusDevs::servo;
        using servo_pa = BusDevs::servo_pa;
        using servo_v_t=servo_pa::value_type;
        using nvm      = devices::eeprom;
        using pa       = servo::protocoll_adapter_type;
        
//        bus::_;
        
        using terminal = etl::basic_ostream<term_dev>;
        
        inline static constexpr External::Tick<timer> debugTicks{300_ms}; 
        inline static constexpr External::Tick<timer> eepromTicks{1000_ms}; 
        inline static constexpr External::Tick<timer> initTicks{300_ms}; 
        
        using blinker = External::Blinker2<typename devices::scanLedPin, timer, 100_ms, 2000_ms>;
        using bl_t = blinker::count_type;
        using speed_type = etl::uint_ranged<uint8_t, 1, 8>;
        
        enum class State : uint8_t {Undefined, Init, Setup, Run, Load};
        
        inline static constexpr void init(const bool inverted = false) {
            nvm::init();
            if (nvm::data().mMagic != 42) {
                nvm::data().mMagic = 42;
                nvm::data().mSpeed = 1;
                nvm::data().mCurrent = 1;
                nvm::data().mBalance = (servo_v_t::Upper + servo_v_t::Lower) / 2;
#ifndef NO_SHIFT
                nvm::data().mShift = (servo_v_t::Upper + servo_v_t::Lower) / 2;
#endif
                nvm::data().mSine = 0;
                nvm::data().changed();
            }
            if constexpr(External::Bus::isIBus<bus>::value) {
                servo::template init<BaudRate<115200>>();
                etl::outl<terminal>("IB"_pgm); 
            }
            else if constexpr(External::Bus::isSBus<bus>::value) {
                servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
                if (inverted) {
                    etl::outl<terminal>("SBI"_pgm); 
                    servo::rxInvert(true);
                }
                else {
                    etl::outl<terminal>("SBI"_pgm); 
                }
            }
            blinker::init();
        }
        inline static constexpr void periodic() {
            stepper::periodic();
            servo::periodic();
            if constexpr(!std::is_same_v<term_dev, void>) {
                term_dev::periodic();
            }
            nvm::saveIfNeeded([&]{
                etl::outl<terminal>("ep s"_pgm);
            });
        }
        inline static constexpr void ratePeriodic() {
            blinker::ratePeriodic();
            servo_pa::ratePeriodic();
            (++eepromTick).on(eepromTicks, []{
                nvm::data().expire();
            });
            (++debugTick).on(debugTicks, [&]{
                etl::outl<terminal>("s: "_pgm, stepper::mSineType); 
//                 etl::outl<terminal>("s: "_pgm, speed, " p: "_pgm, servo_pa::packages()); 
//                 etl::outl<terminal>("sc0: "_pgm, stepper::mScale0, " sc1: "_pgm, stepper::mScale1, " max: "_pgm, stepper::mCurrentMax); 
            });

            const auto oldState = mState;
            ++stateTick;
            
            switch(mState) {
            case State::Undefined:
                stateTick.on(initTicks, []{
                    packages = servo_pa::packages();
                    mState = State::Init; 
                });
                break;
            case State::Init:
                stateTick.on(initTicks, []{
                    if (packages != servo_pa::packages()) {
                        mState = State::Setup; 
                    }
                    else {
                        mState = State::Load; 
                    }
                });
                break;
            case State::Load:
                mState = State::Run;
                break;
            case State::Setup:
                if (const auto s = pa::value(0); s) {
                    speed = etl::scaleTo<speed_type>(s.toRanged());
                    nvm::data().mSpeed = speed.toInt();
                    nvm::data().change();
                } 
                if (const auto b = pa::value(1); b) {
                    stepper::balance(b.toRanged());
                    nvm::data().mBalance = b.toInt();
                    nvm::data().change();
                }
                if (const auto c = pa::value(2); c) {
                    stepper::current(c.toRanged());
                    nvm::data().mCurrent = c.toInt();
                    nvm::data().change();
                }
#ifndef NO_SHIFT
                if (const auto c = pa::value(3); c) {
                    stepper::shift(c.toRanged());
                    nvm::data().mShift = c.toInt();
                    nvm::data().change();
                }
                if (const auto c = pa::value(4); c) {
                    stepper::sine(c.toRanged());
                    nvm::data().mSine = c.toInt();
                    nvm::data().change();
                }
#else
                if (const auto c = pa::value(3); c) {
                    stepper::sine(c.toRanged());
                    nvm::data().mSine = c.toInt();
                    nvm::data().change();
                }
#endif
                stateTick.on(initTicks, []{
                    if (servo_pa::packages() == 0) {
                        mState = State::Run; 
                    }
                    servo_pa::resetStats();
                });
                [[fallthrough]];
            case State::Run:
                (++speedDivider).on(speed, []{
                    stepper::ratePeriodic();
                });
                break;
            }
            if (oldState != mState) {
                stateTick.reset();
                switch (mState) {
                case State::Undefined:
                    break;
                case State::Init:
//                    etl::outl<terminal>("S I"_pgm); 
                    break;
                case State::Setup:
//                    etl::outl<terminal>("S S"_pgm); 
                    blinker::blink(bl_t{2});
                    stepper::setDuty();
                    stepper::init();
                    break;
                case State::Load:
//                    etl::outl<terminal>("S L s:"_pgm, nvm::data().mSpeed, " b: "_pgm, nvm::data().mBalance, " c: "_pgm, nvm::data().mCurrent); 
                    speed.set(nvm::data().mSpeed);
                    stepper::balance(servo_v_t{nvm::data().mBalance}.toRanged());
                    stepper::current(servo_v_t{nvm::data().mCurrent}.toRanged());
#ifndef NO_SHIFT                    
                    stepper::shift(servo_v_t{nvm::data().mShift}.toRanged());
#endif
                    stepper::sine(servo_v_t{nvm::data().mSine}.toRanged());
                    stepper::setDuty();
                    stepper::init();
                    break;
                case State::Run:
//                    etl::outl<terminal>("S R"_pgm); 
                    blinker::blink(bl_t{1});
                    break;
                }
            }
        }
    private:
        inline static uint16_t packages{};
        inline static State mState{State::Undefined};
        inline static speed_type speed{1};
        inline static External::CountDown speedDivider{};
        inline static External::Tick<timer> eepromTick{}; 
        inline static External::Tick<timer> stateTick{}; 
        inline static External::Tick<timer> debugTick{}; 
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
            
            inline static constexpr External::Tick<timer> debugTicks{200_ms}; 
            
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
//                    etl::outl<terminal>("s: "_pgm, speed); 
                    const auto sv = adc::value(adc_i_t{0});
                    speed.set((sv >> 7) + 1);
                });
            }
        private:
            inline static speed_type speed{8};
            inline static External::CountDown speedDivider{};
            inline static External::Tick<timer> stateTick{}; 
        };
}

