#pragma once

#include <mcu/avr.h>
#include <mcu/internals/port.h>
#include <mcu/internals/adcomparator.h>
#include <mcu/internals/pwm.h>

#include <external/units/physical.h>
#include <external/units/percent.h>

#include <etl/meta.h>

namespace BLDC {

    template<typename AC, typename Capture, typename PWM, typename LowPins, typename MCU = DefaultMcuType>
    struct Communter;
    
    template<typename AC, typename Capture, typename PWM, typename... PP, AVR::Concepts::At01Series MCU>
    struct Communter<AC, Capture, PWM, Meta::List<PP...>, MCU> {
        using pin_list = Meta::List<PP...>;
        using l0 = Meta::nth_element<0, pin_list>;
        using l1 = Meta::nth_element<1, pin_list>;
        using l2 = Meta::nth_element<2, pin_list>;
        
        using h0 = AVR::PWM::WO<0>;
        using h1 = AVR::PWM::WO<1>;
        using h2 = AVR::PWM::WO<2>;
        
        using lowSides  = Meta::List<l0, l1, l2>;
        
        using index_type = etl::uint_ranged_circular<uint8_t, 0, 5>;
        
        template<typename Pin>
        requires Meta::contains<lowSides, Pin>::value
        inline static void floating() {
            Pin::off();
        }
        template<typename Pin>
        requires Meta::contains<lowSides, Pin>::value
        inline static void on() {
            Pin::on();
        }
        
        template<typename W>
        inline static void off() {
            PWM::template off<W>();
        }

        template<typename W>
        inline static void pwm() {
            PWM::template on<W>();
        }
        
        template<uint8_t N>
        inline static void pwm() {
            floating<Meta::nth_element<N, lowSides>>();
            PWM::template on<AVR::PWM::WO<N>>();
        }
        template<uint8_t N>
        inline static void floating() {
            floating<Meta::nth_element<N, lowSides>>();
            PWM::template off<AVR::PWM::WO<N>>();
        }
        template<uint8_t N>
        inline static void low() {
            PWM::template off<AVR::PWM::WO<N>>();
            on<Meta::nth_element<N, lowSides>>();
        }
        
        inline static void init() {
            ((PP::off(), ...));
            ((PP::template dir<AVR::Output>(), ...));
            AC::init();
            AC::positiv_channel(0);
            Capture::init();
//            AC::enableCapture();
        }
        inline static void enable() {
            AC::init();
        }
        inline static void disable() {
            AC::disable();
        }
        inline static void off() {
            ((PP::template dir<AVR::Output>(), ...));
            (PP::off(), ...);
        }
        inline static void startPosition() {
            state = 0;
        }
        inline static void on_full_state() {
            switch(state) {
            case 0: // pwm(0) -> 2, ac = 1, rising
                floating<1>();
                pwm<0>();                
                low<2>();                
                AC::negativ_channel(1);
                AC::template edge<typename AC::Positiv>();
//                Capture::captureRising(true);
                break;
            case 1: // pwm(1) -> 2, ac = 0
                floating<0>();
                low<2>();                
                pwm<1>();                
                AC::negativ_channel(0);
                AC::template edge<typename AC::Negativ>();
//                Capture::captureRising(false);
                break;
            case 2: // pwm(1) -> 0, ac = 2
                floating<2>();                
                low<0>();
                pwm<1>();                
                AC::negativ_channel(2);
                AC::template edge<typename AC::Positiv>();
//                Capture::captureRising(true);
                break;
            case 3: // pwm(2) -> 0, ac = 1
                floating<1>();                
                low<0>();
                pwm<2>();                
                AC::negativ_channel(1);
                AC::template edge<typename AC::Negativ>();
//                Capture::captureRising(false);
                break;
            case 4: // pwm(2) -> 1, ac = 0
                floating<0>();
                low<1>();                
                pwm<2>();                
                AC::negativ_channel(0);
                AC::template edge<typename AC::Positiv>();
//                Capture::captureRising(true);
                break;
            case 5: // pwm(0) -> 1, ac = 2
                floating<2>();                
                low<1>();                
                pwm<0>();
                AC::negativ_channel(2);
                AC::template edge<typename AC::Negativ>();
//                Capture::captureRising(false);
                break;
            default:
                break;
            }
        }
        
        inline static void on() {
            switch(state) {
            case 0: // pwm(0) -> 2, ac = 1, rising
                floating<l1>();
                on<l2>();
                AC::negativ_channel(1);
                AC::template edge<typename AC::Positiv>();
//                Capture::captureRising(true);
                break;
            case 1: // pwm(1) -> 2, ac = 0
                off<h0>();
                pwm<h1>();
                AC::negativ_channel(0);
                AC::template edge<typename AC::Negativ>();
//              Capture::captureRising(false);
                break;
            case 2: // pwm(1) -> 0, ac = 2
                floating<l2>();
                on<l0>();
                AC::negativ_channel(2);
                AC::template edge<typename AC::Positiv>();
//                Capture::captureRising(true);
                break;
            case 3: // pwm(2) -> 0, ac = 1
                off<h1>();
                pwm<h2>();
                AC::negativ_channel(1);
                AC::template edge<typename AC::Negativ>();
//                Capture::captureRising(false);
                break;
            case 4: // pwm(2) -> 1, ac = 0
                floating<l0>();
                on<l1>();
                AC::negativ_channel(0);
                AC::template edge<typename AC::Positiv>();
//                Capture::captureRising(true);
                break;
            case 5: // pwm(0) -> 1, ac = 2
                off<h2>();
                pwm<h0>();
                AC::negativ_channel(2);
                AC::template edge<typename AC::Negativ>();
//                Capture::captureRising(false);
                break;
            default:
                break;
            }
        }
        
        inline static void next() {
            ++state;
            on();
        }
        inline static void set(index_type p) {
            state = p;
            on_full_state();
        }
    private:
        inline static index_type state{0};
    };
    
}
