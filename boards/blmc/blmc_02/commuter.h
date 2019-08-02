#pragma once

//#include <mcu/avr.h>
//#include <mcu/internals/systemclock.h>
//#include <mcu/internals/usart.h>
//#include <mcu/internals/port.h>
//#include <mcu/internals/adc.h>
//#include <mcu/internals/adcomparator.h>
//#include <mcu/internals/capture.h>
//#include <mcu/internals/pwm.h>

//#include <external/units/physical.h>
//#include <external/units/percent.h>

//#include <etl/meta.h>
//#include <etl/fixedpoint.h>
//#include <etl/output.h>

//#include <std/chrono>

namespace BLDC {
    
    template<typename AC, typename Capture, typename... PP>
    struct Communter {
        using pin_list = Meta::List<PP...>;
        using h0 = Meta::nth_element<0, pin_list>;
        using h1 = Meta::nth_element<1, pin_list>;
        using h2 = Meta::nth_element<2, pin_list>;
        using l0 = Meta::nth_element<3, pin_list>;
        using l1 = Meta::nth_element<4, pin_list>;
        using l2 = Meta::nth_element<5, pin_list>;
        
        using highSides = Meta::List<h0, h1, h2>;
        using lowSides  = Meta::List<l0, l1, l2>;
        
        using index_type = etl::uint_ranged_circular<uint8_t, 0, 5>;
        
        template<typename Pin>
        requires Meta::contains<highSides, Pin>::value
        inline static void pwm() {
            Pin::template dir<AVR::Input>();
        }
        template<typename Pin>
        requires Meta::contains<highSides, Pin>::value
        inline static void off() {
            Pin::template dir<AVR::Output>();
        }
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
        
        template<uint8_t N>
        inline static void pwm() {
            floating<Meta::nth_element<N, lowSides>>();
            pwm<Meta::nth_element<N, highSides>>();
        }
        template<uint8_t N>
        inline static void floating() {
            floating<Meta::nth_element<N, lowSides>>();
            off<Meta::nth_element<N, highSides>>();
        }
        template<uint8_t N>
        inline static void low() {
            off<Meta::nth_element<N, highSides>>();
            on<Meta::nth_element<N, lowSides>>();
        }
        
        inline static void init() {
            ((PP::off(), ...));
            ((PP::template dir<AVR::Output>(), ...));
            AC::init();
            AC::enableCapture();
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
                AC::channel(1);
                Capture::captureRising(true);
                break;
            case 1: // pwm(1) -> 2, ac = 0
                floating<0>();
                low<2>();                
                pwm<1>();                
                AC::channel(0);
                Capture::captureRising(false);
                break;
            case 2: // pwm(1) -> 0, ac = 2
                floating<2>();                
                low<0>();
                pwm<1>();                
                AC::channel(2);
                Capture::captureRising(true);
                break;
            case 3: // pwm(2) -> 0, ac = 1
                floating<1>();                
                low<0>();
                pwm<2>();                
                AC::channel(1);
                Capture::captureRising(false);
                break;
            case 4: // pwm(2) -> 1, ac = 0
                floating<0>();
                low<1>();                
                pwm<2>();                
                AC::channel(0);
                Capture::captureRising(true);
                break;
            case 5: // pwm(0) -> 1, ac = 2
                floating<2>();                
                low<1>();                
                pwm<0>();
                AC::channel(2);
                Capture::captureRising(false);
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
                AC::channel(1);
                Capture::captureRising(true);
                break;
            case 1: // pwm(1) -> 2, ac = 0
                off<h0>();
                pwm<h1>();
                AC::channel(0);
                Capture::captureRising(false);
                break;
            case 2: // pwm(1) -> 0, ac = 2
                floating<l2>();
                on<l0>();
                AC::channel(2);
                Capture::captureRising(true);
                break;
            case 3: // pwm(2) -> 0, ac = 1
                off<h1>();
                pwm<h2>();
                AC::channel(1);
                Capture::captureRising(false);
                break;
            case 4: // pwm(2) -> 1, ac = 0
                floating<l0>();
                on<l1>();
                AC::channel(0);
                Capture::captureRising(true);
                break;
            case 5: // pwm(0) -> 1, ac = 2
                off<h2>();
                pwm<h0>();
                AC::channel(2);
                Capture::captureRising(false);
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
