#pragma once

#include <mcu/avr.h>
#include <mcu/internals/port.h>
#include <mcu/internals/adcomparator.h>
#include <mcu/internals/pwm.h>

#include <external/units/physical.h>
#include <external/units/percent.h>

#include <etl/meta.h>

namespace BLDC {

    template<typename AC, typename PWM, typename LowPins, typename Debug = void, typename MCU = DefaultMcuType>
    struct Communter;
    
    template<typename AC, typename PWM, typename... PP, typename Debug, AVR::Concepts::At01Series MCU>
    struct Communter<AC, PWM, Meta::List<PP...>, Debug, MCU> {
        using pin_list = Meta::List<PP...>;
        using l0 = Meta::nth_element<0, pin_list>;
        using l1 = Meta::nth_element<1, pin_list>;
        using l2 = Meta::nth_element<2, pin_list>;
        
        using h0 = AVR::PWM::WO<0>;
        using h1 = AVR::PWM::WO<1>;
        using h2 = AVR::PWM::WO<2>;
        
        using lowSides  = Meta::List<l0, l1, l2>;
        
        using state_type = etl::uint_ranged_circular<uint8_t, 0, 5>;
        using ac_index_t = typename AC::index_type;
        
        template<typename Pin>
        requires (Meta::contains<lowSides, Pin>::value)
        inline static void floating() {
            Pin::off();
        }

        template<typename Pin>
        requires (Meta::contains<lowSides, Pin>::value && requires {Pin::lutOn();})
        inline static void lutOn() {
            Pin::lutOn();
        }
        template<typename Pin>
        requires (Meta::contains<lowSides, Pin>::value && !requires {Pin::lutOn();})
        inline static void lutOn() {
        }
        template<typename Pin>
        requires (Meta::contains<lowSides, Pin>::value && requires {Pin::lutOn();})
        inline static void lutOff() {
            Pin::off();
        }
        template<typename Pin>
        requires (Meta::contains<lowSides, Pin>::value && !requires {Pin::lutOn();})
        inline static void lutOff() {
        }
        
        template<typename Pin>
        requires Meta::contains<lowSides, Pin>::value
        inline static void on() {
            Pin::on();
        }
        
        template<typename W>
        inline static void off() {
            PWM::template off<Meta::List<W>>();
        }

        template<typename W>
        inline static void pwm() {
            PWM::template on<Meta::List<W>, etl::NoDisableEnable>();
        }
        
        template<uint8_t N>
        inline static void pwm() {
            using ls = Meta::nth_element<N, lowSides>;
//            floating<ls>();
            lutOn<ls>();
            PWM::template on<Meta::List<AVR::PWM::WO<N>>, etl::NoDisableEnable>();
        }
        template<uint8_t N>
        inline static void floating() {
            using ls = Meta::nth_element<N, lowSides>;
            floating<ls>();
            PWM::template off<Meta::List<AVR::PWM::WO<N>>>();
        }
        template<uint8_t N>
        inline static void low() {
            using ls = Meta::nth_element<N, lowSides>;
            PWM::template off<Meta::List<AVR::PWM::WO<N>>>();
            on<ls>();
        }
        
        inline static void init() {
            if constexpr(!std::is_same_v<Debug, void>) {
                Debug::template dir<AVR::Output>();
            }
            ((PP::off(), ...));
            ((PP::template dir<AVR::Output>(), ...));
            AC::init();
            AC::positiv_channel(ac_index_t{1});
        }
        inline static void off() {
            floating<0>();
            floating<1>();
            floating<2>();
        }
        inline static void startPosition() {
            if (mReverse) {
                state = state_type{5};
            }
            else {
                state = state_type{0};
            }
        }
        inline static void on_full_state() {
            switch(state.toInt()) {
            case 0: // pwm(0) -> 2, ac = 1, rising
                floating<1>();
                low<2>();                
                AC::negativ_channel(ac_index_t{1});
                AC::template edge<typename AC::Positiv>();
                pwm<0>();                
                break;
            case 1: // pwm(1) -> 2, ac = 0
                floating<0>();
                low<2>();                
                AC::negativ_channel(ac_index_t{0});
                AC::template edge<typename AC::Negativ>();
                pwm<1>();                
                break;
            case 2: // pwm(1) -> 0, ac = 2
                floating<2>();                
                low<0>();
                AC::negativ_channel(ac_index_t{2});
                AC::template edge<typename AC::Positiv>();
                pwm<1>();                
                break;
            case 3: // pwm(2) -> 0, ac = 1
                floating<1>();                
                low<0>();
                AC::negativ_channel(ac_index_t{1});
                AC::template edge<typename AC::Negativ>();
                pwm<2>();                
                break;
            case 4: // pwm(2) -> 1, ac = 0
                floating<0>();
                low<1>();                
                AC::negativ_channel(ac_index_t{0});
                AC::template edge<typename AC::Positiv>();
                pwm<2>();                
                break;
            case 5: // pwm(0) -> 1, ac = 2
                floating<2>();                
                AC::negativ_channel(ac_index_t{2});
                AC::template edge<typename AC::Negativ>();
                low<1>();                
                pwm<0>();
                break;
            default:
                break;
            }
        }
        
        inline static void on() {
            switch(state.toInt()) {
            case 0: // pwm(0) -> 2, ac = 1, rising
                floating<l1>();
                AC::negativ_channel(ac_index_t{1});
                AC::template edge<typename AC::Negativ>();
                on<l2>();
                break;
            case 1: // pwm(1) -> 2, ac = 0
                off<h0>();
                AC::negativ_channel(ac_index_t{0});
                AC::template edge<typename AC::Positiv>();
                pwm<h1>();
                break;
            case 2: // pwm(1) -> 0, ac = 2
                floating<l2>();
                AC::negativ_channel(ac_index_t{2});
                AC::template edge<typename AC::Negativ>();
                on<l0>();
                break;
            case 3: // pwm(2) -> 0, ac = 1
                off<h1>();
                AC::negativ_channel(ac_index_t{1});
                AC::template edge<typename AC::Positiv>();
                pwm<h2>();
                break;
            case 4: // pwm(2) -> 1, ac = 0
                floating<l0>();
                AC::negativ_channel(ac_index_t{0});
                AC::template edge<typename AC::Negativ>();
                on<l1>();
                break;
            case 5: // pwm(0) -> 1, ac = 2
                off<h2>();
                AC::negativ_channel(ac_index_t{2});
                AC::template edge<typename AC::Positiv>();
                pwm<h0>();
                break;
            default:
                break;
            }
        }

        inline static void on_reverse() {
            switch(state.toInt()) {
            case 0: // pwm(0) -> 2, ac = 1, rising
                floating<l2>();
                AC::negativ_channel(ac_index_t{2});
                AC::template edge<typename AC::Negativ>();
                on<l1>();
                break;
            case 1: // pwm(1) -> 2, ac = 0
                off<h0>();
                AC::negativ_channel(ac_index_t{0});
                AC::template edge<typename AC::Positiv>();
                pwm<h2>();
                break;
            case 2: // pwm(1) -> 0, ac = 2
                floating<l1>();
                AC::negativ_channel(ac_index_t{1});
                AC::template edge<typename AC::Negativ>();
                on<l0>();
                break;
            case 3: // pwm(2) -> 0, ac = 1
                off<h2>();
                AC::negativ_channel(ac_index_t{2});
                AC::template edge<typename AC::Positiv>();
                pwm<h1>();
                break;
            case 4: // pwm(2) -> 1, ac = 0
                floating<l0>();
                AC::negativ_channel(ac_index_t{0});
                AC::template edge<typename AC::Negativ>();
                on<l2>();
                break;
            case 5: // pwm(0) -> 1, ac = 2
                off<h1>();
                AC::negativ_channel(ac_index_t{1});
                AC::template edge<typename AC::Positiv>();
                pwm<h0>();
                break;
            default:
                break;
            }
        }
        
        inline static void next() {
            if constexpr(!std::is_same_v<Debug, void>) {
                Debug::toggle();
            }
            ++state;
            if (mReverse) {
                on_reverse();
            } else {
                on();
            }
        }
        inline static void set(state_type p) {
            if (mReverse) {
                state = p.flip();                
            }
            else {
                state = p;
            }
            if constexpr(!std::is_same_v<Debug, void>) {
                Debug::toggle();
            }
            on_full_state();
        }

        template<uint8_t S, typename F>
        inline static void onState(F f) {
            static_assert(S < 6);
            if (state.toInt() == S) {
                f();
            }
        }
        
//        inline static void set(volatile index_type p) {
//            if (mReverse) {
//                state = p.Upper - p.toInt();                
//            }
//            else {
//                state = p;
//            }
//            if constexpr(!std::is_same_v<Debug, void>) {
//                Debug::toggle();
//            }
//            on_full_state();
//        }
        
        
        //    private:
        inline static state_type state{0};
        inline static bool mReverse = false;
    };
    
}
