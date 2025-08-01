#pragma once

#include <type_traits>
#include <concepts>

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"
#include "rcc.h"
#include "atomic.h"
#include "exti.h"
#include "output.h"
#include "timer.h"

template<typename Config, typename MCU = DefaultMcu>
struct ExtiSlave {
    using pin = Config::pin;
    using debug = Config::debug;
    using slaves = Config::slaves;
    using inits = Config::init;

    using gpio = pin::gpio_t;
    using port = gpio::port_t;

    static inline constexpr uint8_t N = pin::number;
    static inline constexpr uint8_t exti_n = pin::number / 4;
    static inline constexpr uint8_t exti_s = pin::number % 4;

    static inline constexpr uint8_t map = Mcu::Stm::ExtI::Map<port>::value;

    static inline void init() {
        IO::outl<debug>("# Exti Slv: ", N, ", ", exti_n, ", ", exti_s, ", ", map);
        if constexpr(exti_n < 4) {
            EXTI->EXTICR[exti_n] = map << (exti_s * 8);
        }
        else {
            static_assert(false);
        }
        EXTI->RTSR1 |= (0x01 << N);
        EXTI->FTSR1 |= (0x01 << N);
        EXTI->IMR1  |= (0x01 << N);

        if constexpr(!std::is_same_v<inits, void>) {
            inits::start();
        }

        Meta::visit<slaves>([]<typename S>(Meta::Wrapper<S>){
                                S::template dir<Mcu::Output>();
                            });
    }
    static inline void reset() {
        EXTI->RTSR1 &= ~(0x01 << N);
        EXTI->FTSR1 &= ~(0x01 << N);
        EXTI->IMR1  &= ~(0x01 << N);
        Meta::visit<slaves>([]<typename S>(Meta::Wrapper<S>){
                                // S::analog();
                            });
    }
    struct Isr {
        static inline void edge() {
            if (EXTI->RPR1 & (0x01 << N)) {
                Meta::visit<slaves>([]<typename S>(Meta::Wrapper<S>){
                                        S::set();
                                    });
                EXTI->RPR1 = (0x01 << N);
            }
            else if (EXTI->FPR1 & (0x01 << N)) {
                Meta::visit<slaves>([]<typename S>(Meta::Wrapper<S>){
                                        S::reset();
                                    });
                EXTI->FPR1 = (0x01 << N);
            }
        }
    };
    private:
};
