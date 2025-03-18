#pragma once

#include <etl/meta.h>

template<typename List> struct Leds;
template<typename... L>
struct Leds<Meta::List<L...>> {
    using list = Meta::List<L...>;
    static inline void init() {
        (L::template dir<AVR::Output>(), ...);
    }
    static inline void set(const uint8_t led, const bool on) {
        Meta::visitAt<list>(led, [&]<typename LL>(Meta::Wrapper<LL>){
                                   LL::onOff(on);
                               });
    }
};

