#pragma once

#include <cstring>

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "clock.h"
#include "timer.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "gpio.h"
#include "tick.h"

namespace External {
    template<typename... PCAs>
    struct Switches {
        using pcas = Meta::List<PCAs...>;
        static inline constexpr uint8_t numberOfPCAs = sizeof...(PCAs);
        static inline void startRead(auto f) {
            Meta::visitAt<pcas>(mActual, [&]<typename P>(Meta::Wrapper<P>){
                                    if (P::isIdle()) {

                                        for(uint8_t i = 0; i < 8; ++i) {
                                            if (auto v = P::switchValue(i); v != mSwStates[mActual][i]) {
                                                mSwStates[mActual][i] = v;
                                                f(mActual * 8 + i, v);
                                            }
                                        }

                                        ++mActual;
                                        if (mActual >= numberOfPCAs) {
                                            mActual = 0;
                                        }
                                    }
                                });
            Meta::visitAt<pcas>(mActual, []<typename P>(Meta::Wrapper<P>){
                                       P::startRead();
                                });
        }

        static inline void init() {
            (PCAs::init(), ...);
        }
        static inline void periodic() {
            (PCAs::periodic(), ...);

        }
        static inline void ratePeriodic() {
            Meta::visitAt<pcas>(mActual, []<typename P>(Meta::Wrapper<P>){
                P::ratePeriodic();
            });
        }
        private:
        static inline uint8_t mActual{};
        static inline std::array<std::array<uint8_t, 8>, numberOfPCAs> mSwStates;
    };
}

