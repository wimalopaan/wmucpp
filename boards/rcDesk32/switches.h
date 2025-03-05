#pragma once

#include <cstring>

#include "meta.h"

#include "mcu/mcu.h"
#include "timer.h"
#include "concepts.h"
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
                                        if (++mActual >= numberOfPCAs) {
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
            for(auto& pca: mSwStates) {
                for(auto& sw : pca) {
                    sw = 1;
                }
            }
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
