/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstring>

#include "meta.h"

#include "mcu/mcu.h"
#include "timer.h"
#include "concepts.h"
#include "tick.h"

namespace External {
    template<typename Config>
    struct Switches {
        using pcas = Config::pcas;
        using timer = Config::timer;
        using debug = Config::debug;
        using callback = Config::callback;

        static inline constexpr uint8_t numberOfPCAs = Meta::size_v<pcas>;

        static inline constexpr External::Tick<timer> switchesTicks{20ms};

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
            Meta::visit<pcas>([]<typename P>(Meta::Wrapper<P>) static {
                                  P::init();
                              });
            for(auto& pca: mSwStates) {
                for(auto& sw : pca) {
                    sw = 1;
                }
            }
        }
        static inline void periodic() {
            Meta::visit<pcas>([]<typename P>(Meta::Wrapper<P>) static {
                                  P::periodic();
                              });
        }
        static inline void ratePeriodic() {
            Meta::visitAt<pcas>(mActual, []<typename P>(Meta::Wrapper<P>) static {
                P::ratePeriodic();
            });
            (++mStateTick).on(switchesTicks, [] static {
                                  startRead([](const uint8_t index, const uint8_t newState) static {
                                      IO::outl<debug>("# Switch1 ", index, " ", newState);
                                      set(index, newState);
                                  });

                              });
        }
        private:

        static inline void set(const uint8_t index, const uint8_t state) {
            if (index < 32) {
                const uint8_t wIndex1 = 2 * index;
                const uint8_t wIndex2 = 2 * index + 1;
                const bool on1 = (state == 0);
                const bool on2 = (state == 2);
                callback::set(wIndex1, on1);
                callback::set(wIndex2, on2);
            }
        }
        static inline uint8_t mActual{};
        static inline External::Tick<timer> mStateTick;
        static inline std::array<std::array<uint8_t, 8>, numberOfPCAs> mSwStates;
    };
}
