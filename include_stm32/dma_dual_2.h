/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "units.h"
#include "concepts.h"
#include "components.h"
#include "dma_2.h"

namespace Mcu::Stm::Dma {
    namespace V2 {

        template<typename Ch_RW, typename Ch_R, typename Ch_W, typename Config>
        struct DualChannel;

        template<typename Ch_RW, typename Config>
        struct DualChannel<Ch_RW, void, void, Config> {
            using value_t = Config::value_t;
            struct dmaChConfig;
            using dmaChRW = Mcu::Stm::Dma::V2::Channel<Ch_RW::number_t::value, dmaChConfig>;
            struct dmaChConfig {
                using debug = Config::debug;
                using controller = Mcu::Stm::Dma::Controller<Ch_RW::controller::number_t::value>;
                using value_t = DualChannel::value_t;
                static inline constexpr bool memoryIncrement = true;
            };
            static inline void init() {
                dmaChRW::init();
            }
            static inline void reset() {
                dmaChRW::reset();
            }
            static inline void startRead(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                dmaChRW::startRead(size, pAdr, mAdr, mux);
            }
            static inline void startWrite(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                dmaChRW::startWrite(size, pAdr, mAdr, mux);
            }
            static inline uint32_t counter() {
                return dmaChRW::counter();
            }
            static inline void memoryAddress(volatile value_t* adr) {
                dmaChRW::memoryAddress(adr);
            }
            static inline volatile value_t* memoryAddress() {
                return dmaChRW::memoryAddress();
            }
            // static inline void enable(const bool on = true) {

            // }
            static inline void reConfigure(const auto f) {
                dmaChRW::reConfigure(f);
            }
            static inline void size(const uint32_t s) {
                dmaChRW::size(s);
            }
        };
        template<typename Ch_R, typename Ch_W, typename Config>
        struct DualChannel<void, Ch_R, Ch_W, Config> {
            using value_t = Config::value_t;
            struct dmaRConfig {
                using debug = Config::debug;
                using controller = Mcu::Stm::Dma::Controller<Ch_R::controller::number_t::value>;
                using value_t = DualChannel::value_t;
                static inline constexpr bool memoryIncrement = true;
            };
            using dmaChR = Mcu::Stm::Dma::V2::Channel<Ch_R::number_t::value, dmaRConfig>;
            struct dmaWConfig {
                using debug = Config::debug;
                using controller = Mcu::Stm::Dma::Controller<Ch_W::controller::number_t::value>;
                using value_t = DualChannel::value_t;
                static inline constexpr bool memoryIncrement = true;
            };
            using dmaChW = Mcu::Stm::Dma::V2::Channel<Ch_W::number_t::value, dmaWConfig>;

            static inline void init() {
                dmaChR::init();
                dmaChW::init();
            }
            static inline void reset() {
                dmaChR::reset();
                dmaChW::reset();
            }
            static inline void startRead(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                dmaChR::startRead(size, pAdr, mAdr, mux);
            }
            static inline void startWrite(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                dmaChW::startWrite(size, pAdr, mAdr, mux);
            }
            template<bool Read = true>
            static inline uint32_t counter() {
                if constexpr(Read) {
                    return dmaChR::counter();
                }
                else {
                    return dmaChW::counter();
                }
            }
            template<bool Read = true>
            static inline void memoryAddress(volatile value_t* adr) {
                if constexpr(Read) {
                    dmaChR::memoryAddress(adr);
                }
                else {
                    dmaChW::memoryAddress(adr);
                }
            }
            template<bool Read = true>
            static inline volatile value_t* memoryAddress() {
                if constexpr(Read) {
                    return dmaChR::memoryAddress();
                }
                else {
                    return dmaChW::memoryAddress();
                }
            }
            // static inline void enable(const bool on = true) {
            // }
            template<bool Read = true>
            static inline void reConfigure(const auto f) {
                if constexpr(Read) {
                    return dmaChR::reConfigure(f);
                }
                else {
                    return dmaChW::reConfigure(f);
                }
            }
            template<bool Read = true>
            static inline void size(const uint32_t s) {
                if constexpr(Read) {
                    return dmaChR::size(s);
                }
                else {
                    return dmaChW::size(s);
                }
            }
        };

        template<typename Ch_W, typename Config>
        struct DualChannel<void, void, Ch_W, Config> {
            using value_t = Config::value_t;
            struct dmaWConfig {
                using debug = Config::debug;
                using controller = Mcu::Stm::Dma::Controller<Ch_W::controller::number_t::value>;
                using value_t = DualChannel::value_t;
                static inline constexpr bool memoryIncrement = true;
                struct Isr {
                    static inline constexpr bool txComplete = Config::Isr::txComplete;
                };
            };
            using dmaChW = Mcu::Stm::Dma::V2::Channel<Ch_W::number_t::value, dmaWConfig>;

            static inline constexpr uint8_t number = dmaChW::number;

            static inline void init() {
                dmaChW::init();
            }
            static inline void reset() {
                dmaChW::reset();
            }
            static inline void startWrite(const size_t size, const uint32_t pAdr, volatile value_t* mAdr, uint8_t mux) {
                dmaChW::startWrite(size, pAdr, mAdr, mux);
            }
            static inline uint32_t counter() {
                return dmaChW::counter();
            }
            static inline void memoryAddress(volatile value_t* adr) {
                dmaChW::memoryAddress(adr);
            }
            static inline volatile value_t* memoryAddress() {
                return dmaChW::memoryAddress();
            }
            static inline void reConfigure(const auto f) {
                return dmaChW::reConfigure(f);
            }
            static inline void size(const uint32_t s) {
                return dmaChW::size(s);
            }
            struct Isr {
                static inline void onTransferComplete(const auto f) {
                    dmaChW::onTransferComplete(f);
                }
            };

        };

    }

}
