/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "units.h"
#include "etl/fifo.h"

using namespace Units::literals;

namespace Arm {

#ifdef TPI
#ifdef USE_MCU_STM_V3
    inline
#endif
    namespace V3 {
        template<typename Clock, Units::megahertz f = 2_MHz, size_t Size = 64>
        struct Trace {
            static inline void init() {
                TPI->ACPR = Clock::config::f / f - 1;
                if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
                    ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
                {
                    mActive = true;
                }
            }
            static inline bool isTxQueueEmpty() {
                return mData.empty();
            }
            static inline void put(const char c) {
                if (mActive) {
                    mData.push_back(c);
                }
            }
            static inline void periodic() {
                if (ITM->PORT[0].u32 != 0UL) {
                    if (const auto d = mData.pop_front()) {
                        ITM->PORT[0].u8 = *d;
                    }
                }
            }
            private:
            static inline bool mActive{};
            static inline etl::FiFo<char, Size> mData;
        };
    }

#ifdef USE_MCU_STM_V2
    inline
#endif
    namespace V2 {
        template<typename Clock, Units::megahertz f = 2_MHz, size_t Size = 64>
        struct Trace {
            static inline void init() {
                TPI->ACPR = Clock::config::f / f - 1;
                if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
                    ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
                {
                    mActive = true;
                }
            }
            static inline void put(const char c) {
                if (mActive) {
                    mData.push_back(c);
                }
            }
            static inline void periodic() {
                if (ITM->PORT[0].u32 != 0UL) {
                    if (const auto d = mData.pop_front()) {
                        ITM->PORT[0].u8 = *d;
                    }
                }
            }
            private:
            static inline bool mActive{};
            static inline etl::FiFo<char, Size> mData;
        };
    }
    
#ifdef USE_MCU_STM_V1
    inline
#endif
    namespace V1 {
        template<typename Clock, Units::megahertz f = 2_MHz>
        struct Trace {
            static inline void init() {
                TPI->ACPR = Clock::config::f / f - 1;
            }
            static inline void put(const char c) {
                ITM_SendChar(c);
            }
            private:
            static inline etl::FiFo<char, 128> mData;
        };
        
    }
#endif
}
