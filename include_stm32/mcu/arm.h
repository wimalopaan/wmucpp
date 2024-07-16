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
