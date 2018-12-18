#pragma once

#include "../common/register.h"

#include <array>

namespace AVR {
    
    template<typename BitsType>
    struct PrescalerPair {
        typedef BitsType  bits_type;
        typedef uint16_t scale_type;
        const BitsType  bits;
        const uint16_t scale;
    };
    
    template<typename CS>
    constexpr std::array<PrescalerPair<CS>, 4> twiPrescalerBit = {
        PrescalerPair<CS>{CS::twps1 | CS::twps0, 64},
        PrescalerPair<CS>{CS::twps1           , 16},
        PrescalerPair<CS>{            CS::twps0,  4},
        PrescalerPair<CS>{static_cast<CS>(0) ,  1},
    };
    
    
    template<typename CS>
    constexpr std::array<PrescalerPair<CS>, 6> prescalerValues10Bit = {
        PrescalerPair<CS>{CS::cs2          | CS::cs0, 1024},
        PrescalerPair<CS>{CS::cs2                   , 256},
        PrescalerPair<CS>{         CS::cs1 | CS::cs0, 64},
        PrescalerPair<CS>{         CS::cs1          , 8},
        PrescalerPair<CS>{                   CS::cs0, 1},
        PrescalerPair<CS>{static_cast<CS>(0)        , 0},
    };
    
    template<typename CS>
    constexpr CS csMask10Bit = {CS::cs2 | CS::cs1 | CS::cs0};
    
    template<typename CS>
    constexpr std::array<PrescalerPair<CS>, 8> prescalerValues10BitExtended = {
        PrescalerPair<CS>{CS::cs2 | CS::cs1 | CS::cs0, 1024},
        PrescalerPair<CS>{CS::cs2 | CS::cs1          , 256},
        PrescalerPair<CS>{CS::cs2           | CS::cs0, 128},
        PrescalerPair<CS>{CS::cs2                    , 64},
        PrescalerPair<CS>{          CS::cs1 | CS::cs0, 32},
        PrescalerPair<CS>{          CS::cs1          , 8},
        PrescalerPair<CS>{                    CS::cs0, 1},
        PrescalerPair<CS>{static_cast<CS>(0)         , 0}
    };
    template<typename CS>
    constexpr std::array<PrescalerPair<CS>, 16> prescalerValues14Bit = {
        PrescalerPair<CS>{CS::cs3 | CS::cs2 | CS::cs1 | CS::cs0, 16384},
        PrescalerPair<CS>{CS::cs3 | CS::cs2 | CS::cs1          , 8192},
        PrescalerPair<CS>{CS::cs3 | CS::cs2 |           CS::cs0, 4096},
        PrescalerPair<CS>{CS::cs3 | CS::cs2                    , 2048},
        PrescalerPair<CS>{CS::cs3 |           CS::cs1 | CS::cs0, 1024},
        PrescalerPair<CS>{CS::cs3 |           CS::cs1          , 512},
        PrescalerPair<CS>{CS::cs3 |                     CS::cs0, 256},
        PrescalerPair<CS>{CS::cs3                              , 128},
        PrescalerPair<CS>{          CS::cs2 | CS::cs1 | CS::cs0, 64},
        PrescalerPair<CS>{          CS::cs2 | CS::cs1          , 32},
        PrescalerPair<CS>{          CS::cs2 |           CS::cs0, 16},
        PrescalerPair<CS>{          CS::cs2                    , 8},
        PrescalerPair<CS>{                    CS::cs1 | CS::cs0, 4},
        PrescalerPair<CS>{                    CS::cs1          , 2},
        PrescalerPair<CS>{                              CS::cs0, 1},
        PrescalerPair<CS>{static_cast<CS>(0)                   , 0}
    };
}

#if defined(__AVR_ATmega1284P__)
# include "atmega1284p.h"
#elif defined(__AVR_ATmega328P__)
# include "atmega328p.h"
#elif defined(__AVR_ATmega88P__)
# include "atmega88p.h"
#elif defined(__AVR_ATmega168P__)
# include "atmega168p.h"
#elif defined(__AVR_ATmega328PB__)
# include "atmega328pb.h"
#elif defined(__AVR_ATmega8__)
# include "tmega8.h"
#elif defined(__AVR_ATmega324PB__)
# include "atmega324pb.h"
#elif defined(__AVR_ATtiny85__)
# include "attiny85.h"
#elif defined(__AVR_ATtiny25__)
# include "attiny25.h"
#elif defined(__AVR_ATtiny84__)
# include "attiny84.h"
#else
typedef AVR::ATMegaNone DefaultMcuType;
#endif
