#pragma once

#include <cstdint>
#include <array>

namespace External::RC {
    enum class Band : uint8_t {_27MHz, _35AMHz, _35BMHz, _40MHz, _41MHz, _72MHz, _Test};
    
//    static inline constexpr uint8_t NumberOfBands = 7;
    
    struct Channel {
        const Band     mBand{};
        const uint16_t mNumber;
        const uint32_t mFreq{};
    };
    
    inline static constexpr std::array channels {
#ifdef RF_EXTRA
        Channel{Band::_Test,   500, 7'000'000 * 4}, // 
        Channel{Band::_Test,   501, 7'000'000}, // 
#endif
        Channel{Band::_27MHz,   4, 26'995'000},
        Channel{Band::_27MHz,   5, 27'005'000},
        Channel{Band::_27MHz,   6, 27'015'000},
        Channel{Band::_27MHz,   7, 27'025'000},
        Channel{Band::_27MHz,   8, 27'035'000},
        Channel{Band::_27MHz,   9, 27'045'000},
        Channel{Band::_27MHz,  10, 27'055'000},
        Channel{Band::_27MHz,  11, 27'065'000},
        Channel{Band::_27MHz,  12, 27'075'000},
        Channel{Band::_27MHz,  13, 27'085'000},
        Channel{Band::_27MHz,  14, 27'095'000}, // 10
        Channel{Band::_27MHz,  15, 27'105'000},
        Channel{Band::_27MHz,  16, 27'115'000},
        Channel{Band::_27MHz,  17, 27'125'000},
        Channel{Band::_27MHz,  18, 27'135'000},
        Channel{Band::_27MHz,  19, 27'145'000},
        Channel{Band::_27MHz,  24, 27'195'000},
        Channel{Band::_27MHz,  30, 27'255'000},
        Channel{Band::_35AMHz, 61, 35'010'000},
        Channel{Band::_35AMHz, 62, 35'020'000},
        Channel{Band::_35AMHz, 63, 35'030'000}, // 20
        Channel{Band::_35AMHz, 64, 35'040'000},
        Channel{Band::_35AMHz, 65, 35'050'000},
        Channel{Band::_35AMHz, 66, 35'060'000},
        Channel{Band::_35AMHz, 67, 35'070'000},
        Channel{Band::_35AMHz, 68, 35'080'000},
        Channel{Band::_35AMHz, 69, 35'090'000},
        Channel{Band::_35AMHz, 70, 35'100'000},
        Channel{Band::_35AMHz, 71, 35'110'000},
        Channel{Band::_35AMHz, 72, 35'120'000},
        Channel{Band::_35AMHz, 73, 35'130'000}, // 30
        Channel{Band::_35AMHz, 74, 35'140'000},
        Channel{Band::_35AMHz, 75, 35'150'000},
        Channel{Band::_35AMHz, 76, 35'160'000},
        Channel{Band::_35AMHz, 77, 35'170'000},
        Channel{Band::_35AMHz, 78, 35'180'000},
        Channel{Band::_35AMHz, 79, 35'190'000},
        Channel{Band::_35AMHz, 80, 35'200'000},
        Channel{Band::_35BMHz, 182, 35'820'000},
        Channel{Band::_35BMHz, 183, 35'830'000},
        Channel{Band::_35BMHz, 184, 35'840'000}, // 40
        Channel{Band::_35BMHz, 185, 35'850'000},
        Channel{Band::_35BMHz, 186, 35'860'000},
        Channel{Band::_35BMHz, 187, 35'870'000},
        Channel{Band::_35BMHz, 188, 35'880'000},
        Channel{Band::_35BMHz, 189, 35'890'000},
        Channel{Band::_35BMHz, 190, 35'900'000},
        Channel{Band::_35BMHz, 191, 35'910'000},
        Channel{Band::_40MHz, 50, 40'665'000},
        Channel{Band::_40MHz, 51, 40'675'000},
        Channel{Band::_40MHz, 52, 40'685'000}, // 50
        Channel{Band::_40MHz, 53, 40'695'000},
        Channel{Band::_40MHz, 54, 40'715'000},
        Channel{Band::_40MHz, 55, 40'725'000},
        Channel{Band::_40MHz, 56, 40'735'000},
        Channel{Band::_40MHz, 57, 40'765'000},
        Channel{Band::_40MHz, 58, 40'775'000}, // 56
        Channel{Band::_40MHz, 59, 40'785'000},
        Channel{Band::_40MHz, 81, 40'815'000}, // 58
        Channel{Band::_40MHz, 82, 40'825'000},
        Channel{Band::_40MHz, 83, 40'835'000}, // 60
        Channel{Band::_40MHz, 84, 40'865'000},
        Channel{Band::_40MHz, 85, 40'875'000},
        Channel{Band::_40MHz, 86, 40'885'000},
        Channel{Band::_40MHz, 87, 40'915'000}, // 64
        Channel{Band::_40MHz, 88, 40'925'000},
        Channel{Band::_40MHz, 89, 40'935'000},
        Channel{Band::_40MHz, 90, 40'965'000},
        Channel{Band::_40MHz, 91, 40'975'000},
        Channel{Band::_40MHz, 92, 40'985'000},
        Channel{Band::_41MHz, 400, 41'000'000},
        Channel{Band::_41MHz, 401, 41'010'000},
        Channel{Band::_72MHz, 200, 72'000'000},
        Channel{Band::_72MHz, 202, 72'020'000},
        Channel{Band::_72MHz, 204, 72'040'000},
    };
    
    constexpr uint16_t minChannelNumber = []{
        uint16_t min = channels[0].mNumber;
        for(const auto& c: RC::channels) {
            if (c.mNumber < min) min = c.mNumber;
        }
        return min;
    }();

    constexpr uint16_t maxChannelNumber = []{
        uint16_t max = channels[0].mNumber;
        for(const auto& c: RC::channels) {
            if (c.mNumber > max) max = c.mNumber;
        }
        return max;
    }();
    
    namespace Deprecated {
//        inline static constexpr std::array channels1 {
    //        Channel{Band::_27MHz,   1, 26'965'000},
    //        Channel{Band::_27MHz,   2, 26'975'000},
    //        Channel{Band::_35AMHz, 61, 35'010'000},
    //        Channel{Band::_35AMHz, 62, 35'020'000},
    //        Channel{Band::_35AMHz, 63, 35'030'000},
    //        Channel{Band::_35AMHz, 64, 35'040'000},
    //        Channel{Band::_35AMHz, 65, 35'050'000},
    //        Channel{Band::_35AMHz, 66, 35'060'000},
    //        Channel{Band::_35AMHz, 67, 35'070'000},
    //        Channel{Band::_35AMHz, 68, 35'080'000},
    //        Channel{Band::_35AMHz, 69, 35'090'000},
    //        Channel{Band::_35AMHz, 70, 35'100'000},
    //        Channel{Band::_35AMHz, 71, 35'110'000},
    //        Channel{Band::_35AMHz, 72, 35'120'000},
    //        Channel{Band::_35AMHz, 73, 35'130'000},
    //        Channel{Band::_35AMHz, 74, 35'140'000},
    //        Channel{Band::_35AMHz, 75, 35'150'000},
    //        Channel{Band::_35AMHz, 76, 35'160'000},
    //        Channel{Band::_35AMHz, 77, 35'170'000},
    //        Channel{Band::_35AMHz, 78, 35'180'000},
    //        Channel{Band::_35AMHz, 79, 35'190'000},
    //        Channel{Band::_35AMHz, 80, 35'200'000},
    //        Channel{Band::_35BMHz, 182, 35'820'000},
    //        Channel{Band::_35BMHz, 183, 35'830'000},
//            Channel{Band::_40MHz, 50, 40'665'000},
//            Channel{Band::_40MHz, 51, 40'675'000},
//            Channel{Band::_40MHz, 52, 40'685'000},
//            Channel{Band::_40MHz, 53, 40'695'000},
//            Channel{Band::_40MHz, 54, 40'715'000},
//            Channel{Band::_40MHz, 55, 40'725'000},
//            Channel{Band::_40MHz, 56, 40'735'000},
//            Channel{Band::_40MHz, 57, 40'765'000},
//            Channel{Band::_40MHz, 58, 40'778'000},
//            Channel{Band::_40MHz, 59, 40'785'000},
//            Channel{Band::_40MHz, 81, 40'815'000},
//            Channel{Band::_40MHz, 82, 40'825'000},
//            Channel{Band::_40MHz, 83, 40'835'000},
//            Channel{Band::_40MHz, 84, 40'865'000},
//            Channel{Band::_40MHz, 85, 40'875'000},
//            Channel{Band::_40MHz, 86, 40'885'000},
//            Channel{Band::_40MHz, 87, 40'915'000},
//            Channel{Band::_40MHz, 88, 40'925'000},
//            Channel{Band::_40MHz, 89, 40'935'000},
//            Channel{Band::_40MHz, 90, 40'965'000},
//            Channel{Band::_40MHz, 91, 40'975'000},
//            Channel{Band::_40MHz, 92, 40'985'000},
//            Channel{Band::_41MHz, 400, 41'000'000},
//            Channel{Band::_41MHz, 401, 41'010'000},
//        };
        
    }
    
}
