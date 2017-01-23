/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdint.h>

#include "units/duration.h"
#include "std/literals.h"

#include "external/hott/sensorprotocoll.h"
#include "external/hott/sensorprotocolladapter.h"
#include "external/hott/sensorprotocollbuffer.h"
#include "external/hott/sensortextprotocollbuffer.h"
#include "external/hott/sumdprotocoll.h"
#include "external/hott/sumdprotocolladapter.h"

namespace Hott {

// todo: ersetzen

template<typename Usart>
class SensorProtocoll final {
public:
    SensorProtocoll() = delete;

//    static void hott_responseAscii(void) {
//        Util::delay(hottDelayBeforeAnswer);
//        Usart::template rxEnable<false>();

//        hottTextResponse.start_byte = 0x7b;
//        hottTextResponse.stop_byte = 0x7d;
//        hottTextResponse.esc = 0;

//        hottTextResponse.text[0].insertAtFill(0, "WM Sensor 0.1"_pgm);

//        hottTextResponse.text[1].insertAtFill(0, " Test2"_pgm);
//        hottTextResponse.text[2].insertAtFill(0, " Test3"_pgm);
//        hottTextResponse.text[3].insertAtFill(0, " Test4"_pgm);
//        hottTextResponse.text[4].insertAtFill(0, " Test5"_pgm);
//        hottTextResponse.text[5].insertAtFill(0, " Test6"_pgm);
//        hottTextResponse.text[6].insertAtFill(0, " Test7"_pgm);
//        hottTextResponse.text[7].insertAtFill(0, " Test8"_pgm);

//        hottTextResponse.text[mRow + 1][mColumn] =  '>';

//        Usart::put(Util::parity(hottTextResponse, [](uint8_t v){
//                       Usart::put(v);
//                       Util::delay(hottDelayBetweenBytes);
//                   }));

//        Usart::waitSendComplete();
//        Usart::template rxEnable<true>();
//    }
//    static void key(uint8_t k) {
//        switch (k) {
//        case keyDown:
//            mRow = (mRow + 1) % mNumberOfRows;
//            break;
//        case keyUp:
//            mRow = (mRow - 1 + mNumberOfRows) % mNumberOfRows;
//            break;
//        case keyRight:
//            mColumn = (mColumn + 1) % mNumberOfColumns;
//            break;
//        case keyLeft:
//            mColumn = (mColumn - 1 + mNumberOfColumns) % mNumberOfColumns;
//            break;
//        default:
//            break;
//        }
//        mKey = k;
//    }

private:
    static constexpr const uint8_t mNumberOfRows = 7;
    static constexpr const uint8_t mNumberOfColumns = 2;

    static uint8_t mRow;
    static uint8_t mColumn;
    static uint8_t mKey;

    static GamMsg hottBinaryResponse;
    static TextMsg hottTextResponse;
};
template<typename U>
uint8_t SensorProtocoll<U>::mKey = 0;
template<typename U>
uint8_t SensorProtocoll<U>::mRow = 0;
template<typename U>
uint8_t SensorProtocoll<U>::mColumn = 0;
template<typename U>
GamMsg SensorProtocoll<U>::hottBinaryResponse = {};
template<typename U>
TextMsg SensorProtocoll<U>::hottTextResponse = {};

//class HottSwitchDecoder {
//public:
//    static void decode(uint8_t) {

//    }
//};





}
