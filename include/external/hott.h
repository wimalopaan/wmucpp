/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "config.h"
#include "util/delay.h"
#include "hal/event.h"
#include "external/hottprotocoll.h"

namespace Hott {

constexpr const uint8_t keyRight = 14;
constexpr const uint8_t keyLeft  =  7;
constexpr const uint8_t keyUp    = 11;
constexpr const uint8_t keyDown  = 13;
constexpr const uint8_t keySet   =  9;

constexpr const auto hottDelay1 = 5000_us;
constexpr const auto hottDelay2 = 1600_us;

template<uint8_t M>
class SumDProtocollAdapter final {
    enum sumdstate {Undefined = 0, Start1, StartNormal, StartFailSafe, ChannelDataL, ChannelDataH, CrcL, CrcH, NumberOfStates};
    typedef enum sumdstate sumdstate_t;
    template<int N, typename PA> friend class AVR::Usart;

public:
    SumDProtocollAdapter() = delete;

    static uint16_t value(uint8_t channel) {
        Scoped<DisbaleInterrupt> di;
        return std::combinedValue(msg().channelData[channel]);
    }
    static uint8_t value8Bit(uint8_t channel) {
        return msg().channelData[channel].first;
    }
    static uint8_t numberOfChannels() {
        return msg().nChannels;
    }
private:
    inline static bool process(uint8_t c) { // from isr only
        static sumdstate state = sumdstate::Undefined;
        static uint8_t channel = 0;

        switch (state) {
        case sumdstate::Undefined:
            if (c == 0xa8) {
                state = sumdstate::Start1;
            }
            else {
                state = sumdstate::Undefined;
            }
            break;
        case sumdstate::Start1:
            if (c == 0x01) {
                state = sumdstate::StartNormal;
            }
            else if (c == 0x81) {
                state = sumdstate::StartFailSafe;
            }
            else {
                state = sumdstate::Undefined;
            }
            break;
        case sumdstate::StartNormal:
            msg().nChannels = c;
            state = sumdstate::ChannelDataH;
            break;
        case sumdstate::StartFailSafe:
            msg().nChannels = c;
            state = sumdstate::ChannelDataH;
            break;
        case sumdstate::ChannelDataH:
            msg().channelData[channel].first = c;
            state = sumdstate::ChannelDataL;
            break;
        case sumdstate::ChannelDataL:
            msg().channelData[channel].second = c;
            state = sumdstate::ChannelDataL;
            ++channel;
            if (channel < msg().nChannels) {
                state = sumdstate::ChannelDataH;
            }
            else {
                state = sumdstate::CrcH;
                channel = 0;
            }
            break;
        case sumdstate::CrcH:
            msg().crc = c << 8;
            state = sumdstate::CrcL;
            break;
        case sumdstate::CrcL:
            msg().crc |= c;
            state = sumdstate::Undefined;
            break;
        default:
            assert(false);
            break;
        }
        return true;
    }
    static volatile SumDMsg& msg() {
        static volatile SumDMsg data;
        return data;
    }
};

// todo: Aufteilen in upper/lower-half (flag -> periodic-checker)
template<uint8_t M>
class SensorProtocollAdapter final {
    enum class hottstate {Undefined = 0, Request1, RequestA1, NumberOfStates};
    template<int N, typename PA> friend class AVR::Usart;
public:
    SensorProtocollAdapter() = delete;
private:
    inline static bool process(uint8_t c) { // from isr only
        static hottstate state = hottstate::Undefined;
        switch (state) {
        case hottstate::Undefined:
            if (c == 0x80) {
                state = hottstate::Request1;
            }
            if (c == 0x7f) {
                state = hottstate::RequestA1;
            }
            break;
        case hottstate::RequestA1:
            if (c == 0x0f) {
                EventManager::enqueueISR({EventType::HottAsciiRequest, 0});
                state = hottstate::Undefined;
            }
            else {
                EventManager::enqueueISR({EventType::HottAsciiKey, c});
                state = hottstate::Undefined;
            }
            break;
        case hottstate::Request1:
            if (c == 0x8d) {
                EventManager::enqueueISR({EventType::HottBinaryRequest, 0});
                state = hottstate::Undefined;
            }
            else if (c == 0x80) {
                EventManager::enqueueISR({EventType::HottSensorBroadcast, 0});
                state = hottstate::Undefined;
            }
            else {
                state = hottstate::Undefined;
            }
            break;
        default:
            assert(false);
            break;
        }
        return true;
    }
};

template<typename Usart>
class SensorProtocoll final {
public:
    SensorProtocoll() = delete;

    static void hott_response() {
        Util::delay(hottDelay1);
        Usart::template rxEnable<false>();
        static GamMsg rb = {};
        rb.start_byte = 0x7c;
        rb.gam_sensor_id = 0x8d;
        rb.sensor_id = 0xd0;
        rb.cell[0] = 111;
        rb.cell[1] = 222;
        rb.cell[2] = 10;
        rb.rpm = 111;
        rb.rpm2 = 222;
        rb.stop_byte = 0x7d;
        uint8_t* p = (uint8_t*) &rb;
        rb.parity = 0;
        for(uint8_t i = 0; i < sizeof(rb) - 1; ++i) {
            rb.parity += *p;
            Usart::put(*p++);
            Util::delay(hottDelay2);
        }
        Usart::put(*p); // parity
        Usart::waitSendComplete();
        Usart::template rxEnable<true>();
    }

    static void hott_responseAscii(void) {
        Util::delay(hottDelay1);
        Usart::template rxEnable<false>();

        static TextMsg ra = {};
        ra.start_byte = 0x7b;
        ra.stop_byte = 0x7d;
        ra.esc = 0;

        ra.text[0].insertAtFill(0, "WM Sensor 0.1"_pgm);

        ra.text[1].insertAtFill(0, " Test2"_pgm);
        ra.text[2].insertAtFill(0, " Test3"_pgm);
        ra.text[3].insertAtFill(0, " Test4"_pgm);
        ra.text[4].insertAtFill(0, " Test5"_pgm);
        ra.text[5].insertAtFill(0, " Test6"_pgm);
        ra.text[6].insertAtFill(0, " Test7"_pgm);
        ra.text[7].insertAtFill(0, " Test8"_pgm);

        ra.text[mRow + 1][mColumn] =  '>';

        uint8_t* p = (uint8_t*) &ra;
        ra.parity = 0;
        for(uint8_t i = 0; i < sizeof(ra) - 1; ++i) {
            ra.parity += *p;
            Usart::put(*p++);
            Util::delay(hottDelay2);
        }
        Usart::put(*p); // parity
        Usart::waitSendComplete();
        Usart::template rxEnable<true>();
    }
    static void key(uint8_t k) {
        switch (k) {
        case keyDown:
            mRow = (mRow + 1) % mNumberOfRows;
            break;
        case keyUp:
            mRow = (mRow - 1 + mNumberOfRows) % mNumberOfRows;
            break;
        case keyRight:
            mColumn = (mColumn + 1) % mNumberOfColumns;
            break;
        case keyLeft:
            mColumn = (mColumn - 1 + mNumberOfColumns) % mNumberOfColumns;
            break;
        default:
            break;
        }
        mKey = k;
    }

private:
    static constexpr const uint8_t mNumberOfRows = 7;
    static constexpr const uint8_t mNumberOfColumns = 2;

    static uint8_t mRow;
    static uint8_t mColumn;
    static uint8_t mKey;
};
template<typename U>
uint8_t SensorProtocoll<U>::mKey = 0;
template<typename U>
uint8_t SensorProtocoll<U>::mRow = 0;
template<typename U>
uint8_t SensorProtocoll<U>::mColumn = 0;

class HottSwitchDecoder {
public:
    static void decode(uint8_t) {

    }
};





}
