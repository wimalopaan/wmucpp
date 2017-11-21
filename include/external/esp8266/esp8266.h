/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>

#include "container/boundedbuffer.h"
#include "hal/event.h"
#include "util/bcd.h"

// esptool.py -p /dev/ttyUSB0 write_flash 0xfe000 ../blank.bin 0xfc000 ../esp_init_data_default.bin 0x00000 ../boot_v1.2.bin 0x01000 512+512/user1.1024.new.2.bin  0x7e000 ../blank.bin  0x81000 512+512/user2.1024.new.2.bin

namespace Esp8266 {

enum class Command : uint8_t {Reset, Ok, Version, StationMode, ListAP, ListAPOpt, AddressInfo, JoinAP1, 
                              NistTimeS, NistTimeM,
                              Mux, Single, Status};

template<uint8_t UsartNumber>
class Interface final {
    Interface() = delete;
    
    enum class State : uint8_t {Undefined, Ready, Busy};
    
public:
    using usart = AVR::Usart<UsartNumber, ATProtocollAdapter<UsartNumber, 64>>;
    typedef typename usart::protocoll_adapter_type protocoll_adapter_type;
    
    static void periodic() {
        if (!mCommandQueue.empty()) {
            if (mState == State::Ready) {
                if (auto c = mCommandQueue.pop_front()) {
                    send(*c);
                    mState = State::Busy;
                }
            }
        }
    }
    static void init() {
        usart::template init<115200>();
    }
    static inline bool put(Command command) {
        return mCommandQueue.push_back(command);
    }
    
    static inline void send(Command command) {
        switch (command) {
        case Command::Reset:
            putLine(ATReset);
            break;
        case Command::Ok:
            putLine(ATOk);
            break;
        case Command::Version:
            putLine(ATVersion);
            break;
        case Command::StationMode:
            putLine(ATStationMode);
            break;
        case Command::ListAPOpt:
            putLine(ATListOpt);
            break;
        case Command::ListAP:
            putLine(ATList);
            break;
        case Command::AddressInfo:
            putLine(ATAddressInfo);
            break;
        case Command::JoinAP1:
            putLine(ATJoinAP1);
            break;
        case Command::NistTimeS:
            putLine(ATNistTimeQueryS);
            break;
        case Command::NistTimeM:
            putLine(ATNistTimeQueryM);
            break;
        case Command::Mux:
            putLine(ATMux);
            break;
        case Command::Single:
            putLine(ATSingle);
            break;
        case Command::Status:
            putLine(ATStatus);
            break;
        }
    }
    static inline const volatile auto& data() {
        return protocoll_adapter_type::data();
    }

private:
    template<typename String>
    static void putLine(const String& string) {
        Util::put<usart>(string);
        Util::put<usart>('\r');
        Util::put<usart>('\n');
    }
    static inline auto ATOk      = "AT"_pgm;
    static inline auto ATReset   = "AT+RST"_pgm;
    static inline auto ATVersion = "AT+GMR"_pgm;
    static inline auto ATStationMode = "AT+CWMODE=1"_pgm;
    static inline auto ATListOpt = "AT+CWLAPOPT=1,2"_pgm;
    static inline auto ATList = "AT+CWLAP"_pgm;
    static inline auto ATStatus = "AT+CIPSTATUS"_pgm;
    static inline auto ATMux = "AT+CIPMUX=1"_pgm;
    static inline auto ATSingle= "AT+CIPMUX=0"_pgm;
    static inline auto ATAddressInfo = "AT+CIFSR"_pgm;
    static inline auto ATJoinAP1 = "AT+CWJAP=\"FRITZ!Box 7490\",\"08674464592570801814\""_pgm; // AT+CWJAP="FRITZ!Box 7490","08674464592570801814"
    static inline auto ATNistTimeQueryM = "AT+CIPSTART=4,\"TCP\",\"time.nist.gov\",37"_pgm; // AT+CIPSTART=4,"TCP","time.nist.gov",37
    static inline auto ATNistTimeQueryS = "AT+CIPSTART=\"TCP\",\"time.nist.gov\",37"_pgm;
    
    static inline std::FiFo<Command, 16> mCommandQueue;
    static inline State mState = State::Undefined;
};

template<uint8_t N, uint8_t Size = 64>
class ATProtocollAdapter final {
    ATProtocollAdapter() = delete;

    enum class State : uint8_t {Undefined = 0, Initial = 1, Ok1, Ok2, ReadyR, ReadyE, ReadyA, ReadyD, ReadyY, LineEnd, Error1, 
                               CommandA, CommandT, Response1, ResponseC, ResponseCData, ResponseI, ResponseIF1, ResponseIF2, ResponseIData, Data};
public:
    static constexpr uint8_t size = Size;
    
    inline static bool process(std::byte b) { // from isr only
        auto c = std::to_integer<uint8_t>(b);
        switch (mState) {
        case State::Undefined:
            if (c == 'O') {
                mState = State::Ok1;
            }
            else if (c == 'r') {
                mState = State::ReadyR;
            }
            else if (c == '\r') {
                mState = State::LineEnd;
            }
            break;
        case State::ReadyR:
            if (c == 'e') {
                mState = State::ReadyE;
            }
            break;
        case State::ReadyE:
            if (c == 'a') {
                mState = State::ReadyA;
            }
            break;
        case State::ReadyA:
            if (c == 'd') {
                mState = State::ReadyD;
            }
            break;
        case State::ReadyD:
            if (c == 'y') {
                mState = State::ReadyY;
            }
            break;
        case State::ReadyY:
            if (c == '\r') {
                mState = State::LineEnd;
            }
            break;
        case State::Initial:
            if (c == 'O') {
                mState = State::Ok1;
            }
            else if (c == 'A') {
                mState = State::CommandA;
            }
            else if (c == '+') {
                mState = State::Response1;
            }
            else if (c == 'E') {
                mState = State::Error1;
            }
            else if (c == '\r') {
                mState = State::LineEnd;
            }
            break;
        case State::Error1:
//            EventManager::enqueueISR(EventType::Esp_Error);
            if (c == '\r') {
                mState = State::LineEnd;
            }
            break;
        case State::Response1:
            if (c == 'C') {
                mState = State::ResponseC;
            }
            else if (c == 'I') {
                mState = State::ResponseI;
            }
            break;
        case State::ResponseC:
            if (c == ':') {
                mState = State::ResponseCData;
                mData.clear();
            }
            else if (c == '\r') {
                mState = State::LineEnd;
            }
            break;
        case State::ResponseCData:
            if ((c >= ' ') && (c <= 'z')) {
                mData.insert(c);
            }
            else if (c == '\r') {
//                EventManager::enqueueISR(EventType::Esp_CData);
                mState = State::LineEnd;
            }
            break;
        case State::ResponseI:
            if (c == ',') {
                mState = State::ResponseIF1;
                mData.clear();
            }
            break;
        case State::ResponseIF1:
            if (c == ',') {
                mState = State::ResponseIF2;
                mData.clear();
                uint8_t connection = BCD::uconvert<uint8_t>(mData);
            }
            else if (c == ':') {
                mPayLoadSize = BCD::uconvert<uint8_t>(mData);
                mData.clear();
                mState = State::ResponseIData;
            }
            else if ((c >= '0') && (c <= '9')) {
                mData.insert(c);
            }
            break;
        case State::ResponseIF2:
            if (c == ':') {
                mPayLoadSize = BCD::uconvert<uint8_t>(mData);
                mData.clear();
                mState = State::ResponseIData;
            }
            else if ((c >= '0') && (c <= '9')) {
                mData.insert(c);
            }
            break;
        case State::ResponseIData:
            mData.insert(c);
            if (--mPayLoadSize <= 0) {
//                EventManager::enqueueISR(EventType::Esp_IData);
                mState = State::Initial;
            }
            break;
        case State::CommandA:
            if (c == '\r') {
                mState = State::LineEnd;
            }
            else if (c == 'T') {
                mState = State::CommandT;
            }
            break;
        case State::CommandT:
            if (c == '\r') {
                mState = State::LineEnd;
            }
            break;
        case State::Ok1:
            if (c == 'K') {
                mState = State::Ok2;
            }
            else if (c == '\r') {
                mState = State::LineEnd;
            }
            break;
        case State::Ok2:
            if (c == '\r') {
                mState = State::LineEnd;
//                EventManager::enqueueISR(EventType::Esp_OK);
            }
            break;
        case State::LineEnd:
            if (c == '\n') {
                mState = State::Initial;
            }
            break;
        default:
            break;
        }
        return true;
    }    
    static inline State state() {
        return mState;
    }
    static inline const auto& data() {
        return mData;
    }
private:
    volatile inline static State mState = State::Undefined;
    volatile inline static BoundedBuffer<uint8_t, Size> mData;
    volatile inline static uint16_t mPayLoadSize = 0;
};

}