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

#include <stdint.h>

#include "hal/event.h"
#include "appl/clockstatemachine.h"

template<typename DCFDecoder, typename StateEntry = void>
class RadioClock final {
    typedef ClockStateMachine::Machine<StateEntry> clockFSM;
    
    struct DCFReceive0Handler : public EventHandler<EventType::DCFReceive0> {
        static bool process(uint8_t) {
            return true;
        }  
    };
    struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
        static bool process(uint8_t) {
            return true;
        }  
    };
    struct DCFDecodeHandler : public EventHandler<EventType::DCFDecode> {
        static bool process(uint8_t) {
            clockFSM::process(ClockStateMachine::Event::DCFDecode);
            EventManager::enqueue({EventType::RadioClockReady, 0});
//            oscillator::referenceTime(dcfDecoder::dateTime());
//            std::cout << "Delta: "_pgm << oscillator::delta() << std::endl;
//            std::cout << "Durat: "_pgm << oscillator::lastMeasurementDuration().value << std::endl;
//            if (auto delta = oscillator::delta(); delta != 0) {
//                if (delta > 0) {
//                    oscillator::adjust(-1);
//                }
//                else {
//                    oscillator::adjust(1);
//                }
//            }
//            std::cout << "OSCCAL: "_pgm << oscillator::calibration() << std::endl;
            return true;
        }  
    };
    struct DCFSyncHandler : public EventHandler<EventType::DCFSync> {
        static bool process(uint8_t) {
            clockFSM::process(ClockStateMachine::Event::DCFSync);
            return true;
        }  
    };
    struct DCFErrorHandler : public EventHandler<EventType::DCFError> {
        static bool process(uint8_t) {
            clockFSM::process(ClockStateMachine::Event::DCFError);
            return true;
        }  
    };
    struct DCFParityHandler : public EventHandler<EventType::DCFParityError> {
        static bool process(uint8_t) {
            clockFSM::process(ClockStateMachine::Event::DCFError);
            return true;
        }  
    };
    struct ResetHandler : public EventHandler<EventType::RadioClockReset> {
        static bool process(uint8_t) {
            clockFSM::process(ClockStateMachine::Event::Reset);
            return true;
        }  
    };
    struct StartHandler : public EventHandler<EventType::RadioClockStart> {
        static bool process(uint8_t) {
            clockFSM::process(ClockStateMachine::Event::Start);
            return true;
        }  
    };
public:
    class HandlerGroup final : public EventHandlerGroup<
            DCFReceive0Handler, DCFReceive1Handler, DCFSyncHandler, DCFDecodeHandler,
            DCFErrorHandler, DCFParityHandler,
            ResetHandler, StartHandler> {
    };
    static void init() {
        DCFDecoder::init();
    }
private:
};

