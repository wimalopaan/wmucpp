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

#define MEM
//#define NDEBUG

#include "rcsensorled01.h"
#include "console.h"
#include "util/meta.h"
#include "appl/blink.h"

namespace {
    constexpr bool useTerminal = true;
}

namespace Constants {
    static constexpr Color cOff{0};
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
    static constexpr auto title = "Test 08"_pgm;
}

using terminalDevice = std::conditional<useTerminal, SSpi0, void>::type;
using terminal = std::basic_ostream<terminalDevice>;

using statusLed = Blinker<led, Constants::cGreen>;

template<typename... T>
struct Distributor {
    using Items = Meta::filter<Meta::nonVoid, Meta::List<T...>>;
    template<typename U> struct NonVoidDistributor;
    template<template<typename...> typename L, typename... U>
    struct NonVoidDistributor<L<U...>> {
        inline static void init() {
            (U::init(), ...);
        }
    };
    inline static void init() {
        NonVoidDistributor<Items>::init();
    }
};

using sensorData = Hott::SensorProtocollBuffer<0>;
using menuData = Hott::SensorTextProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;

using isrRegistrar = IsrRegistrar<sensorUsart::RxHandler, sensorUsart::TxHandler>;

struct HottBinaryHandler : public EventHandler<EventType::HottBinaryRequest> {
    inline static bool process(std::byte) {
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        return true;
    }
};

struct HottKeyHandler : public EventHandler<EventType::HottAsciiKey> {
    inline static bool process(std::byte v) {
        std::outl<terminal>("key: "_pgm, v);
        return true;
    }
};

struct HottBroadcastHandler : public EventHandler<EventType::HottSensorBroadcast> {
    inline static bool process(std::byte) {
        std::outl<terminal>("hbr"_pgm);
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        return true;
    }
};

struct HottTextHandler : public EventHandler<EventType::HottAsciiRequest> {
    inline static bool process(std::byte) {
        std::outl<terminal>("hba"_pgm);
        crWriterSensorBinary::enable<false>();
        crWriterSensorText::enable<true>();
        return true;
    }
};
struct Usart0Handler : public EventHandler<EventType::UsartRecv0> {
    inline static bool process(std::byte) {
        statusLed::blink(Blue{32}, 2);
        return true;
    }
};
struct UsartFeHandler : public EventHandler<EventType::UsartFe> {
    inline static bool process(std::byte) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 3);
        return true;
    }
};
struct UsartUpeHandler : public EventHandler<EventType::UsartUpe> {
    inline static bool process(std::byte) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 4);
        return true;
    }
};
struct UsartDorHandler : public EventHandler<EventType::UsartDor> {
    inline static bool process(std::byte) {
        statusLed::blink(Color{Red{32}, Green{0}, Blue{32}}, 5);
        return true;
    }
};

std::array<OneWire::ow_rom_t, 5> dsIds;
const auto tempTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);
const auto measureTimer = alarmTimer::create(750_ms, AlarmFlags::Disabled | AlarmFlags::OneShot);

struct TempFSM {
    enum class State : uint8_t {Start, Wait, WaitRead};
    enum class Event : uint8_t {Measure, WaitOver, Read};
    static void process(Event e) {
        switch (mState) {
        case State::Start:
            if (e == Event::Measure) {
                ds18b20::convert();
                alarmTimer::start(*measureTimer);
                mState = State::Wait;
            }
            break;
        case State::Wait:
            if (e == Event::WaitOver) {
                ds18b20::startGet(dsIds[0]);
                mState = State::WaitRead;
            }            
            break;
        case State::WaitRead:
            if (e == Event::Read) {
                mState = State::Start;
            }
        }
    }

    inline static State mState = State::Start;
};

using tempFSM = TempFSM;


struct DS18B20MeasurementHandler: public EventHandler<EventType::DS18B20Measurement> {
    static bool process(std::byte) {
        auto t = ds18b20::temperature();
        sensorData::temp2(t);
        std::outl<terminal>("DS Temp: "_pgm, t);
        tempFSM::process(tempFSM::Event::Read);
        return true;
    }
};
struct DS18B20ErrorHandler: public EventHandler<EventType::DS18B20Error> {
    static bool process(std::byte) {
        std::outl<terminal>("DS Error"_pgm);
        tempFSM::process(tempFSM::Event::Read);
        return true;
    }
};




const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

struct TimerHandler : public EventHandler<EventType::Timer> {
    inline static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *periodicTimer) {
            ++mCounter;
            std::outl<terminal>("Test counter: "_pgm, mCounter, " Unused: "_pgm, Util::Memory::getUnusedMemory());
            uint8_t channel = mCounter % adcController::NumberOfChannels;
//            std::outl<terminal>("Adc "_pgm, channel, " :"_pgm, adcController::value(channel).toInt());
            statusLed::tick();
            return true;
        }
        else if (timer == *measureTimer) {
            tempFSM::process(tempFSM::Event::WaitOver);
        }
        else if (timer == *tempTimer) {
            tempFSM::process(tempFSM::Event::Measure);
        }
        return false;
    }
    inline static uint8_t mCounter = 0;
};

using distributor = Distributor<terminalDevice, isrRegistrar, statusLed, crWriterSensorBinary, crWriterSensorText, adcController, ds18b20>;

void updateMeasurements() {
    constexpr uint8_t hottScale = adcController::mcu_adc_type::VRef / 0.02;
    
    constexpr auto rawMax = adcController::value_type::Upper;
    
    uint16_t v1 = adcController::value(1) * 4;
    uint16_t v2 = adcController::value(2) * 8;
    uint16_t v3 = adcController::value(0) * 12;
    
    uint16_t z1 = (v1 * hottScale) / rawMax;
    uint16_t z2 = ((v2 - v1) * hottScale) / rawMax;
    uint16_t z3 = ((v3 - v2) * hottScale) / rawMax;
    
    sensorData::cellVoltageRaw(0, z1);
    sensorData::cellVoltageRaw(1, z2);
    sensorData::cellVoltageRaw(2, z3);

    constexpr uint8_t battScale = adcController::mcu_adc_type::VRef / 0.1;
    
    uint16_t b1 = (v3 * battScale) / rawMax;
    sensorData::batteryVoltageRaw(0, b1);
    
    constexpr uint8_t tempScale = adcController::mcu_adc_type::VRef / 0.314;
    
    uint8_t t1 = (adcController::value(3) * tempScale * 25) / rawMax + 20;
    sensorData::temperatureRaw(0, t1);
}

int main() {
    distributor::init();
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    constexpr auto tsd = AVR::Util::calculate<sensorRateTimer>(fCr);
    static_assert(tsd, "wrong parameter");
    sensorRateTimer::prescale<tsd.prescaler>();
    sensorRateTimer::ocra<tsd.ocr>();
    sensorRateTimer::mode(AVR::TimerMode::CTCNoInt);
    
    sensorUsart::init<19200>();

    using allEventHandler = EventHandlerGroup<
    TimerHandler, UsartFeHandler, UsartUpeHandler, UsartDorHandler, Usart0Handler, 
    HottBinaryHandler, HottBroadcastHandler, HottTextHandler, HottKeyHandler,
    ds18b20, DS18B20ErrorHandler, DS18B20MeasurementHandler>;

    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>(Constants::title);

        constexpr uint8_t hottScale = adcController::mcu_adc_type::VRef / 0.02;
        
        std::outl<terminal>("hott scale :"_pgm, hottScale);

        oneWireMaster::findDevices(dsIds, ds18b20::family);
        for(const auto& id : dsIds) {
            std::outl<terminal>(id);
        }

        
        EventManager::run2<allEventHandler>([](){
            adcController::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                alarmTimer::periodic();
            });
            
            updateMeasurements();
            
            sensorRateTimer::periodic<sensorRateTimer::flags_type::ocfa>([]() {
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                oneWireMasterAsync::rateProcess();
            });
            if (EventManager::unprocessedEvent()) {
                statusLed::blink(Constants::cRed, 10);
            }
            if (EventManager::leakedEvent()) {
                statusLed::blink(Constants::cBlue, 10);
            }
            
        });
    }    
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif

// Timer 2
// SystemClock
//ISR(TIMER2_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<2>::CompareA>();
//}
// Timer 4
// Sensor Constant Rate
//ISR(TIMER4_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<4>::CompareA>();
//}
// Sensor
ISR(USART0_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}
