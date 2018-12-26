/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

//#define USE_TC1_AS_HARDPPM // no RPM1 (ICP1) -> PPM3/4
#define USE_TC3_AS_HARDPPM // no RPM2 (ICP3) -> PPM1/2

#define MEM
#define NDEBUG
#define OUTPUT

#include "local.h"
#include "rcsensorled10.h"
#include "console.h"

#include <vector>


// todo: mit ifdef
using testPin0 = AVR::Pin<PortA, 5>; // ACS1
using testPin1 = AVR::Pin<PortA, 6>; // ACS2
using testPin2 = AVR::Pin<PortA, 7>; // ACS3

namespace Constants {
    static constexpr std::hertz pwmFrequency = 100_Hz * 256; 
    static constexpr const std::hertz fSCL = 100000_Hz;
}

struct AsciiHandler;
struct BinaryHandler;
struct BCastHandler;

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0, UseEvents<false>, AsciiHandler, BinaryHandler, BCastHandler>, 
MCU::UseInterrupts<true>, UseEvents<false>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>, MCU::UseInterrupts<true>, UseEvents<false>>;

using terminalDevice = rcUsart;
using terminal = std::basic_ostream<terminalDevice>;

using namespace std::literals::quantity;

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler
, gpsUsart::RxHandler, gpsUsart::TxHandler
, sensorUsart::RxHandler, sensorUsart::TxHandler
>;

using sensorData = Hott::SensorProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using menuData = Hott::SensorTextProtocollBuffer<0>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;

struct AsciiHandler {
    static void start() {
        crWriterSensorText::enable<true>();
    }    
    static void stop() {
        // not: das disable sollte automatisch laufen
        //        crWriterSensorText::enable<false>();
    }    
    static void process(std::byte ) {
    }
};
struct BinaryHandler {
    static void start() {
        crWriterSensorBinary::enable<true>();
    }    
    static void stop() {
        // not: das disable sollte automatisch laufen
        //        crWriterSensorBinary::enable<false>();
    }    
};
struct BCastHandler {
    static void start() {
#ifdef OUTPUT
        std::outl<terminal>("hbr start"_pgm);
#endif
        crWriterSensorBinary::enable<true>();
    }    
    static void stop() {
    }    
};

struct Storage {
    inline static StringBuffer<GPS::Sentence::TimeMaxWidth> time;
    inline static StringBuffer<GPS::Sentence::DecimalMaxWidth> decimalBuffer;
    
    inline static constexpr uint8_t NumberOfOWireDevs = 4;
    inline static std::vector<OneWire::ow_rom_t, NumberOfOWireDevs> dsIds;
    inline static std::array<FixedPoint<int, 4>, NumberOfOWireDevs> temps;
    
    inline static constexpr uint8_t NumberOfRpms = 3;
    inline static std::array<std::RPM, NumberOfRpms> rpms;
};

template<typename Sensor, typename Store, typename Alarm>
class TempFSM {
public:
    inline static void init() {
        mMeasureTimer = Alarm::create(750_ms, AlarmFlags::Disabled | AlarmFlags::OneShot);
        mTempTimer = Alarm::create(3000_ms, AlarmFlags::Periodic);
    }
    inline static void tick(uint8_t timer) {
        if (timer == *mMeasureTimer) {
            process(Event::WaitOver);
        }
        else if (timer == *mTempTimer) {
            process(Event::Measure);
        }
    }
    inline static void periodic() {
        Sensor::periodic([]{
            if (mState == State::WaitRead) {
                mState = State::Start;
                if (mSensorNumber == mSensorNumbers[0]) {
                    sensorData::temp1(Sensor::temperature());
                }
                if (mSensorNumber == mSensorNumbers[1]) {
                    sensorData::temp2(Sensor::temperature());
                }
                Store::temps[mSensorNumber] = Sensor::temperature();
                mSensorNumber = (mSensorNumber + 1) % Store::dsIds.size();
            }
        });
    }
private:
    enum class State : uint8_t {Start, Wait, WaitRead};
    enum class Event : uint8_t {Measure, WaitOver};
    
    inline static void process(Event e) {
        switch (mState) {
        case State::Start:
            if (e == Event::Measure) {
                Sensor::convert();
                alarmTimer::start(*mMeasureTimer);
                mState = State::Wait;
            }
            break;
        case State::Wait:
            if (e == Event::WaitOver) {
                if (Store::dsIds[mSensorNumber]) {
                    Sensor::startGet(Store::dsIds[mSensorNumber]);
                    mState = State::WaitRead;
                }
                else {
                    mSensorNumber = 0;
                    mState = State::Start;
                }
            }            
            break;
        case State::WaitRead:
            break;
        }
    }
    inline static State mState = State::Start;
    inline static uint8_t mSensorNumber = 0; 
    inline static std::optional<uint7_t> mMeasureTimer;
    inline static std::optional<uint7_t> mTempTimer;
    inline static std::array<uint8_t, 2> mSensorNumbers = {0, 1};
};

using tempFSM = TempFSM<ds18b20, Storage, alarmTimer>;

int main() {
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    testPin0::dir<AVR::Output>();
    testPin0::off();
    testPin1::dir<AVR::Output>();
    testPin1::off();
    testPin2::dir<AVR::Output>();
    testPin2::off();
    
    hbrigeError::dir<AVR::Input>();
    
    isrRegistrar::init();
    
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    hardPwm::init<Constants::pwmFrequency>();
    hbridge1::init();
    hbridge2::init();
    
    crWriterSensorBinary::init();
    crWriterSensorText::init();
    
    gpsUsart::init<9600>();
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    
#ifdef USE_TC1_AS_HARDPPM
    hardPpm1::init();
#else
    rpm1::init();
#endif
    
#ifdef USE_TC3_AS_HARDPPM
    hardPpm2::init();
#else
    rpm2::init();
#endif
    
    rpm3::init();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test17"_pgm);
        Util::delay(100_ms);
        
        {
            std::outl<terminal>("1Wire:"_pgm);
            std::array<OneWire::ow_rom_t, Storage::dsIds.capacity> ids;
            oneWireMaster::findDevices(ids, ds18b20::family);
            for(const auto& id : ids) {
                if (id) {
                    Storage::dsIds.push_back(id);
                    std::outl<terminal>(id);
                    Util::delay(100_ms);
                }
            }
        }
        
        Util::delay(100_ms);
        
        std::outl<terminal>("tc4 pre: "_pgm, rpmTimer3::frequency());
        
        const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);
        
        Util::delay(100_ms);
        
        tempFSM::init();
        
        // todo: menu für Kanäle
        while(true) {
            // todo: in eigene Klasse -> RoundRobin
            tempFSM::periodic();
            auto v1 = Hott::SumDProtocollAdapter<0>::value(0);
            hbridge1::pwm(v1);
            auto v2 = Hott::SumDProtocollAdapter<0>::value(5);
            hbridge2::pwm(v2);
            rpm3::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([&](){
                testPin0::on();
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                oneWireMasterAsync::rateProcess();

                Storage::rpms[2] = rpm3::rpm();
                sensorData::rpm1(Storage::rpms[2]);
                
                testPin0::off();
                alarmTimer::periodic([&](uint7_t timer){
                    if (timer == *periodicTimer) {
                        rpm3::check();
                        //                        std::outl<terminal>("mem: "_pgm, Util::Memory::getUnusedMemory());
                        GPS::RMC::timeRaw(Storage::time);
                        GPS::VTG::speedRaw(Storage::decimalBuffer);
                        auto s = Util::StringConverter<FixedPoint<int16_t, 4>>::parse(Storage::decimalBuffer);
                        //                        std::outl<terminal>("time: "_pgm, Storage::time);
                        std::outl<terminal>("speed: "_pgm, s);
                        //                        std::outl<terminal>("v: "_pgm, v.toInt());
                        //                        auto valid = Hott::SumDProtocollAdapter<0>::valid();
                        //                        std::outl<terminal>("valid: "_pgm, valid);
                        std::outl<terminal>("temps: "_pgm, Storage::temps[0], ", "_pgm, Storage::temps[1]);
                        std::outl<terminal>("error: "_pgm, hbrigeError::read());
                        
                        std::outl<terminal>("r: "_pgm, Storage::rpms[2]);
                    }
                    else {
                        tempFSM::tick(timer);
                    }
                });
            });
        }
    }
}
// GPS
ISR(USART2_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<2>::RX>();
}
ISR(USART2_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<2>::UDREmpty>();
}
// SumD
ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}
// Sensor
ISR(USART0_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
}
#endif
