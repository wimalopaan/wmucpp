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

//#define MEM
#define NDEBUG

#include "local.h"
#include "rcsensorled03b.h"
#include "console.h"
#include "util/meta.h"
#include "hal/eeprom.h"
#include "external/hott/menu.h"

using testPin1 = AVR::Pin<PortB, 7>; // dir_2

namespace Storage {
    enum class AVKey : uint8_t {TSensor1 = 0, TSensor2, RpmSensor1, RpmSensor2, Spannung1, Spannung2, 
                                StromOffset, PWM, PWMMOde, Leds1Channel, Leds1Sequence, Leds2Channel, Leds2Sequence, _Number};
    
    class ApplData : public EEProm::DataBase<ApplData> {
    public:
        uint_NaN<uint8_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
    std::array<OneWire::ow_rom_t, 4> dsIds;
}

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

namespace {
    constexpr bool useTerminal = true;
}

namespace Constants {
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
}

struct AsciiHandler;
struct BinaryHandler;
struct BCastHandler;

using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0, UseEvents<false>, AsciiHandler, BinaryHandler, BCastHandler>, 
MCU::UseInterrupts<true>, UseEvents<false>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>, MCU::UseInterrupts<true>, UseEvents<false>>;

using terminalDevice = std::conditional<useTerminal, rcUsart, void>::type;
using terminal = std::basic_ostream<terminalDevice>;

const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

using namespace std::literals::quantity;

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, 
sensorUsart::RxHandler, sensorUsart::TxHandler,
softPpm::OCAHandler, softPpm::OCBHandler>;

using sensorData = Hott::SensorProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using menuData = Hott::SensorTextProtocollBuffer<0>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;


class TSensorId final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
public:
    TSensorId(uint8_t number) : mNumber{number} {
        assert(number < Storage::dsIds.size);
    }
    virtual void putTextInto(Hott::BufferString& buffer) const override {
        if (Storage::dsIds[mNumber]) {
            for(uint8_t i = 0; i < (Storage::dsIds[mNumber].size - 2); ++i) {
                uint8_t d = Storage::dsIds[mNumber][i + 1];
                Util::itoa_r<16>(d, &buffer[i * 3]);
            }
            for(uint8_t i = 0; i < (Storage::dsIds[mNumber].size - 3); ++i) {
                buffer[i * 3 + 2] = ':';
            }
        }
        else {
            buffer.insertAtFill(0, "--:--:--:--:--:--"_pgm);
        }
    }
private:
    uint8_t mNumber = 0;
};

class TSensorMenu final : public Hott::NumberedMenu {
public:
    TSensorMenu(Menu* parent, uint8_t n) : NumberedMenu(parent, n, "Sensor"_pgm, &mID), mID{n} {}
private:
    TSensorId mID{0};
};

class TemperaturSensorenMenu final : public Hott::Menu {
public:
    TemperaturSensorenMenu(Menu* parent) : Menu(parent, "Temp. Sensoren"_pgm, &mTS1, &mTS2, &mTS3, &mTS4) {}
private:
    TSensorMenu mTS1{this, 0};
    TSensorMenu mTS2{this, 1};
    TSensorMenu mTS3{this, 2};
    TSensorMenu mTS4{this, 3};
};

class TemperaturMenu final : public Hott::Menu {
public:
    TemperaturMenu(Hott::Menu* parent) : Hott::Menu(parent, "Temperatur"_pgm, &mSensor1, &mSensor2, &mSensoren) {}
    
private:
    Hott::TextWithValue<Storage::AVKey, Storage::dsIds.size - 1, Storage::ApplData> mSensor1{"Anzeige 1"_pgm, appData, Storage::AVKey::TSensor1};
    Hott::TextWithValue<Storage::AVKey, Storage::dsIds.size - 1, Storage::ApplData> mSensor2{"Anzeige 2"_pgm, appData, Storage::AVKey::TSensor2};
    TemperaturSensorenMenu mSensoren{this};
};

class SpannungMenu final : public Hott::Menu {
public:
    SpannungMenu(Hott::Menu* parent) : Menu(parent, "Spannung"_pgm, &mSensor1, &mSensor2) {}
private:
    Hott::TextWithValue<Storage::AVKey, 1, Storage::ApplData> mSensor1{"Anzeige 1"_pgm, appData, Storage::AVKey::Spannung1};
    Hott::TextWithValue<Storage::AVKey, 1, Storage::ApplData> mSensor2{"Anzeige 2"_pgm, appData, Storage::AVKey::Spannung2};
};

class DrehzahlMenu final : public Hott::Menu {
public:
    DrehzahlMenu(Hott::Menu* parent) : Menu(parent, "Drehzahl"_pgm, &mSensor1, &mSensor2) {}
private:
    Hott::TextWithValue<Storage::AVKey, 1, Storage::ApplData> mSensor1{"Anzeige 1"_pgm, appData, Storage::AVKey::RpmSensor1};
    Hott::TextWithValue<Storage::AVKey, 1, Storage::ApplData> mSensor2{"Anzeige 2"_pgm, appData, Storage::AVKey::RpmSensor2};
};

class StromMenu final : public Hott::Menu {
public:
    StromMenu(Hott::Menu* parent) : Menu(parent, "Strom"_pgm, &mOffset) {}
private:
    Hott::TextWithValue<Storage::AVKey, 1, Storage::ApplData> mOffset{"Offset"_pgm, appData, Storage::AVKey::StromOffset};
};

class PWMType : public Hott::TextWithValue<Storage::AVKey, 3, Storage::ApplData> {
public:
    PWMType(const PgmStringView& title, Storage::ApplData& data, Storage::AVKey k) :
        TextWithValue(title, data, k) {}
    //    virtual void valueToText(uint8_t value, char *buffer) const override {
    virtual void valueToText(uint8_t value, UI::span<3, char> buffer) const override {
        if (value == 0) {
            buffer[0] = 'V';
            buffer[1] = ' ';
            buffer[2] = ' ';
        }
        else if (value == 1) {
            buffer[0] = 'R';
            buffer[1] = ' ';
            buffer[2] = ' ';
        }
        else if (value == 2) {
            buffer[0] = 'V';
            buffer[1] = '/';
            buffer[2] = 'R';
        }
    }
private:
};

class PWMMenu final : public Hott::Menu {
public:
    PWMMenu(Hott::Menu* parent) : Menu(parent, "PWM"_pgm, &mOffset, &mType) {}
private:
    Hott::TextWithValue<Storage::AVKey, 8, Storage::ApplData> mOffset{"Kanal"_pgm, appData, Storage::AVKey::PWM};
    PWMType mType{"Modus"_pgm, appData, Storage::AVKey::PWMMOde};
};

class LedMenu final : public Hott::Menu {
public:
    LedMenu(Hott::Menu* parent) : Menu(parent, "LEDs"_pgm, &mChannel1, &mSequence1, &mChannel2, &mSequence2) {}
private:
    Hott::TextWithValue<Storage::AVKey, 8, Storage::ApplData> mChannel1{"Kanal 1"_pgm, appData, Storage::AVKey::Leds1Channel};
    Hott::TextWithValue<Storage::AVKey, 8, Storage::ApplData> mSequence1{"Folge 1"_pgm, appData, Storage::AVKey::Leds1Sequence};
    Hott::TextWithValue<Storage::AVKey, 8, Storage::ApplData> mChannel2{"Kanal 2"_pgm, appData, Storage::AVKey::Leds2Channel};
    Hott::TextWithValue<Storage::AVKey, 8, Storage::ApplData> mSequence2{"Folge 2"_pgm, appData, Storage::AVKey::Leds2Sequence};
};

class RCMenu final : public Hott::Menu {
public:
    RCMenu() : Menu(this, "WM SensorLed"_pgm, &mTemperatur, &mSpannung, &mDrehzahl, &mStrom, &mPWM, &mLeds) {}
private:
    TemperaturMenu mTemperatur{this};                        
    SpannungMenu   mSpannung{this};                        
    DrehzahlMenu   mDrehzahl{this};                        
    StromMenu   mStrom{this};                        
    PWMMenu   mPWM{this};                        
    LedMenu   mLeds{this};                        
};

template<typename PA, typename MenuType>
class HottMenu final {
    HottMenu() = delete;
public:
    inline static void periodic() {
        Hott::key_t k = Hott::key_t::nokey;
        {
            Scoped<DisbaleInterrupt<>> di;
            k = mKey;
            mKey = Hott::key_t::nokey;
        }
        if (k != Hott::key_t::nokey) {
            processKey(k);
        }
        mMenu->textTo(PA::text());
    }
    inline static void processKey(Hott::key_t key) {
        assert(mMenu);
        if (auto m = mMenu->processKey(key); m != mMenu) {
            mMenu = m;
            for(auto& line : PA::text()) {
                line.clear();
            }
        }
    }
    inline static void isrKey(std::byte b) {
        mKey = Hott::key_t{b};
    }
private:
    inline static volatile Hott::key_t mKey = Hott::key_t::nokey;
    inline static MenuType mTop;
    inline static Hott::Menu* mMenu = &mTop;
};

using menu = HottMenu<menuData, RCMenu>;


struct AsciiHandler {
    static void start() {
        crWriterSensorText::enable<true>();
    }    
    static void stop() {
        crWriterSensorText::enable<false>();
    }    
    static void process(std::byte key) {
        menu::isrKey(key);        
    }
};
struct BinaryHandler {
    static void start() {
        crWriterSensorBinary::enable<true>();
    }    
    static void stop() {
        crWriterSensorBinary::enable<false>();
    }    
};
struct BCastHandler {
    static void start() {
        std::outl<terminal>("hbr start"_pgm);
    }    
    static void stop() {
    }    
};


int main() {
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    isrRegistrar::init();
    
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    led::init();
    led::set(Constants::cGreen);

    testPin1::dir<AVR::Output>();
    
    crWriterSensorBinary::init();
    crWriterSensorText::init();
    
    softPpm::init();    
    softPpm::ranged_type v1{2501};
    softPpm::ranged_type v2{4999};
    softPpm::ppm(v1, 0);
    softPpm::ppm(v2, 1);
    
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test09"_pgm);
        
        while(true){
            testPin1::toggle(); // 25 us -> 40 KHz
            menu::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                alarmTimer::periodic([](uint7_t timer){
                    if (timer == *periodicTimer) {
                        auto v = Hott::SumDProtocollAdapter<0>::value(0);
                        std::outl<terminal>("v: "_pgm, v.toInt());
                        softPpm::ppm(v, 0);
                    }
                });
                appData.expire();
            });
            while(eeprom::saveIfNeeded()) {
                std::outl<terminal>("."_pgm);
            }
        }
    }    
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
// Timer 4
// softPpm
ISR(TIMER4_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<4>::CompareA>();
}
ISR(TIMER4_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<4>::CompareB>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif

