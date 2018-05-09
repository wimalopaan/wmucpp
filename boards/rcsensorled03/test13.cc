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

//#define USE_RPM2

#define USE_TC1_AS_HARDPPM

#ifndef USE_TC1_AS_HARDPPM
# define USE_RPM2_ON_OPTO2
#endif

//#define MEM
#define NDEBUG

#include "local.h"
#include "rcsensorled03c.h"
#include "console.h"
#include "util/meta.h"
#include "hal/eeprom.h"
#include "external/hott/menu.h"

#include <vector>

using testPin1 = AVR::Pin<PortD, 3>; // I2C Interrupt

struct Storage {
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
    
    inline static std::vector<OneWire::ow_rom_t, 4> dsIds;
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

namespace {
    constexpr bool useTerminal = true;
}

namespace Constants {
    static constexpr std::hertz pwmFrequency = 8000_Hz * 256; 
//    static constexpr std::hertz pwmFrequency = 1000_Hz;
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

using namespace std::literals::quantity;

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, 
sensorUsart::RxHandler, sensorUsart::TxHandler
#ifdef USE_RPM2
, softPpm::OCAHandler, softPpm::OCBHandler
#endif
>;

using sensorData = Hott::SensorProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using menuData = Hott::SensorTextProtocollBuffer<0>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;


class TSensorId final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
public:
    TSensorId(uint8_t number) : mNumber{number} {
        assert(number < Storage::dsIds.capacity);
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
    Hott::TextWithValue<Storage::AVKey, Storage::dsIds.capacity - 1, Storage::ApplData> mSensor1{"Anzeige 1"_pgm, appData, Storage::AVKey::TSensor1};
    Hott::TextWithValue<Storage::AVKey, Storage::dsIds.capacity - 1, Storage::ApplData> mSensor2{"Anzeige 2"_pgm, appData, Storage::AVKey::TSensor2};
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

const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);

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
            mState = State::Start;
            std::outl<terminal>("temp: "_pgm, mSensorNumber, " : "_pgm, Sensor::temperature());
            mSensorNumber = (mSensorNumber + 1) % Store::dsIds.size();
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
};

using tempFSM = TempFSM<ds18b20, Storage, alarmTimer>;

int main() {
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    eeprom::init();
    
    isrRegistrar::init();
    
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    leds::init();
    leds::off();

    Util::delay(1_ms);    
    leds::set(0, Constants::cGreen);
    Util::delay(1_ms);    
    leds::set(1, Constants::cRed);
    Util::delay(1_ms);    
    leds::set(2, Constants::cBlue);

    testPin1::dir<AVR::Output>();
    
    crWriterSensorBinary::init();
    crWriterSensorText::init();
    
    // hardPpm und rpm2 sind nicht gleichzeitig nutzbar
    
#ifndef USE_RPM2
    hardPpm::init();
#else 
    softPpm::init();    
#endif
    rpm1::init();
#ifdef USE_RPM2
    rpm2::init();
#endif
    
    hardPwm::init<Constants::pwmFrequency>();
  
    ds18b20::init();
    
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test13"_pgm);

        {
            std::array<OneWire::ow_rom_t, 4> ids;
            oneWireMaster::findDevices(ids, ds18b20::family);
            for(const auto& id : ids) {
                if (id) {
                    Storage::dsIds.push_back(id);
                    std::outl<terminal>(id);
                    Util::delay(10_ms);
                }
            }
        }
        
        tempFSM::init();

        
        while(true){
//            testPin1::toggle(); // 25 us -> 40 KHz
            menu::periodic();
            rpm1::periodic();
#ifdef USE_RPM2
            rpm2::periodic();
#endif
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                oneWireMasterAsync::rateProcess();
                alarmTimer::periodic([](uint7_t timer){
                    if (timer == *periodicTimer) {
                        rpm1::check();
#ifdef USE_RPM2
                        rpm2::check();
#endif
                        auto v1 = Hott::SumDProtocollAdapter<0>::value(1);
#ifdef USE_RPM2
                        softPpm::ppm(v1, 0);
#endif         
#ifndef USE_RPM2
                        hardPpm::ppm<hardPpm::A>(v1);
#endif                        
                        auto v0 = Hott::SumDProtocollAdapter<0>::value(0);
                        hardPwm1::pwm(v0);
                        hardPwm2::pwm(v0);
                        
                        std::outl<terminal>("rpm1: "_pgm, rpm1::rpm().value());
#ifdef USE_RPM2
                        std::outl<terminal>("rpm2: "_pgm, rpm2::rpm().value());
#endif
                    }
                    else {
                        tempFSM::tick(timer);
                    }
                });
                appData.expire();
            });
            tempFSM::periodic();
            
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
#ifdef USE_RPM2
ISR(TIMER4_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<4>::CompareA>();
}
ISR(TIMER4_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<4>::CompareB>();
}
#endif

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif

