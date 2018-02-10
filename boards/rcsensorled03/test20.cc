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

#define USE_TC1_AS_HARDPPM

#ifndef USE_TC1_AS_HARDPPM
# define USE_RPM2_ON_OPTO2
#endif

#define USE_TC0_AS_SWUSART

//#define USE_PPM_ON_OPTO2

//#ifdef USE_PPM_ON_OPTO2
//# define USE_ICP1
//# ifdef USE_RPM2_ON_OPTO2
//#  error "wrong config"
//# endif
//#endif

#define MEM
#define NDEBUG

#include "local.h"
#include "rcsensorled03c.h"
#include "console.h"
#include "util/meta.h"
#include "hal/eeprom.h"
#include "external/hott/menu.h"

#include <vector>

template<typename F>
struct StringConverter;

template<uint8_t N>
struct StringConverter<FixedPoint<int16_t, N>> {
    template<uint8_t L>
    inline static FixedPoint<int16_t, N> parse(const StringBuffer<L>& str) {
        int16_t value = 0;
        int8_t decimals = -1;
        bool negative = false;
        
        uint8_t index = 0;
        
        if (str[0] == '-') {
            negative = true;
            ++index;
        }
        
        for(; index < L; ++index) {
            if (str[index] == '.') {
                decimals = 0;
            }
            else if ((str[index] >= '0') && (str[index] <= '9')) {
                value *= 10;
                value += (str[index] - '0');
                if (decimals >= 0) {
                    ++decimals;
                }
            }
            else if (str[index] == '\0') {
                break;
            }
        }
        value <<= N;
        while(decimals > 0) {
            value /= 10;
            --decimals;
        }
        if (negative) {
            value = -value;
        }
        return FixedPoint<int16_t, N>::fromRaw(value);
    }
};



using testPin1 = AVR::Pin<PortC, 0>;
using testPin2 = AVR::Pin<PortC, 1>;
using testPin3 = AVR::Pin<PortC, 2>;

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
    
    inline static constexpr uint8_t NumberOfOWireDevs = 4;
    inline static constexpr uint8_t NumberOfI2CDevs = 4;
    
    inline static std::vector<OneWire::ow_rom_t, NumberOfOWireDevs> dsIds;
    inline static std::vector<TWI::Address, NumberOfI2CDevs> i2cDevices;
    
    inline static std::array<FixedPoint<int, 4>, NumberOfOWireDevs> temps;
    inline static std::array<std::RPM, 2> rpms;
    
    inline static std::array<FixedPoint<int, 4>, 2> batts;
    inline static std::array<FixedPoint<int, 4>, 2> minCells;
    
    inline static std::array<FixedPoint<int, 4>, 2> currents;
    
    inline static uint8_t switches = 0;
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

namespace {
    constexpr bool useTerminal = true;
}

namespace Constants {
    static constexpr std::hertz pwmFrequency = 8000_Hz * 256; 
    static constexpr const std::hertz fSCL = 100000_Hz;
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

struct I2CInterrupt : public IsrBaseHandler<AVR::ISR::Int<1>> {
    static void isr() {
    }
};
using i2cInterruptHandler = I2CInterrupt;

using isrRegistrar = IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, 
sensorUsart::RxHandler, sensorUsart::TxHandler
#ifdef USE_RPM2_ON_OPTO2
, softPpm::OCAHandler, softPpm::OCBHandler
#else
//, ppmDecoder
#endif
,i2cInterruptHandler
#ifdef USE_TC0_AS_SWUSART
, gpsUart::RxHandler, gpsUart::TxHandler, gpsUart::StartBitHandler
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

class I2CId final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
public:
    I2CId(uint8_t number) : mNumber{number} {
    }
    virtual void putTextInto(Hott::BufferString& buffer) const override {
        buffer.insertAtFill(0, "--:--:--:--:--:--"_pgm);
    }
private:
    uint8_t mNumber = 0;
};

class I2CDev final : public Hott::NumberedMenu {
public:
    I2CDev(Menu* parent, uint8_t n) : NumberedMenu(parent, n, "Geraet"_pgm, &mDev), mDev{n} {}
private:
    I2CId mDev{0};
};

class I2CMenu final : public Hott::Menu {
public:
    I2CMenu(Hott::Menu* parent) : Menu(parent, "Geraete"_pgm, &mI2CDev1, &mI2CDev2, &mI2CDev3/*, &mI2CDev4*/) {}
private:
    I2CDev mI2CDev1{this, 0};
    I2CDev mI2CDev2{this, 1};
    I2CDev mI2CDev3{this, 2};
    I2CDev mI2CDev4{this, 3};
};

class Switch final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
public:
    Switch(uint8_t number) : mNumber{number} {
    }
    virtual void putTextInto(Hott::BufferString& buffer) const override {
        buffer.insertAt(0, "Switch "_pgm);
        buffer[7] = '0' + (2 * mNumber);
        buffer[8] = ':';
        buffer[9] = '1' + (2 * mNumber);
        
        buffer.insertAt(11, "on "_pgm);
        
        if (mNumber == 1) {
            buffer[11] |= 0x80;
            buffer[12] |= 0x80;
            buffer[13] |= 0x80;
        }
        
        buffer.insertAt(15, "off "_pgm);
    }
private:
    uint8_t mNumber = 0;
};


class SwitchMenu final : public Hott::Menu {
public:
    SwitchMenu(Hott::Menu* parent) : Menu(parent, "Multiswitch"_pgm, &mSW0, &mSW1, &mSW2, &mSW3) {}
private:
    Switch mSW0{0};
    Switch mSW1{1};
    Switch mSW2{2};
    Switch mSW3{3};
};

template<typename T, uint8_t L1, uint8_t LField>
class DualValue : public Hott::MenuItem {
public:
    DualValue(const PgmStringView& text, const T& v1, const T& v2) : mText(text), mData1{v1}, mData2{v2} {}
    
    virtual void putTextInto(Hott::BufferString& buffer) const override {
        buffer.clear();
        buffer.insertAt(0, mText);
        putValue(mData1, UI::make_span<L1, LField>(buffer));
        putValue(mData2, UI::make_span<L1 + LField, LField>(buffer));
    }
private:
    template<typename I, uint8_t F>
    inline void putValue(const FixedPoint<I, F>& value, UI::span<LField, char> b) const{
        uint8_t i = value.integerAbs();
        Util::itoa_r(i, b);
        auto f = value.fraction();
        auto b2 = UI::make_span<3, 5>(b);
        Util::ftoa(f, b2);
    }
    inline void putValue(const std::RPM& rpm, UI::span<LField, char> b) const {
        Util::itoa_r(rpm.value(), b);
    }
    const PgmStringView mText;
    const T& mData1;
    const T& mData2;
};

class InfoMenu final : public Hott::Menu {
public:
    InfoMenu(Hott::Menu* parent) : Menu(parent, "Uebersicht"_pgm, &mTempLine1, &mTempLine2, &mRpmLine, &mBattLine, &mMinCellLine, &mCurrentLine) {}
private:
    DualValue<FixedPoint<int, 4>, 5, 8> mTempLine1{"T1/2"_pgm, Storage::temps[0], Storage::temps[1]};
    DualValue<FixedPoint<int, 4>, 5, 8> mTempLine2{"T3/4"_pgm, Storage::temps[2], Storage::temps[3]};
    DualValue<std::RPM, 5, 8>           mRpmLine{"R1/2"_pgm, Storage::rpms[0], Storage::rpms[1]};
    DualValue<FixedPoint<int, 4>, 5, 8> mBattLine{"B1/2"_pgm, Storage::batts[0], Storage::batts[1]};
    DualValue<FixedPoint<int, 4>, 5, 8> mMinCellLine{"M1/2"_pgm, Storage::minCells[0], Storage::minCells[1]};
    DualValue<FixedPoint<int, 4>, 5, 8> mCurrentLine{"A1/2"_pgm, Storage::currents[0], Storage::currents[1]};
};

class ActorMenu final : public Hott::Menu {
public:
    ActorMenu(Hott::Menu* parent) : Menu(parent, "Aktoren"_pgm, &mDevs, &mSwitch, &mPWM, &mLeds) {}
private:
    SwitchMenu mSwitch{this};
    PWMMenu   mPWM{this};                        
    LedMenu   mLeds{this};                        
    I2CMenu   mDevs{this};                        
};

class RCMenu final : public Hott::Menu {
public:
    RCMenu() : Menu(this, "WM SensMod HW 3 SW 20"_pgm, &mActors, &mInfo, &mTemperatur, &mSpannung, &mDrehzahl, &mStrom) {}
private:
    InfoMenu mInfo{this};
    TemperaturMenu mTemperatur{this};                        
    SpannungMenu   mSpannung{this};                        
    DrehzahlMenu   mDrehzahl{this};                        
    StromMenu   mStrom{this};                        
    ActorMenu mActors{this};
};

RCMenu topMenu; // note: global object instead of static in Hottmenu -> no guards are created (is this a g++ error)

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
    inline static Hott::Menu* mMenu = &topMenu;
};

using menu = HottMenu<menuData, RCMenu>;

struct AsciiHandler {
    static void start() {
        crWriterSensorText::enable<true>();
    }    
    static void stop() {
        // not: das disable sollte automatisch laufen
        //        crWriterSensorText::enable<false>();
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
        // not: das disable sollte automatisch laufen
        //        crWriterSensorBinary::enable<false>();
    }    
};
struct BCastHandler {
    static void start() {
        std::outl<terminal>("hbr start"_pgm);
        crWriterSensorBinary::enable<true>();
    }    
    static void stop() {
    }    
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
            mState = State::Start;
            if (mSensorNumber == 0) {
                sensorData::temp1(Sensor::temperature());
                std::outl<terminal>("Temp1: "_pgm, Sensor::temperature());
            }
            if (mSensorNumber == 1) {
                sensorData::temp2(Sensor::temperature());
            }
            Storage::temps[mSensorNumber] = Sensor::temperature();
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

template<typename LEDs, typename Alarm, uint8_t Offset = 1>
class LedFSM {
    enum class State : uint8_t {};
public:
    inline static void init() {
        mTimer = Alarm::create(1000_ms, AlarmFlags::Periodic);
    }
    inline static void tick(uint8_t timer) {
        if (timer == *mTimer) {
            LEDs::template set<false>(Color{});
            LEDs::set(mActual, Constants::cBlue);
            ++mActual;
        }
    }
private:    
    inline static std::optional<uint7_t> mTimer;
    inline static uint_ranged_circular<uint8_t, Offset, LEDs::size - 1> mActual;
};

using ledFSM = LedFSM<leds, alarmTimer>;

class Measurements {
    inline static constexpr uint8_t hottScale = adcController::mcu_adc_type::VRef / 0.02;
    inline static constexpr auto rawMax = adcController::value_type::Upper;
    inline static constexpr uint8_t battScale = adcController::mcu_adc_type::VRef / 0.1;
public:
    static inline void update() {
        switch (part) {
        case 0:
            start();
            update0();
            break;
        case 1:
            update1();
            break;
        case 2:
            update2();
            break;
        case 3:
            update3();
            break;
        case 4:
            update4();
            break;
        default:
            assert(false);
        }
        ++part;
    }
private:
    static inline void start() {
        zmin = std::numeric_limits<uint16_t>::max();
        zminNum = uint_NaN<uint8_t>();
    }
    static inline void update0() {
        uint16_t v1 = adcController::value(1) * 4;
        uint16_t v2 = adcController::value(2) * 8;
        uint16_t v3 = adcController::value(0) * 12;
        
        uint16_t z1 = (v1 * hottScale) / rawMax;
        uint16_t z2 = ((v2 - v1) * hottScale) / rawMax;
        uint16_t z3 = ((v3 - v2) * hottScale) / rawMax;
        
        if ((z1 > 0) && (z1 < zmin)) {
            zmin = z1;
            zminNum = 0;
        }
        if ((z2 > 0) && (z2 < zmin)) {
            zmin = z2;
            zminNum = 1;
        }
        if ((z3 > 0) && (z3 < zmin)) {
            zmin = z3;
            zminNum = 2;
        }
        sensorData::cellVoltageRaw(0, z1);
        sensorData::cellVoltageRaw(1, z2);
        sensorData::cellVoltageRaw(2, z3);
        uint16_t batt = (v3 * battScale) / rawMax;
        sensorData::batteryVoltageRaw(0, batt);
        sensorData::mainVoltageRaw(batt);
        
        Storage::batts[0] = FixedPoint<int, 4>::fromRaw((batt << 4) / 10);
        if (zminNum) {
            Storage::minCells[0] = FixedPoint<int, 4>::fromRaw((zmin << 4) / 50);
            sensorData::batteryMinimumRaw(zminNum, zmin);
        }
    }
    static inline void update1() {
        uint16_t v1 = adcController::value(4) * 4;
        uint16_t v2 = adcController::value(5) * 8;
        uint16_t v3 = adcController::value(3) * 12;
        
        uint16_t z1 = (v1 * hottScale) / rawMax;
        uint16_t z2 = ((v2 - v1) * hottScale) / rawMax;
        uint16_t z3 = ((v3 - v2) * hottScale) / rawMax;
        
        if ((z1 > 0) && (z1 < zmin)) {
            zmin = z1;
            zminNum = 3;
        }
        if ((z2 > 0) && (z2 < zmin)) {
            zmin = z2;
            zminNum = 4;
        }
        if ((z3 > 0) && (z3 < zmin)) {
            zmin = z3;
            zminNum = 5;
        }
        
        sensorData::cellVoltageRaw(3, z1);
        sensorData::cellVoltageRaw(4, z2);
        sensorData::cellVoltageRaw(5, z3);
        uint16_t batt = (v3 * battScale) / rawMax;
        sensorData::batteryVoltageRaw(1, batt);
        
        Storage::batts[1] = FixedPoint<int, 4>::fromRaw((batt << 4) / 10);
        if (zminNum) {
            Storage::minCells[1] = FixedPoint<int, 4>::fromRaw((zmin << 4) / 50);
            sensorData::batteryMinimumRaw(zminNum, zmin);
        }
    }
    static inline void update2() {
        // todo: Skalierung berechnen    
        uint16_t a = adcController::value(6);
        uint16_t a1 = 0;
        if (a >= 128) {
            a1 = a - 128;
        }
        else {
            a1 = 128 - a;
        }
        
        uint16_t c1 = (a1 * 298) / 100;
        
        sensorData::currentRaw(c1);
    }
    static inline void update3() {
        const auto upm1 = rpm1::rpm();
        sensorData::rpm1(upm1);
        Storage::rpms[0] = rpm1::rpm();
        
#ifdef USE_RPM2_ON_OPTO2
        const auto upm2 = rpm2::rpm();
        Storage::rpms[1] = upm2;
#endif
    }
    static inline void update4() {
#ifdef USE_TC0_AS_SWUSART
        GPS::VTG::speedRaw(decimalBuffer);
        auto s = StringConverter<FixedPoint<int16_t, 4>>::parse(decimalBuffer);
        sensorData::speedRaw(s.raw());
#endif
        
    }
    inline static StringBuffer<GPS::Sentence::DecimalMaxWidth> decimalBuffer;
    
    inline static uint_ranged_circular<uint8_t, 0, 4> part;
    inline static uint16_t zmin = std::numeric_limits<uint16_t>::max();
    inline static uint_NaN<uint8_t> zminNum;
};

inline void updateActors() {
    static uint_ranged_circular<uint8_t, 0, 3> part;
    switch(part) {
    case 0: 
    {
        auto v1 = Hott::SumDProtocollAdapter<0>::value(1);
#ifdef USE_RPM2_ON_OPTO2
        softPpm::ppm(v1, 0);
#else
        hardPpm::ppm<hardPpm::A>(v1);
#endif
    }
        break;
    case 1:
    {
        auto v3 = Hott::SumDProtocollAdapter<0>::value(3);
#ifdef USE_RPM2_ON_OPTO2
        softPpm::ppm(v3, 1);
#else
        hardPpm::ppm<hardPpm::B>(v3);
#endif
    }
        break;
    case 2:
    {
        auto v0 = Hott::SumDProtocollAdapter<0>::value(0);
#ifndef USE_TC0_AS_SWUSART
        hbridge1::pwm(v0);
#endif
    }
        break;
    case 3:
    {
        auto v0 = Hott::SumDProtocollAdapter<0>::value(0);
#ifndef USE_TC0_AS_SWUSART
        hbridge2::pwm(v0);
#endif
    }
        break;
    default:
        assert(false);
    }
    ++part;
}

inline void updateMultiChannel() {
    static uint8_t part = 0;
    static std::byte d{0};
    if (Hott::SumDProtocollAdapter<0>::hasMultiChannel()) {
        if (part < Hott::MultiChannel::size) {
            if (Hott::SumDProtocollAdapter<0>::mChannel(part) == Hott::MultiChannel::State::Up) {
                d |= std::byte(1 << part);
            }
            ++part;
        }
        else {
            mcp23008::startWrite(0x09, d);
            d = 0_B;
            part = 0;            
        }
    }
    else {
        //        mcp23008::startWrite(0x09, std::byte{++part});
    }
}

int main() {
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    testPin1::dir<AVR::Output>();
    testPin1::on();
    
    testPin2::dir<AVR::Output>();
    testPin2::on();
    
    testPin3::dir<AVR::Output>();
    testPin3::on();
    
    eeprom::init();
    adcController::init();
    
    isrRegistrar::init();
    
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
#ifndef USE_RPM2_ON_OPTO2
    //    ppmDecoder::init();
#endif
    
    leds::init();
    leds::off();
    
    Util::delay(1_ms);    
    leds::set(0, Constants::cGreen);
    
    ledFSM::init();
    
    testPin1::dir<AVR::Output>();
    
    crWriterSensorBinary::init();
    crWriterSensorText::init();
    
#ifdef USE_TC1_AS_HARDPPM
    hardPpm::init();
#else 
    softPpm::init();    
#endif
    rpm1::init();
#ifdef USE_RPM2_ON_OPTO2
    rpm2::init();
#endif
    
#ifndef USE_TC0_AS_SWUSART
    hardPwm::init<Constants::pwmFrequency>();
#else
    gpsUart::init();
#endif
    
    ds18b20::init();
    
    TwiMaster::init<Constants::fSCL>();
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test20"_pgm);
        
        {
            std::array<OneWire::ow_rom_t, Storage::dsIds.capacity> ids;
            oneWireMaster::findDevices(ids, ds18b20::family);
            for(const auto& id : ids) {
                if (id) {
                    Storage::dsIds.push_back(id);
                    std::outl<terminal>(id);
                    Util::delay(10_ms);
                }
            }
        }
        {
            std::array<TWI::Address, Storage::i2cDevices.capacity> i2cAddresses;
            TwiMaster::findDevices(i2cAddresses);
            for(const auto& d : i2cAddresses) {
                if (d) {
                    Storage::i2cDevices.push_back(d);
                    std::outl<terminal>(d);
                    Util::delay(10_ms);
                }
            }
        }
        
        mcp23008::startWrite(0x00, std::byte{0x00}); // output
        mcp23008::startWrite(0x09, std::byte{0x00}); // output
        
        if (!oled::init()) {
            std::outl<terminal>("oled error"_pgm);
        }
        
        oled::clear();
        
        tempFSM::init();
        
        const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);
        
        while(true){
            testPin3::toggle(); // max 127 uS ein Intervall
            TwiMasterAsync::periodic();
            menu::periodic();
            rpm1::periodic();
#ifdef USE_RPM2_ON_OPTO2
            rpm2::periodic();
#endif
            adcController::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([&](){
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                oneWireMasterAsync::rateProcess();
                Measurements::update();
                updateActors();
                updateMultiChannel();
                
                alarmTimer::periodic([&](uint7_t timer){
                    if (timer == *periodicTimer) {
                        oled::put('.');
                        if (Hott::SumDProtocollAdapter<0>::hasMultiChannel()) {
                            std::out<terminal>("Multi["_pgm, Hott::SumDProtocollAdapter<0>::mChannelForMultiChannel, "]: "_pgm);
                            for(uint8_t i = 0; i < Hott::MultiChannel::size; ++i) {
                                std::out<terminal>(Char{' '}, (uint8_t)Hott::SumDProtocollAdapter<0>::mChannel(i));
                            }
                            std::outl<terminal>();
                        }
                        rpm1::check();
                        uint16_t a = adcController::value(6);
                        std::outl<terminal>("acs: "_pgm, a);
#ifdef USE_RPM2_ON_OPTO2
                        rpm2::check();
#endif
#ifdef USE_PPM_ON_OPTO2
                        std::outl<terminal>("ppm: "_pgm, ppmDecoder::value(0));
#endif
#ifdef USE_TC0_AS_SWUSART
#endif
#ifdef MEM
                        std::outl<terminal>("mem: "_pgm, Util::Memory::getUnusedMemory());
#endif
                    }
                    else {
                        tempFSM::tick(timer);
                        ledFSM::tick(timer);
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
ISR(INT1_vect) {
    isrRegistrar::isr<AVR::ISR::Int<1>>();
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
#ifdef USE_TC0_AS_SWUSART
ISR(TIMER0_COMPA_vect) {
    testPin3::on();
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
    testPin3::off();
}
ISR(TIMER0_COMPB_vect) {
    testPin2::on();
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareB>();
    testPin2::off();
}
ISR(INT0_vect) {
    testPin1::on();
    isrRegistrar::isr<AVR::ISR::Int<0>>();
    testPin1::off();
}
#endif
// Timer 4
// softPpm
#ifdef USE_RPM2_ON_OPTO2
ISR(TIMER4_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<4>::CompareA>();
}
ISR(TIMER4_COMPB_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<4>::CompareB>();
}
#endif

#ifdef USE_PPM_ON_OPTO2
# ifdef USE_ICP1
ISR(TIMER1_CAPT_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::Capture>();
}
# else
ISR(PCINT0_vect) {
    isrRegistrar::isr<AVR::ISR::PcInt<0>>();
}
# endif
#endif
#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif

