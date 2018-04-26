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
#define USE_TC3_AS_HARDPPM

#define MEM
#define NDEBUG
#define OUTPUT

#include "local.h"
#include "rcsensorled10.h"
#include "console.h"
#include "util/meta.h"
#include "hal/eeprom.h"
#include "external/hott/menu.h"

#include <vector>

using testPin0 = AVR::Pin<PortD, 4>; // ppmout3
using testPin1 = AVR::Pin<PortD, 5>; // ppmout4

struct Storage {
    enum class AVKey : uint8_t {TSensor1 = 0, TSensor2, TSensor3, TSensor4, 
                                RpmSensor1, RpmSensor2,
                                Spannung1, ZellenZahl, 
                                StromOffset1, StromOffset2, StromOffset3, 
                                Strom1,
                                KraftOffset, KraftSteigung,
                                PWM1, PWM1MOde, 
                                PWM2, PWM2MOde, 
                                Leds1Channel, Leds1Sequence, Leds2Channel, Leds2Sequence, 
                                _Number};
    
    class ApplData : public EEProm::DataBase<ApplData> {
    public:
        uint_NaN<uint8_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        // todo: als tuple
        std::array<uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
    
    inline static constexpr uint8_t NumberOfOWireDevs = 4;
    inline static std::vector<OneWire::ow_rom_t, NumberOfOWireDevs> dsIds;
    inline static std::array<FixedPoint<int, 4>, NumberOfOWireDevs> temps;
    
    inline static constexpr uint8_t NumberOfI2CDevs = 4;
    inline static std::vector<TWI::Address, NumberOfI2CDevs> i2cDevices;
    
    inline static constexpr uint8_t NumberOfRpms = 3;
    inline static std::array<std::RPM, NumberOfRpms> rpms;
    
    inline static std::array<FixedPoint<int, 4>, 2> battMinMax;
    
    inline static constexpr uint8_t NumberOfCurrents = 3;
    inline static std::array<FixedPoint<int, 4>, NumberOfCurrents> currents;
    
    inline static StringBuffer<GPS::Sentence::TimeMaxWidth> time;
    inline static StringBuffer<GPS::Sentence::DecimalMaxWidth> speed;
    
    inline static uint8_t keepAliveCounter = 0;
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

namespace Static {
    
    template<typename Key, typename Provider>
    class TextWithValue;
    template<typename T, uint8_t L1, uint8_t LField>
    class DualValue;
    template<typename T, uint8_t L1, uint8_t LField>
    class SingleValue;
    
    class TSensorId;
    class PWMType;
    class I2CId;
    class NumberedMenu;
    class Menu;
    template<uint8_t>
    class TextItem;
    
    using MenuItem = ::UI::Static::MenuItem<
    Menu, 
    TextWithValue<::Storage::AVKey, ::Storage::ApplData>, 
    DualValue<FixedPoint<int, 4>, 5, 8>,
    SingleValue<FixedPoint<int, 4>, 5, 8>,
    DualValue<std::RPM, 5, 8>,
    SingleValue<std::RPM, 5, 8>,
    TSensorId, NumberedMenu, I2CId, PWMType, TextItem<Storage::time.size>
    >;
    
    //    MenuItem::ptype::_;
    
    template<typename Key, typename Provider>
    class TextWithValue : public MenuItem {
    public:
        TextWithValue(const PgmStringView& text, Provider& provider, Key key, uint8_t max) : 
            mTitle(text), mProvider(provider), mKey(key), mMax(max) {}
        
        void parent(ptype) {}
        bool hasChildren() const {return false;}
        
        bool isSelected() const {
            return mSelected;
        }
        void textTo(Hott::Display&) const {}
        
        void valueToText(uint8_t value, UI::span<3, char> buffer) const {
            Util::itoa_r<10>(value, buffer.mData);
        }
        
        void putTextInto(::Hott::BufferString& buffer) const {
            buffer[0] = ' ';
            buffer.insertAtFill(1, mTitle);
            
            auto& value = mProvider[mKey];
            if (value) {
                valueToText(*value, UI::make_span<18, 3>(buffer));
            }
            else {
                buffer[18] = '-';
                buffer[19] = '-';
                buffer[20] = '-';
            }
            
            if (mSelected) {
                buffer[18] |= 0x80;
                buffer[19] |= 0x80;
                buffer[20] |= 0x80;
            }
        }
        ptype processKey(Hott::key_t key) {
            auto& v = mProvider[mKey];
            switch (key) {
            case Hott::key_t::up:
                if (mSelected) {
                    if (v) {
                        if (*v > 0) {
                            --v;
                        }
                        else {
                            v.setNaN();
                        }
                    }
                    else {
                        *v = 0;
                    }
                }
                break;
            case Hott::key_t::down:
                if (mSelected) {
                    if (v) {
                        if (*v < mMax) {
                            ++v;
                        }
                        else {
                            v.setNaN();
                        }
                    }
                    else {
                        *v = 0;
                    }
                }
                break;
            case Hott::key_t::left:
                break;
            case Hott::key_t::right:
                break;
            case Hott::key_t::set:
                if (mSelected) {
                    mProvider.change();
                }
                mSelected = !mSelected;
                break;
            case Hott::key_t::nokey:
                break;
            }
            return MenuItem::make_pointer(*this);
        }
    private:
        const PgmStringView mTitle;
        Provider& mProvider;
        const Key mKey = Hott::key_t::nokey;
        bool mSelected = false;
        uint8_t mMax = 0;
    };
    
    class PWMType : public TextWithValue<Storage::AVKey, Storage::ApplData> {
    public:
        PWMType(const PgmStringView& title, Storage::ApplData& data, Storage::AVKey k) :
            TextWithValue(title, data, k, 3) {}
        void valueToText(uint8_t value, UI::span<3, char> buffer) const {
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
    
    class Menu : public MenuItem {
    public:
        Menu(const PgmStringView& title, std::initializer_list<ptype> items) : mTitle(title) {
            auto it = std::begin(items);
            for(uint8_t i = 0; i < items.size(); ++i) {
                if (it == std::end(items)) {
                    break;
                }
                mItems[i] = *it++;
                MenuItem::parent(mItems[i], MenuItem::make_pointer(*this));
            }
        }
        
        void parent(ptype p) {
            mParent = p;
        }
        
        bool hasChildren() const {return true;}
        
        bool isSelected() const {return false;}
        
        void titleInto(::Hott::BufferString& buffer) const {
            buffer.insertAtFill(0, mTitle);
        }
        
        void putTextInto(::Hott::BufferString& buffer) const {
            buffer[0] = ' ';
            buffer.insertAtFill(1, mTitle);
        }
        void textTo(Hott::Display& display) const {
            static uint_ranged_circular<uint8_t, 0, ::Hott::Display::size - 1> line;
            if (line == 0) {
                titleInto(display[0]);
                ++line;
            }
            else {
                if (lineToDisplay(display[line], line)) {
                    if (mSelectedLine && (mSelectedLine.toInt() == (line - 1))) {
                        display[mSelectedLine.toInt() + 1][0] = '>';
                    }
                    ++line;
                }
            }
        }
        bool lineToDisplay(Hott::BufferString& buffer, uint8_t row) const {
            if (mItems[row - 1]) {
                MenuItem::putTextInto(mItems[row - 1], buffer);
            }
            return true;
        }
        ptype processKey(Hott::key_t key) {
            if (mSelectedLine) {
                if (MenuItem::isSelected(mItems[mSelectedLine.toInt()])) {
                    MenuItem::processKey(mItems[mSelectedLine.toInt()], key);
                    return MenuItem::make_pointer(*this);
                }
            }
            switch (key) {
            case Hott::key_t::down:
                if (mSelectedLine) {
                    if (mItems[mSelectedLine.toInt() + 1]) {
                        ++mSelectedLine;
                    }
                }
                else {
                    mSelectedLine = 0;
                }
                break;
            case Hott::key_t::up:
                if (mSelectedLine) {
                    --mSelectedLine;
                }
                else {
                    mSelectedLine = 0;
                }
                break;
            case Hott::key_t::left:
                if (mParent) {
                    return mParent;
                }
                break;
            case Hott::key_t::right:
                break;
            case Hott::key_t::set:
                if (mSelectedLine) {
                    if (MenuItem::hasChildren(mItems[mSelectedLine.toInt()])) {
                        return mItems[mSelectedLine.toInt()];
                    }        
                    else {
                        MenuItem::processKey(mItems[mSelectedLine.toInt()], key);
                    }
                }
                break;
            case Hott::key_t::nokey:
                break;
            }
            return MenuItem::make_pointer(*this);
        }
    protected:
        ptype mParent;
        const PgmStringView mTitle;
        std::array<ptype, Hott::MenuLength> mItems{};
        uint_ranged_NaN<uint8_t, 0, Hott::MenuLength> mSelectedLine{};
    };
     
    class NumberedMenu : public Menu {
    public:
        NumberedMenu(uint8_t number, const PgmStringView& title, std::initializer_list<ptype> items) : 
            Menu(title, items), 
            mNumber(number) {}
        void titleInto(Hott::BufferString& buffer) const {
            buffer.insertAtFill(0, mTitle);
            buffer[19] =  '0' + mNumber;
        }
        
        void putTextInto(Hott::BufferString& buffer) const {
            buffer[0] = ' ';
            buffer.insertAtFill(1, mTitle);
            buffer[19] =  '0' + mNumber;
        }
    private:
        uint8_t mNumber = 0;
    };
    
    template<typename T, uint8_t L1, uint8_t LField>
    class DualValue : public MenuItem {
    public:
        DualValue(const PgmStringView& text, const T& v1, const T& v2) : mText(text), mData1{v1}, mData2{v2} {}
        
        void parent(ptype) {}
        bool hasChildren() const {return false;}
        
        bool isSelected() const {
            return false;
        }
        ptype processKey(Hott::key_t) {
            return MenuItem::make_pointer(*this);
        }
        void textTo(Hott::Display&) const {}
        
        void putTextInto(Hott::BufferString& buffer) const {
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
    
    template<typename T, uint8_t L1, uint8_t LField>
    class SingleValue : public MenuItem {
    public:
        SingleValue(const PgmStringView& text, const T& v1) : mText(text), mData1{v1} {}
        
        void parent(ptype) {}
        bool hasChildren() const {return false;}
        
        bool isSelected() const {
            return false;
        }
        ptype processKey(Hott::key_t) {
            return MenuItem::make_pointer(*this);
        }
        void textTo(Hott::Display&) const {}
        
        void putTextInto(Hott::BufferString& buffer) const {
            buffer.clear();
            buffer.insertAt(0, mText);
            putValue(mData1, UI::make_span<L1, LField>(buffer));
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
    };
    
    class TSensorId final : public MenuItem {
    public:
        TSensorId(uint8_t number) : mNumber{number} {
            assert(number < Storage::dsIds.capacity);
        }
        void parent(ptype) {}
        bool hasChildren() const {return false;}
        
        bool isSelected() const {
            return false;
        }
        ptype processKey(Hott::key_t) {
            return MenuItem::make_pointer(*this);
        }
        void textTo(Hott::Display&) const {}
        void putTextInto(Hott::BufferString& buffer) const {
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
    
    class I2CId final : public MenuItem {
    public:
        I2CId(uint8_t number) : mNumber{number} {
        }
        void parent(ptype) {}
        bool hasChildren() const {return false;}
        
        bool isSelected() const {
            return false;
        }
        ptype processKey(Hott::key_t) {
            return MenuItem::make_pointer(*this);
        }
        void textTo(Hott::Display&) const {}
        void putTextInto(Hott::BufferString& buffer) const {
            buffer.insertAtFill(0, "--:--:--:--:--:--"_pgm);
        }
    private:
        uint8_t mNumber = 0;
    };
    
    template<uint8_t L>
    class TextItem final : public MenuItem {
    public:
        TextItem(const PgmStringView& title, const StringBuffer<L>& text) : mTitle{title}, mText{text} {}
        
        void parent(ptype) {}
        bool hasChildren() const {return false;}
        
        bool isSelected() const {
            return false;
        }
        ptype processKey(Hott::key_t) {
            return MenuItem::make_pointer(*this);
        }
        void textTo(Hott::Display&) const {}
        
        void putTextInto(Hott::BufferString& buffer) const {
            buffer.insertAt(0, mTitle);
            buffer.insertAt(10, mText);
        }
    private:
        const PgmStringView mTitle;
        const StringBuffer<L>& mText;
    };
    
}

namespace {
    constexpr bool useTerminal = true;
}

namespace Constants {
    static constexpr std::hertz pwmFrequency = 100_Hz * 256; 
    static constexpr const std::hertz fSCL = 100000_Hz;
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
    static constexpr Color cOff{};
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

struct I2CInterrupt : public IsrBaseHandler<AVR::ISR::Int<2>> {
    static void isr() {
    }
};
using i2cInterruptHandler = I2CInterrupt;

using isrRegistrar = IsrRegistrar<
rcUsart::RxHandler, rcUsart::TxHandler
, sensorUsart::RxHandler, sensorUsart::TxHandler
, i2cInterruptHandler
, gpsUsart::RxHandler, gpsUsart::TxHandler
>;

using sensorData = Hott::SensorProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using menuData = Hott::SensorTextProtocollBuffer<0>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;

Static::TSensorId mTsens1{0};
Static::TSensorId mTsens2{1};
Static::TSensorId mTsens3{2};
Static::TSensorId mTsens4{3};

Static::NumberedMenu mTS1{0, "Sensor"_pgm, {Static::MenuItem::make_pointer(mTsens1)}};
Static::NumberedMenu mTS2{1, "Sensor"_pgm, {Static::MenuItem::make_pointer(mTsens2)}};
Static::NumberedMenu mTS3{2, "Sensor"_pgm, {Static::MenuItem::make_pointer(mTsens3)}};
Static::NumberedMenu mTS4{3, "Sensor"_pgm, {Static::MenuItem::make_pointer(mTsens4)}};

Static::Menu mSensoren{"Temp.Sensoren"_pgm, 
    {Static::MenuItem::make_pointer(mTS1), 
                Static::MenuItem::make_pointer(mTS2), 
                Static::MenuItem::make_pointer(mTS3), 
                Static::MenuItem::make_pointer(mTS4)
    }};

Static::TextWithValue<Storage::AVKey, Storage::ApplData> mSensor1{"Anzeige 1"_pgm, appData, Storage::AVKey::TSensor1, Storage::dsIds.capacity - 1};
Static::TextWithValue<Storage::AVKey, Storage::ApplData> mSensor2{"Anzeige 2"_pgm, appData, Storage::AVKey::TSensor2, Storage::dsIds.capacity - 1};

Static::TextWithValue<Storage::AVKey, Storage::ApplData> mRpmSensor1{"Anzeige 1"_pgm, appData, Storage::AVKey::RpmSensor1, 2};
Static::TextWithValue<Storage::AVKey, Storage::ApplData> mRpmSensor2{"Anzeige 2"_pgm, appData, Storage::AVKey::RpmSensor2, 2};

Static::TextWithValue<Storage::AVKey, Storage::ApplData> mStromOffset1{"Offset1"_pgm, appData, Storage::AVKey::StromOffset1, 100};
Static::TextWithValue<Storage::AVKey, Storage::ApplData> mStromOffset2{"Offset2"_pgm, appData, Storage::AVKey::StromOffset2, 100};
Static::TextWithValue<Storage::AVKey, Storage::ApplData> mStromOffset3{"Offset3"_pgm, appData, Storage::AVKey::StromOffset3, 100};

Static::TextWithValue<Storage::AVKey, Storage::ApplData> mPwmKanal{"Kanal"_pgm, appData, Storage::AVKey::PWM1, 8};
Static::PWMType mPwmType{"Modus"_pgm, appData, Storage::AVKey::PWM1MOde};
Static::Menu mPWM{"PWM"_pgm, {Static::MenuItem::make_pointer(mPwmKanal), Static::MenuItem::make_pointer(mPwmType)}};

Static::TextWithValue<Storage::AVKey, Storage::ApplData> mChannel1{"Kanal 1"_pgm, appData, Storage::AVKey::Leds1Channel, 8};
Static::TextWithValue<Storage::AVKey, Storage::ApplData> mSequence1{"Folge 1"_pgm, appData, Storage::AVKey::Leds1Sequence, 8};
Static::TextWithValue<Storage::AVKey, Storage::ApplData> mChannel2{"Kanal 2"_pgm, appData, Storage::AVKey::Leds2Channel, 8};
Static::TextWithValue<Storage::AVKey, Storage::ApplData> mSequence2{"Folge 2"_pgm, appData, Storage::AVKey::Leds2Sequence, 8};

Static::Menu mLeds{"LEDs"_pgm, {Static::MenuItem::make_pointer(mChannel1), Static::MenuItem::make_pointer(mSequence1), 
                Static::MenuItem::make_pointer(mChannel2), Static::MenuItem::make_pointer(mSequence2)}};

Static::I2CId mDev0{0};
Static::I2CId mDev1{1};
Static::I2CId mDev2{2};
Static::I2CId mDev3{3};

Static::NumberedMenu mI2CDev1{0, "Geraet"_pgm, {Static::MenuItem::make_pointer(mDev0)}};
Static::NumberedMenu mI2CDev2{1, "Geraet"_pgm, {Static::MenuItem::make_pointer(mDev1)}};
Static::NumberedMenu mI2CDev3{2, "Geraet"_pgm, {Static::MenuItem::make_pointer(mDev2)}};
Static::NumberedMenu mI2CDev4{3, "Geraet"_pgm, {Static::MenuItem::make_pointer(mDev3)}};
Static::Menu mDevs{"Geraete"_pgm, {
        Static::MenuItem::make_pointer(mI2CDev1), 
                Static::MenuItem::make_pointer(mI2CDev2), 
                Static::MenuItem::make_pointer(mI2CDev3), 
                Static::MenuItem::make_pointer(mI2CDev4)
    }};

Static::DualValue<FixedPoint<int, 4>, 5, 8> mTempLine1{"T1/2"_pgm, Storage::temps[0], Storage::temps[1]};
Static::DualValue<FixedPoint<int, 4>, 5, 8> mTempLine2{"T3/4"_pgm, Storage::temps[2], Storage::temps[3]};
Static::DualValue<std::RPM, 5, 8> mRpmLine1{"R1/2"_pgm, Storage::rpms[0], Storage::rpms[1]};
Static::SingleValue<std::RPM, 5, 8> mRpmLine2{"R3"_pgm, Storage::rpms[2]};
Static::DualValue<FixedPoint<int, 4>, 5, 8> mBattLine{"Bm/M"_pgm, Storage::battMinMax[0], Storage::battMinMax[1]};
Static::DualValue<FixedPoint<int, 4>, 5, 8> mCurrentLine1{"A1/2"_pgm, Storage::currents[0], Storage::currents[1]};
Static::SingleValue<FixedPoint<int, 4>, 5, 8> mCurrentLine2{"A3"_pgm, Storage::currents[2]};

Static::Menu mInfo{"Uebersicht"_pgm, {
        Static::MenuItem::make_pointer(mTempLine1), 
                Static::MenuItem::make_pointer(mTempLine2), 
                Static::MenuItem::make_pointer(mRpmLine1), 
                Static::MenuItem::make_pointer(mRpmLine2), 
                Static::MenuItem::make_pointer(mBattLine), 
                Static::MenuItem::make_pointer(mCurrentLine1), 
                Static::MenuItem::make_pointer(mCurrentLine2)
    }};
Static::Menu mTemperatur{"Temperatur"_pgm, {
        Static::MenuItem::make_pointer(mSensor1), 
                Static::MenuItem::make_pointer(mSensor2), 
                Static::MenuItem::make_pointer(mSensoren)
    }};                        
Static::Menu mDrehzahl{"Drehzahl"_pgm, {
        Static::MenuItem::make_pointer(mRpmSensor1), 
                Static::MenuItem::make_pointer(mRpmSensor2)
    }};             
Static::Menu mStrom{"Strom"_pgm, {
        Static::MenuItem::make_pointer(mStromOffset1),
                Static::MenuItem::make_pointer(mStromOffset2),
                Static::MenuItem::make_pointer(mStromOffset3)
    }};                        
Static::Menu mActors{"Aktoren"_pgm, {
        Static::MenuItem::make_pointer(mPWM), 
                Static::MenuItem::make_pointer(mLeds), 
                Static::MenuItem::make_pointer(mDevs)
    }};

Static::TextItem<Storage::time.size> gpsTime("Time"_pgm, Storage::time);

Static::Menu mTest{"Status"_pgm, {
        Static::MenuItem::make_pointer(gpsTime)
    }};                        

Static::Menu topMenu{"WM SensMod HW10 SW24"_pgm, {
        Static::MenuItem::make_pointer(mInfo), 
                Static::MenuItem::make_pointer(mTemperatur), 
                Static::MenuItem::make_pointer(mDrehzahl),
                Static::MenuItem::make_pointer(mStrom), 
                Static::MenuItem::make_pointer(mActors),
                Static::MenuItem::make_pointer(mTest)
    }};


template<typename PA>
class HottMenu final {
    HottMenu() = delete;
    
    using MenuItem = Static::MenuItem;
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
        MenuItem::textTo(mMenu, PA::text());
    }
    inline static void processKey(Hott::key_t key) {
        assert(mMenu);
        if (auto m = MenuItem::processKey(mMenu, key); m != mMenu) {
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
    inline static auto mMenu = MenuItem::make_pointer(topMenu);
};

using menu = HottMenu<menuData>;


struct AsciiHandler {
    static void start() {
        crWriterSensorText::enable<true>();
    }    
    static void stop() {
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
#ifdef OUTPUT
                std::outl<terminal>("Temp1: "_pgm, Sensor::temperature());
#endif
            }
            if (mSensorNumber == 1) {
                sensorData::temp2(Sensor::temperature());
#ifdef OUTPUT
                std::outl<terminal>("Temp2: "_pgm, Sensor::temperature());
#endif
            }
            if (mSensorNumber == 2) {
#ifdef OUTPUT
                std::outl<terminal>("Temp3: "_pgm, Sensor::temperature());
#endif
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
    
    inline static constexpr uint8_t parts = 5;
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
        uint16_t v1 = adcController::value(0) * 4;
        uint16_t v2 = adcController::value(1) * 8;
        uint16_t v3 = adcController::value(2) * 12;
        uint16_t v4 = adcController::value(3) * 16;
        uint16_t v5 = adcController::value(4) * 20;
        
        uint16_t z1 = Util::RationalDivider<uint16_t, hottScale, rawMax>::scale(v1);
        uint16_t z2 = Util::RationalDivider<uint16_t, hottScale, rawMax>::scale(v2 - v1);
        uint16_t z3 = Util::RationalDivider<uint16_t, hottScale, rawMax>::scale(v3 - v2);
        uint16_t z4 = Util::RationalDivider<uint16_t, hottScale, rawMax>::scale(v4 - v3);
        uint16_t z5 = Util::RationalDivider<uint16_t, hottScale, rawMax>::scale(v5 - v4);
        
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
        if ((z4 > 0) && (z4 < zmin)) {
            zmin = z4;
            zminNum = 3;
        }
        if ((z5 > 0) && (z5 < zmin)) {
            zmin = z5;
            zminNum = 4;
        }
        sensorData::cellVoltageRaw(0, z1);
        sensorData::cellVoltageRaw(1, z2);
        sensorData::cellVoltageRaw(2, z3);
        sensorData::cellVoltageRaw(3, z4);
        sensorData::cellVoltageRaw(4, z5);
        
        uint16_t b1 = Util::maximum(v1, v2, v3, v4, v5);
        uint16_t zmax = Util::maximum(z1, z2, z3, z4, z5);
        
        uint16_t batt = Util::RationalDivider<uint16_t, battScale, rawMax>::scale(b1);
        
        sensorData::batteryVoltageRaw(0, batt);
        sensorData::mainVoltageRaw(batt);
        
        Storage::battMinMax[0] = FixedPoint<int, 4>::fromRaw(Util::RationalDivider<uint16_t, 16, 50>::scale(zmin) + zmin);
        Storage::battMinMax[1] = FixedPoint<int, 4>::fromRaw(Util::RationalDivider<uint16_t, 16, 50>::scale(zmax) + zmax);
        
        if (zminNum) {
            sensorData::batteryMinimumRaw(zminNum.toInt(), zmin);
        }
    }
    static inline void update1() {
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
        
        uint16_t c1 = Util::RationalDivider<uint16_t, 98, 100>::scale(a1) + a1 + a1;
        
        sensorData::currentRaw(c1);
    }
    static inline void update3() {
        Storage::rpms[0] = rpm1::rpm();
        Storage::rpms[1] = rpm2::rpm();
        Storage::rpms[2] = rpm3::rpm();
        
        if (const auto& s = appData[Storage::AVKey::RpmSensor1]) {
            sensorData::rpm1(Util::select(s.toInt(), Storage::rpms[0], Storage::rpms[1], Storage::rpms[2]));
        }
        if (const auto& s = appData[Storage::AVKey::RpmSensor2]) {
            sensorData::rpm2(Util::select(s.toInt(), Storage::rpms[0], Storage::rpms[1], Storage::rpms[2]));
        }
    }
    static inline void update4() {
        auto s = Util::StringConverter<FixedPoint<int16_t, 4>>::parse(Storage::speed);
        sensorData::speedRaw(s.raw());
        
        sensorData::forceRaw(hx711::value());
    }

    inline static uint_ranged_circular<uint8_t, 0, parts - 1> part;
    inline static uint16_t zmin = std::numeric_limits<uint16_t>::max();
    inline static uint_NaN<uint8_t> zminNum;
};

inline void updateActors() {
    static uint_ranged_circular<uint8_t, 0, 3> part;
    switch(part) {
    case 0: 
    {
        auto v1 = Hott::SumDProtocollAdapter<0>::value(1);
#ifdef USE_TC1_AS_HARDPPM
        hardPpm1::ppm<hardPpm1::A>(v1);
#endif
    }
        break;
    case 1:
    {
        auto v3 = Hott::SumDProtocollAdapter<0>::value(3);
#ifdef USE_TC1_AS_HARDPPM
        hardPpm1::ppm<hardPpm1::B>(v3);
#endif
    }
        break;
    case 2:
    {
        auto v0 = Hott::SumDProtocollAdapter<0>::value(0);
        //        decltype(v0)::_;
        hbridge1::pwm(v0);
    }
        break;
    case 3:
    {
        auto v0 = Hott::SumDProtocollAdapter<0>::value(1);
        hbridge2::pwm(v0);
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
        mcp23008::startWrite(0x09, std::byte{Storage::keepAliveCounter});
    }
}

int main() {
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    testPin0::dir<AVR::Output>();
    testPin0::on();
    
    testPin1::dir<AVR::Output>();
    testPin1::on();
    
    isrRegistrar::init();
    
    eeprom::init();
    adcController::init();
    
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    //    ppmDecoder::init();
    
    leds::init();
    leds::off();
    
    leds2::init();
    leds2::off();
    
    Util::delay(1_ms);    
    leds::set(0, Constants::cGreen);
    
    Util::delay(1_ms);    
    leds2::set(0, Constants::cOff);
    
    Util::delay(1_ms);    
    ledFSM::init();
    
    crWriterSensorBinary::init();
    crWriterSensorText::init();
    
#ifdef USE_TC1_AS_HARDPPM
    hardPpm1::init();
#endif
#ifdef USE_TC3_AS_HARDPPM
    hardPpm2::init();
#endif
    rpm1::init();
    rpm2::init();
    rpm3::init();
    
    hardPwm::init<Constants::pwmFrequency>();
    hbridge1::init();
    hbridge2::init();
    
    hx711::init();
    
    ds18b20::init();
    
    TwiMaster::init<Constants::fSCL>();
    
    gpsUsart::init<9600>();
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test24"_pgm);
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
        {
            std::outl<terminal>("I2C:"_pgm);
            std::array<TWI::Address, Storage::i2cDevices.capacity> i2cAddresses;
            TwiMaster::findDevices(i2cAddresses);
            for(const auto& d : i2cAddresses) {
                if (d) {
                    Storage::i2cDevices.push_back(d);
                    std::outl<terminal>(d);
                    Util::delay(100_ms);
                }
            }
        }
        
        mcp23008::startWrite(0x00, std::byte{0x00}); // output
        mcp23008::startWrite(0x09, std::byte{0x00}); // output
        
        if (!oled::init()) {
            std::outl<terminal>("oled error"_pgm);
        }
        
        std::outl<terminal>("hx711 max: "_pgm, hx711::max);
        
        std::outl<terminal>("ocmin: "_pgm, hardPpm1::parameter::ocMin);
        std::outl<terminal>("ocmax: "_pgm, hardPpm1::parameter::ocMax);
        std::outl<terminal>("freq: "_pgm, hardPpm1::parameter::timerFrequency);
        
        oled::clear();
        
        tempFSM::init();
        
        const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);
        
        while(true){
            testPin0::toggle(); // max 135 uS ein Intervall
            TwiMasterAsync::periodic();
            menu::periodic();
            rpm1::periodic();
            rpm2::periodic();
            rpm3::periodic();
            hx711::periodic();
            adcController::periodic();
            tempFSM::periodic();
            eeprom::saveIfNeeded();
            
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
                        ++Storage::keepAliveCounter;
                        rpm1::check();
                        rpm2::check();
                        rpm3::check();
                        GPS::RMC::timeRaw(Storage::time);
                        GPS::VTG::speedRaw(Storage::speed);
                        auto v0 = Hott::SumDProtocollAdapter<0>::value(0);
                        uint16_t a = adcController::value(6);
#ifdef OUTPUT
                        if (Hott::SumDProtocollAdapter<0>::hasMultiChannel()) {
                            std::out<terminal>("M["_pgm, Hott::SumDProtocollAdapter<0>::mChannelForMultiChannel, "] "_pgm);
                            for(uint8_t i = 0; i < Hott::MultiChannel::size; ++i) {
                                std::out<terminal>(Char{' '}, (uint8_t)Hott::SumDProtocollAdapter<0>::mChannel(i));
                            }
                            std::outl<terminal>(Char{' '});
                        }
                        std::outl<terminal>("W: "_pgm, hx711::value());
                        std::outl<terminal>("v: "_pgm, v0.toInt());
                        std::outl<terminal>("t: "_pgm, Storage::time);
                        std::outl<terminal>("a: "_pgm, a);
                        std::outl<terminal>("r: "_pgm, Storage::rpms[2]);
#ifdef MEM
                        std::outl<terminal>("m: "_pgm, Util::Memory::getUnusedMemory());
#endif
#ifdef USE_PPM_ON_OPTO2
                        std::outl<terminal>("ppm: "_pgm, ppmDecoder::value(0));
#endif
#endif
                    }
                    else {
                        tempFSM::tick(timer);
                        ledFSM::tick(timer);
                    }
                });
                appData.expire();
            });
        }
    }    
}
ISR(INT2_vect) {
    isrRegistrar::isr<AVR::ISR::Int<2>>();
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
    //    while(true) {}
}
#endif
