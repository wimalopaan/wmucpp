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

#define USE_SPI_AS_TEST // use SPI-Pins as test signals

#define USE_TC1_AS_HARDPPM // ppm3/4
#define USE_TC3_AS_HARDPPM // ppm1/2

//#define MEM
#define NDEBUG
#define OUTPUT

#include "board.h"

using namespace AVR;
using namespace etl;
using namespace std;

using namespace External::Units;

#ifdef USE_SPI_AS_TEST
using testPin1 = Pin<PortC, 4>; 
using testPin2 = Pin<PortC, 5>; 
using testPin3 = Pin<PortC, 6>; 
using testPin4 = Pin<PortC, 7>; 
#endif

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
    inline static FixedVector<OneWire::ow_rom_t, NumberOfOWireDevs> dsIds;
    inline static array<FixedPoint<int, 4>, NumberOfOWireDevs> temps;
    
    inline static constexpr uint8_t NumberOfRpms = 3;
    inline static std::array<RPM, NumberOfRpms> rpms;
    
    inline static std::array<FixedPoint<int, 4>, 2> battMinMax;
    
    inline static constexpr uint8_t NumberOfCurrents = 3;
    inline static std::array<FixedPoint<int, 4>, NumberOfCurrents> currents;
    
    inline static StringBuffer<External::GPS::Sentence::TimeMaxWidth> time;
    inline static StringBuffer<External::GPS::Sentence::DecimalMaxWidth> speed;
    
    inline static uint8_t keepAliveCounter = 0;
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

namespace Static {
    
    class PWMType : public Hott::TextWithValue<Storage::AVKey, Storage::ApplData> {
    public:
        PWMType(const PgmStringView& title, Storage::ApplData& data, Storage::AVKey k) :
            TextWithValue(title, data, k, 3) {}
        virtual void valueToText(uint8_t value, etl::span<3, etl::Char> buffer) const override{
            if (value == 0) {
                buffer.insertLeftFill("V"_pgm);
            }
            else if (value == 1) {
                buffer.insertLeftFill("R"_pgm);
            }
            else if (value == 2) {
                buffer.insertLeftFill("V/R"_pgm);
            }
        }
    private:
    };
    
    
    //    template<typename T, uint8_t L1, uint8_t LField>
    //    class DualValue : public MenuItem {
    //    public:
    //        DualValue(const PgmStringView& text, const T& v1, const T& v2) : mText(text), mData1{v1}, mData2{v2} {}
    
    //        void parent(ptype) {}
    //        bool hasChildren() const {return false;}
    
    //        bool isSelected() const {
    //            return false;
    //        }
    //        ptype processKey(Hott::key_t) {
    //            return MenuItem::make_pointer(*this);
    //        }
    //        void textTo(Hott::Display&) const {}
    
    //        void putTextInto(Hott::BufferString& buffer) const {
    //            buffer.clear();
    //            buffer.insertAt(0, mText);
    //            putValue(mData1, UI::make_span<L1, LField>(buffer));
    //            putValue(mData2, UI::make_span<L1 + LField, LField>(buffer));
    //        }
    //    private:
    //        template<typename I, uint8_t F>
    //        inline void putValue(const FixedPoint<I, F>& value, UI::span<LField, char> b) const{
    //            uint8_t i = value.integerAbs();
    //            Util::itoa_r(i, b);
    //            auto f = value.fraction();
    //            auto b2 = UI::make_span<3, 5>(b);
    //            Util::ftoa(f, b2);
    //        }
    //        inline void putValue(const std::RPM& rpm, UI::span<LField, char> b) const {
    //            Util::itoa_r(rpm.value(), b);
    //        }
    //        const PgmStringView mText;
    //        const T& mData1;
    //        const T& mData2;
    //    };
    
    //    template<typename T, uint8_t L1, uint8_t LField>
    //    class SingleValue : public MenuItem {
    //    public:
    //        SingleValue(const PgmStringView& text, const T& v1) : mText(text), mData1{v1} {}
    
    //        void parent(ptype) {}
    //        bool hasChildren() const {return false;}
    
    //        bool isSelected() const {
    //            return false;
    //        }
    //        ptype processKey(Hott::key_t) {
    //            return MenuItem::make_pointer(*this);
    //        }
    //        void textTo(Hott::Display&) const {}
    
    //        void putTextInto(Hott::BufferString& buffer) const {
    //            buffer.clear();
    //            buffer.insertAt(0, mText);
    //            putValue(mData1, UI::make_span<L1, LField>(buffer));
    //        }
    //    private:
    //        template<typename I, uint8_t F>
    //        inline void putValue(const FixedPoint<I, F>& value, UI::span<LField, char> b) const{
    //            uint8_t i = value.integerAbs();
    //            Util::itoa_r(i, b);
    //            auto f = value.fraction();
    //            auto b2 = UI::make_span<3, 5>(b);
    //            Util::ftoa(f, b2);
    //        }
    //        inline void putValue(const std::RPM& rpm, UI::span<LField, char> b) const {
    //            Util::itoa_r(rpm.value(), b);
    //        }
    //        const PgmStringView mText;
    //        const T& mData1;
    //    };
    
    class TSensorId final : public Hott::MenuItem {
    public:
        TSensorId(uint8_t number) : mNumber{number} {
            assert(number < Storage::dsIds.capacity);
        }
        //        void parent(ptype) {}
        //        bool hasChildren() const {return false;}
        
        //        bool isSelected() const {
        //            return false;
        //        }
        //        ptype processKey(Hott::key_t) {
        //            return MenuItem::make_pointer(*this);
        //        }
        //        void textTo(Hott::Display&) const {}
        
        virtual void putTextInto(Hott::BufferString& buffer) const {
            if (auto id = Storage::dsIds[mNumber]) {
                for(uint8_t i = 0; i < (id.size() - 2); ++i) {
                    uint8_t d = static_cast<uint8_t>(id[i + 1]);
                    auto s = etl::make_span<2>(i * 3, buffer);
                    etl::itoa_r<16>(d, s);
                    //                    etl::itoa_r<16>(d, &buffer[i * 3]);
                }
                for(uint8_t i = 0; i < (id.size() - 3); ++i) {
                    buffer[i * 3 + 2] = etl::Char{':'};
                }
            }
            else {
                buffer.insertAtFill(0, "--:--:--:--:--:--"_pgm);
            }
        }
    private:
        uint8_t mNumber = 0;
    };
    
    template<uint8_t L>
    class TextItem final : public Hott::MenuItem {
    public:
        TextItem(const PgmStringView& title, const StringBuffer<L>& text) : mTitle{title}, mText{text} {}
        
        //        void parent(ptype) {}
        //        bool hasChildren() const {return false;}
        
        //        bool isSelected() const {
        //            return false;
        //        }
        //        ptype processKey(Hott::key_t) {
        //            return MenuItem::make_pointer(*this);
        //        }
        //        void textTo(Hott::Display&) const {}
        
        void putTextInto(Hott::BufferString& buffer) const {
            buffer.insertAt(0, mTitle);
            buffer.insertAt(10, mText);
        }
    private:
        const PgmStringView mTitle;
        const StringBuffer<L>& mText;
    };
    
}

namespace Constants {
    using namespace External;
    using namespace External::Units;
    using namespace External::Units::literals;
    
    
    static constexpr hertz pwmFrequency = 100_Hz * 256; 
    static constexpr hertz fSCL = 100000_Hz;
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
    static constexpr Color cOff{};
}

struct AsciiHandler;
struct BinaryHandler;
struct BCastHandler;

using terminalDevice = rcUsart;;
using terminal = etl::basic_ostream<terminalDevice>;

//using isrRegistrar = IsrRegistrar<>;

using sensorData = Hott::GamProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using menuData = Hott::SensorTextProtocollBuffer<Hott::gam_code, 0>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;

Static::TSensorId mTsens1{0};
Static::TSensorId mTsens2{1};
Static::TSensorId mTsens3{2};
Static::TSensorId mTsens4{3};

//Hott::NumberedMenu mTS1{0, "Sensor"_pgm, {Static::MenuItem::make_pointer(mTsens1)}};
//Static::NumberedMenu mTS2{1, "Sensor"_pgm, {Static::MenuItem::make_pointer(mTsens2)}};
//Static::NumberedMenu mTS3{2, "Sensor"_pgm, {Static::MenuItem::make_pointer(mTsens3)}};
//Static::NumberedMenu mTS4{3, "Sensor"_pgm, {Static::MenuItem::make_pointer(mTsens4)}};

//Static::Menu mSensoren{"Temp.Sensoren"_pgm, 
//    {Static::MenuItem::make_pointer(mTS1), 
//                Static::MenuItem::make_pointer(mTS2), 
//                Static::MenuItem::make_pointer(mTS3), 
//                Static::MenuItem::make_pointer(mTS4)
//    }};

//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mSensor1{"Anzeige 1"_pgm, appData, Storage::AVKey::TSensor1, Storage::dsIds.capacity - 1};
//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mSensor2{"Anzeige 2"_pgm, appData, Storage::AVKey::TSensor2, Storage::dsIds.capacity - 1};

//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mRpmSensor1{"Anzeige 1"_pgm, appData, Storage::AVKey::RpmSensor1, 2};
//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mRpmSensor2{"Anzeige 2"_pgm, appData, Storage::AVKey::RpmSensor2, 2};

//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mStromOffset1{"Offset1"_pgm, appData, Storage::AVKey::StromOffset1, 100};
//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mStromOffset2{"Offset2"_pgm, appData, Storage::AVKey::StromOffset2, 100};
//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mStromOffset3{"Offset3"_pgm, appData, Storage::AVKey::StromOffset3, 100};

//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mPwm1Kanal{"Kanal1"_pgm, appData, Storage::AVKey::PWM1, 8};
//Static::PWMType mPwm1Type{"Modus1"_pgm, appData, Storage::AVKey::PWM1MOde};
//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mPwm2Kanal{"Kanal2"_pgm, appData, Storage::AVKey::PWM2, 8};
//Static::PWMType mPwm2Type{"Modus2"_pgm, appData, Storage::AVKey::PWM2MOde};
//Static::Menu mPWM{"PWM"_pgm, {
//        Static::MenuItem::make_pointer(mPwm1Kanal), Static::MenuItem::make_pointer(mPwm1Type),
//        Static::MenuItem::make_pointer(mPwm2Kanal), Static::MenuItem::make_pointer(mPwm2Type)
//    }
//};

//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mChannel1{"Kanal 1"_pgm, appData, Storage::AVKey::Leds1Channel, 8};
//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mSequence1{"Folge 1"_pgm, appData, Storage::AVKey::Leds1Sequence, 8};
//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mChannel2{"Kanal 2"_pgm, appData, Storage::AVKey::Leds2Channel, 8};
//Static::TextWithValue<Storage::AVKey, Storage::ApplData> mSequence2{"Folge 2"_pgm, appData, Storage::AVKey::Leds2Sequence, 8};

//Static::Menu mLeds{"LEDs"_pgm, {Static::MenuItem::make_pointer(mChannel1), Static::MenuItem::make_pointer(mSequence1), 
//                Static::MenuItem::make_pointer(mChannel2), Static::MenuItem::make_pointer(mSequence2)}};

//Static::I2CId mDev0{0};
//Static::I2CId mDev1{1};
//Static::I2CId mDev2{2};
//Static::I2CId mDev3{3};

//Static::NumberedMenu mI2CDev1{0, "Geraet"_pgm, {Static::MenuItem::make_pointer(mDev0)}};
//Static::NumberedMenu mI2CDev2{1, "Geraet"_pgm, {Static::MenuItem::make_pointer(mDev1)}};
//Static::NumberedMenu mI2CDev3{2, "Geraet"_pgm, {Static::MenuItem::make_pointer(mDev2)}};
//Static::NumberedMenu mI2CDev4{3, "Geraet"_pgm, {Static::MenuItem::make_pointer(mDev3)}};
//Static::Menu mDevs{"Geraete"_pgm, {
//        Static::MenuItem::make_pointer(mI2CDev1), 
//                Static::MenuItem::make_pointer(mI2CDev2), 
//                Static::MenuItem::make_pointer(mI2CDev3), 
//                Static::MenuItem::make_pointer(mI2CDev4)
//    }};

//Static::DualValue<FixedPoint<int, 4>, 5, 8> mTempLine1{"T1/2"_pgm, Storage::temps[0], Storage::temps[1]};
//Static::DualValue<FixedPoint<int, 4>, 5, 8> mTempLine2{"T3/4"_pgm, Storage::temps[2], Storage::temps[3]};
//Static::DualValue<std::RPM, 5, 8> mRpmLine1{"R1/2"_pgm, Storage::rpms[0], Storage::rpms[1]};
//Static::SingleValue<std::RPM, 5, 8> mRpmLine2{"R3"_pgm, Storage::rpms[2]};
//Static::DualValue<FixedPoint<int, 4>, 5, 8> mBattLine{"Bm/M"_pgm, Storage::battMinMax[0], Storage::battMinMax[1]};
//Static::DualValue<FixedPoint<int, 4>, 5, 8> mCurrentLine1{"A1/2"_pgm, Storage::currents[0], Storage::currents[1]};
//Static::SingleValue<FixedPoint<int, 4>, 5, 8> mCurrentLine2{"A3"_pgm, Storage::currents[2]};

//Static::Menu mInfo{"Uebersicht"_pgm, {
//        Static::MenuItem::make_pointer(mTempLine1), 
//                Static::MenuItem::make_pointer(mTempLine2), 
//                Static::MenuItem::make_pointer(mRpmLine1), 
//                Static::MenuItem::make_pointer(mRpmLine2), 
//                Static::MenuItem::make_pointer(mBattLine), 
//                Static::MenuItem::make_pointer(mCurrentLine1), 
//                Static::MenuItem::make_pointer(mCurrentLine2)
//    }};
//Static::Menu mTemperatur{"Temperatur"_pgm, {
//        Static::MenuItem::make_pointer(mSensor1), 
//                Static::MenuItem::make_pointer(mSensor2), 
//                Static::MenuItem::make_pointer(mSensoren)
//    }};                        
//Static::Menu mDrehzahl{"Drehzahl"_pgm, {
//        Static::MenuItem::make_pointer(mRpmSensor1), 
//                Static::MenuItem::make_pointer(mRpmSensor2)
//    }};             
//Static::Menu mStrom{"Strom"_pgm, {
//        Static::MenuItem::make_pointer(mStromOffset1),
//                Static::MenuItem::make_pointer(mStromOffset2),
//                Static::MenuItem::make_pointer(mStromOffset3)
//    }};                        
//Static::Menu mActors{"Aktoren"_pgm, {
//        Static::MenuItem::make_pointer(mPWM), 
//                Static::MenuItem::make_pointer(mLeds), 
//                Static::MenuItem::make_pointer(mDevs)
//    }};

//Static::TextItem<Storage::time.size> gpsTime("Time"_pgm, Storage::time);

//Static::Menu mStatus{"Status"_pgm, {
//        Static::MenuItem::make_pointer(gpsTime)
//    }};                        

//Static::Menu topMenu{"WM SensMod HW10 SW25"_pgm, {
//        Static::MenuItem::make_pointer(mInfo), 
//                Static::MenuItem::make_pointer(mTemperatur), 
//                Static::MenuItem::make_pointer(mDrehzahl),
//                Static::MenuItem::make_pointer(mStrom), 
//                Static::MenuItem::make_pointer(mActors),
//                Static::MenuItem::make_pointer(mStatus)
//    }};


//template<typename PA>
//class HottMenu final {
//    HottMenu() = delete;

//    using MenuItem = Static::MenuItem;
//public:
//    inline static void periodic() {
//        Hott::key_t k = Hott::key_t::nokey;
//        {
//            Scoped<DisbaleInterrupt<>> di;
//            k = mKey;
//            mKey = Hott::key_t::nokey;
//        }
//        if (k != Hott::key_t::nokey) {
//            processKey(k);
//        }
//        MenuItem::textTo(mMenu, PA::text());
//    }
//    inline static void processKey(Hott::key_t key) {
//        assert(mMenu);
//        if (auto m = MenuItem::processKey(mMenu, key); m != mMenu) {
//            mMenu = m;
//            for(auto& line : PA::text()) {
//                line.clear();
//            }
//        }
//    }
//    inline static void isrKey(std::byte b) {
//        mKey = Hott::key_t{b};
//    }
//private:
//    inline static volatile Hott::key_t mKey = Hott::key_t::nokey;
//    inline static auto mMenu = MenuItem::make_pointer(topMenu);
//};

//using menu = HottMenu<menuData>;

class Dashboard final : public Hott::Menu {
public:
    Dashboard() : Menu(this, "Dashboard"_pgm
                       ) {}
private:
    //    Hott::TextWithValue<Storage::AVKey, 3, Storage::ApplData> mMode{"Mode"_pgm, appData, Storage::AVKey::SplitMode};
};

class RCMenu final : public Hott::Menu {
public:
    RCMenu() : Menu(this, "RCSL 0.1"_pgm,
                    &mDashboard
                    ) {}
private:
    Dashboard mDashboard;
};

RCMenu topMenu; // note: global object instead of static in Hottmenu -> no guards are created (is this a g++ error)

template<typename PA>
class HottMenu final {
    HottMenu() = delete;
public:
    inline static void init() {
        clear();
    }
    inline static void periodic() {
        Hott::key_t k = Hott::key_t::nokey;
        {
            etl::Scoped<etl::DisbaleInterrupt<>> di;
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
            clear();
        }
    }
    inline static void isrKey(std::byte b) {
        mKey = Hott::key_t{b};
    }
private:
    inline static void clear() {
        for(auto& line : PA::text()) {
            line.clear();
        }
    }
    inline static /*volatile*/ Hott::key_t mKey = Hott::key_t::nokey;
    inline static Hott::Menu* mMenu = &topMenu;
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
        etl::outl<terminal>("hbr start"_pgm);
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
        mMeasureTimer = Alarm::create(750_ms, Alarm::flags_type::Disabled | Alarm::flags_type::OneShot);
        mTempTimer = Alarm::create(3000_ms, Alarm::flags_type::Periodic);
    }
    inline static void tick(alarmTimer::index_type timer) {
        if (timer == mMeasureTimer) {
            process(Event::WaitOver);
        }
        else if (timer == mTempTimer) {
            process(Event::Measure);
        }
    }
    inline static void periodic() {
        Sensor::periodic([]{
            mState = State::Start;
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
                alarmTimer::start(mMeasureTimer);
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
    inline static uint_NaN<uint8_t> mMeasureTimer;
    inline static uint_NaN<uint8_t> mTempTimer;
};

using tempFSM = TempFSM<ds18b20, Storage, alarmTimer>;

template<typename LEDs, typename Alarm, uint8_t Offset = 1>
class LedFSM {
    enum class State : uint8_t {};
public:
    inline static void init() {
        mTimer = Alarm::create(1000_ms, Alarm::flags_type::Periodic);
    }
    inline static void tick(alarmTimer::index_type timer) {
        if (timer == mTimer) {
            LEDs::template set<false>(Color{});
            LEDs::set(mActual, Constants::cBlue);
            ++mActual;
        }
    }
private:    
    inline static etl::uint_NaN<uint8_t> mTimer;
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
//        uint16_t v1 = adcController::value(0) * 4;
//        uint16_t v2 = adcController::value(1) * 8;
//        uint16_t v3 = adcController::value(2) * 12;
//        uint16_t v4 = adcController::value(3) * 16;
//        uint16_t v5 = adcController::value(4) * 20;
        
//        uint16_t z1 = etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(v1);
//        uint16_t z2 = etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(v2 - v1);
//        uint16_t z3 = etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(v3 - v2);
//        uint16_t z4 = etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(v4 - v3);
//        uint16_t z5 = etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(v5 - v4);
        
//        if ((z1 > 0) && (z1 < zmin)) {
//            zmin = z1;
//            zminNum = 0;
//        }
//        if ((z2 > 0) && (z2 < zmin)) {
//            zmin = z2;
//            zminNum = 1;
//        }
//        if ((z3 > 0) && (z3 < zmin)) {
//            zmin = z3;
//            zminNum = 2;
//        }
//        if ((z4 > 0) && (z4 < zmin)) {
//            zmin = z4;
//            zminNum = 3;
//        }
//        if ((z5 > 0) && (z5 < zmin)) {
//            zmin = z5;
//            zminNum = 4;
//        }
//        sensorData::cellVoltageRaw(0, z1);
//        sensorData::cellVoltageRaw(1, z2);
//        sensorData::cellVoltageRaw(2, z3);
//        sensorData::cellVoltageRaw(3, z4);
//        sensorData::cellVoltageRaw(4, z5);
        
//        uint16_t b1 = etl::maximum(v1, v2, v3, v4, v5);
//        uint16_t zmax = etl::maximum(z1, z2, z3, z4, z5);
        
//        uint16_t batt = etl::Rational::RationalDivider<uint16_t, battScale, rawMax>::scale(b1);
        
//        sensorData::batteryVoltageRaw(0, batt);
//        sensorData::mainVoltageRaw(batt);
        
//        Storage::battMinMax[0] = FixedPoint<int, 4>::fromRaw(etl::Rational::RationalDivider<uint16_t, 16, 50>::scale(zmin) + zmin);
//        Storage::battMinMax[1] = FixedPoint<int, 4>::fromRaw(etl::Rational::RationalDivider<uint16_t, 16, 50>::scale(zmax) + zmax);
        
//        if (zminNum) {
//            sensorData::batteryMinimumRaw(zminNum.toInt(), zmin);
//        }
    }
    static inline void update1() {
        if (const auto& s = appData[Storage::AVKey::TSensor1]) {
            sensorData::temp1(etl::select(s.toInt(), Storage::temps[0], Storage::temps[1], Storage::temps[2], Storage::temps[3]));
        }
        if (const auto& s = appData[Storage::AVKey::TSensor2]) {
            sensorData::temp2(etl::select(s.toInt(), Storage::temps[0], Storage::temps[1], Storage::temps[2], Storage::temps[3]));
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
        
        uint16_t c1 = etl::Rational::RationalDivider<uint16_t, 98, 100>::scale(a1) + a1 + a1;
        
        sensorData::currentRaw(c1);
    }
    static inline void update3() {
#ifndef USE_TC1_AS_HARDPPM
        Storage::rpms[0] = rpm1::rpm();
#endif
#ifndef USE_TC3_AS_HARDPPM
        Storage::rpms[1] = rpm2::rpm();
#endif
        //        Storage::rpms[2] = rpm3::rpm();
        
        if (const auto& s = appData[Storage::AVKey::RpmSensor1]) {
            sensorData::rpm1(etl::select(s.toInt(), Storage::rpms[0], Storage::rpms[1], Storage::rpms[2]));
        }
        if (const auto& s = appData[Storage::AVKey::RpmSensor2]) {
            sensorData::rpm2(etl::select(s.toInt(), Storage::rpms[0], Storage::rpms[1], Storage::rpms[2]));
        }
    }
    static inline void update4() {
        auto s = etl::StringConverter<FixedPoint<int16_t, 4>>::parse(Storage::speed);
        sensorData::speedRaw(s.raw());
        //        sensorData::forceRaw(hx711::value());
    }
    
    inline static uint_ranged_circular<uint8_t, 0, parts - 1> part;
    inline static uint16_t zmin = std::numeric_limits<uint16_t>::max();
    inline static uint_NaN<uint8_t> zminNum;
};

struct Actors {
    inline static uint_ranged_circular<uint8_t, 0, 7> part;
    inline static void update() {
        switch(part) {
        case 0: 
        {
#ifdef USE_TC3_AS_HARDPPM
            auto v1 = Hott::SumDProtocollAdapter<0>::value(1);
            //            hardPpm2::ppm<hardPpm2::A>(v1);
#endif
        }
            break;
        case 1:
        {
#ifdef USE_TC3_AS_HARDPPM
            auto v3 = Hott::SumDProtocollAdapter<0>::value(3);
            //            hardPpm2::ppm<hardPpm2::B>(v3);
#endif
        }
            break;
        case 2:
        {
            auto v0 = Hott::SumDProtocollAdapter<0>::value(0);
            //            hbridge1::pwm(v0);
        }
            break;
        case 3:
        {
            auto v0 = Hott::SumDProtocollAdapter<0>::value(5);
            //            hbridge2::pwm(v0);
        }
            break;
        case 4:
        {
#ifdef USE_TC1_AS_HARDPPM
            auto v1 = Hott::SumDProtocollAdapter<0>::value(2);
            //            hardPpm1::ppm<hardPpm1::A>(v1);
#endif
        }
            break;
        case 5:
        {
#ifdef USE_TC1_AS_HARDPPM
            auto v1 = Hott::SumDProtocollAdapter<0>::value(5);
            //            hardPpm1::ppm<hardPpm1::B>(v1);
#endif
        }
            break;
        case 6:
            break;
        case 7:
            break;
        default:
            assert(false);
        }
        ++part;
    }
};

//struct MultiChannel {
//    inline static uint8_t part = 0;
//    inline static std::byte d{0};
//    inline static void update() {
//        if (Hott::SumDProtocollAdapter<0>::hasMultiChannel()) {
//            if (part < Hott::MultiChannel::size) {
//                if (Hott::SumDProtocollAdapter<0>::mChannel(part) == Hott::MultiChannel::State::Up) {
//                    d |= std::byte(1 << part);
//                }
//                ++part;
//            }
//            else {
//                mcp23008::startWrite(0x09, d);
//                d = 0_B;
//                part = 0;            
//            }
//        }
//        else {
//            mcp23008::startWrite(0x09, std::byte{Storage::keepAliveCounter});
//        }
//    }
//};

int main() {
    using namespace External::Units;
    using namespace etl;
    
    constexpr hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    //    static_assert(fCr == Config::Timer::frequency);
    
#ifdef USE_SPI_AS_TEST
    testPin1::dir<AVR::Output>();
    testPin1::off();
    testPin2::dir<AVR::Output>();
    testPin2::off();
    testPin3::dir<AVR::Output>();
    testPin3::off();
#endif
    //    isrRegistrar::init();
    
    eeprom::init();
    adcController::init();
    
    //    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    leds::init();
    leds::off();
    
    leds2::init();
    leds2::off();
    
    Util::delay(1_ms);    
    leds::set(0, Constants::cGreen);
    //    Util::delay(1_ms);    
    //    leds2::set(0, Constants::cOff);
    
    Util::delay(1_ms);    
    ledFSM::init();
    
    crWriterSensorBinary::init();
    crWriterSensorText::init();
    
#ifdef USE_TC1_AS_HARDPPM
    //    hardPpm1::init();
#else
    rpm1::init();
#endif
#ifdef USE_TC3_AS_HARDPPM
    //    hardPpm2::init();
    //    otherOCMPin::dir<AVR::Output>();
    //    otherOCMPin::on();
#else
    rpm2::init();
#endif
    //    rpm3::init();
    
//    hardPwm::init<Constants::pwmFrequency>();
//    hbridge1::init();
//    hbridge2::init();
    
//    hbridge1:: mode(hbridge1::Mode::BiDirectonal);
//    hbridge2:: mode(hbridge2::Mode::BiDirectonal);
    
    //    hx711::init();
    
    ds18b20::init();
    
    //    TwiMaster::init<Constants::fSCL>();
    
    //    gpsUsart::init<9600>();
    sensorUsart::init<BaudRate<19200>>();
    rcUsart::init<BaudRate<115200>>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        outl<terminal>("Test24"_pgm);
        Util::delay(100_ms);
        
        {
            outl<terminal>("1Wire:"_pgm);
            std::array<OneWire::ow_rom_t, Storage::dsIds.capacity> ids;
            oneWireMaster::findDevices(ids, ds18b20::family);
            for(const auto& id : ids) {
                if (id) {
                    Storage::dsIds.push_back(id);
                    outl<terminal>(id);
                    Util::delay(100_ms);
                }
            }
        }
        Util::delay(100_ms);
        {
            //            outl<terminal>("I2C:"_pgm);
            //            std::array<TWI::Address, Storage::i2cDevices.capacity> i2cAddresses;
            //            TwiMaster::findDevices(i2cAddresses);
            //            for(const auto& d : i2cAddresses) {
            //                if (d) {
            //                    Storage::i2cDevices.push_back(d);
            //                    std::outl<terminal>(d);
            //                    Util::delay(100_ms);
            //                }
            //            }
        }
        Util::delay(100_ms);
        
        //        mcp23008::startWrite(0x00, std::byte{0x00}); // output
        //        mcp23008::startWrite(0x09, std::byte{0x00}); // output
        
        //        if (!oled::init()) {
        //            std::outl<terminal>("oled error"_pgm);
        //        }
        
        //        std::outl<terminal>("hx711 max: "_pgm, hx711::max);
        
        //        outl<terminal>("ocmin: "_pgm, hardPpm1::parameter::ocMin);
        //        outl<terminal>("ocmax: "_pgm, hardPpm1::parameter::ocMax);
        //        outl<terminal>("freq: "_pgm, hardPpm1::parameter::timerFrequency);
        
        //        oled::clear();
        
        tempFSM::init();
        
        const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
        
        bool eepSave = false;
        while(true){
            testPin1::toggle(); // max 135 uS ein Intervall
            
            testPin2::on();
            //            TwiMasterAsync::periodic();
            menu::periodic();
#ifndef USE_TC1_AS_HARDPPM
            rpm1::periodic();
#endif
#ifndef USE_TC3_AS_HARDPPM
            rpm2::periodic();
#endif
            //            rpm3::periodic();
            //            hx711::periodic();
            adcController::periodic();
            tempFSM::periodic();
            eepSave |= eeprom::saveIfNeeded();
            testPin2::off();
            
            systemClock::periodic([&](){
                testPin3::on();
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                oneWireMasterAsync::rateProcess();
                Measurements::update();
                Actors::update();
                //                MultiChannel::update();
                testPin3::off();
                
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == periodicTimer) {
                        //                        oled::put('.');
                        ++Storage::keepAliveCounter;
#ifndef USE_TC1_AS_HARDPPM
                        rpm1::check();
#endif
#ifndef USE_TC3_AS_HARDPPM
                        rpm2::check();
#endif
                        //                        rpm3::check();
#ifdef OUTPUT
                        //                        GPS::RMC::timeRaw(Storage::time);
                        //                        GPS::VTG::speedRaw(Storage::speed);
                        uint16_t a = adcController::value(6);
                        auto v0 = Hott::SumDProtocollAdapter<0>::value(0);
                        //                        if (Hott::SumDProtocollAdapter<0>::hasMultiChannel()) {
                        //                            std::out<terminal>("M["_pgm, Hott::SumDProtocollAdapter<0>::mChannelForMultiChannel, "] "_pgm);
                        //                            for(uint8_t i = 0; i < Hott::MultiChannel::size; ++i) {
                        //                                std::out<terminal>(Char{' '}, (uint8_t)Hott::SumDProtocollAdapter<0>::mChannel(i));
                        //                            }
                        //                            std::outl<terminal>(Char{' '});
                        //                        }
                        //                        std::outl<terminal>("W: "_pgm, hx711::value());
                        outl<terminal>("v: "_pgm, v0.toInt());
                        outl<terminal>("t: "_pgm, Storage::time);
                        outl<terminal>("a: "_pgm, a);
                        outl<terminal>("r: "_pgm, Storage::rpms[2]);
                        if (eepSave) {
                            eepSave = false;
                            // todo: einmal LED grün leuchten lassen
                            outl<terminal>("eeprom save"_pgm);
                        }
# ifdef MEM
                        std::outl<terminal>("m: "_pgm, Util::Memory::getUnusedMemory());
# endif
# ifdef USE_PPM_ON_OPTO2
                        std::outl<terminal>("ppm: "_pgm, ppmDecoder::value(0));
# endif
#endif // Output
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
//ISR(INT2_vect) {
//    isrRegistrar::isr<AVR::ISR::Int<2>>();
//}
//// GPS
//ISR(USART2_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<2>::RX>();
//}
//ISR(USART2_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<2>::UDREmpty>();
//}
//// SumD
//ISR(USART1_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
//}
//ISR(USART1_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
//}
//// Sensor
//ISR(USART0_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
//}
//ISR(USART0_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
//}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    //    while(true) {}
}
#endif
