#define NDEBUG
#define OUTPUT

#define HOTT_NEW

#include "board.h"

#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>

using namespace AVR;
using namespace etl;
using namespace std;


// spi-pins als debug
using testPin1 = AVR::Pin<PortC, 4>; 
using testPin2 = AVR::Pin<PortC, 5>; 
using testPin3 = AVR::Pin<PortC, 6>; 
using testPin4 = AVR::Pin<PortC, 7>; 

using sensor = Hott::Experimental::Sensor<AVR::Component::Usart<0>, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, systemClock>;

struct Storage final {
    Storage() = delete;
    enum class AVKey : uint8_t {TSensor1 = 0, TSensor2,  // Sensornummer [0-N]
                                RpmSensor1, RpmSensor2, // Sensornummer [0-2]
                                Spannung1, Spannung2, 
                                ZellenZahl, 
                                StromOffset1, StromOffset2, StromOffset3, // Nullabgleich
                                Strom, // Sonsornummer [0-3]
                                PWM1, PWM1Mode, 
                                PWM2, PWM2Mode, 
                                Leds1Channel, Leds1Sequence, Leds2Channel, Leds2Sequence, 
                                _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
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

class PWMType final : public Hott::TextWithValue<Storage::AVKey, Storage::ApplData> {
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

class TSensorId final : public Hott::MenuItem {
public:
    TSensorId(uint8_t number) : mNumber{number} {
        assert(number < Storage::dsIds.capacity);
    }
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

namespace Constants {
    using namespace External;
    using namespace External::Units;
    using namespace External::Units::literals;
    
    static constexpr hertz pwmFrequency = 1000_Hz * 256; 
    static constexpr hertz fSCL = 100000_Hz;
    static constexpr Color cRed{Red{32}};
    static constexpr Color cBlue{Blue{32}};
    static constexpr Color cGreen{Green{32}};
    static constexpr Color cOff{};
}

//using isrRegistrar = IsrRegistrar<>;

class Dashboard final : public Hott::Menu {
public:
    Dashboard() : Menu(this, "Dashboard"_pgm) {}
private:
};

class SensorChoice final : public Hott::Menu {
public:
    SensorChoice() : Menu(this, "Sensorauswahl"_pgm, &mRpm1, &mRpm2, &mTemp1, &mTemp2) {}
private:
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mRpm1{"Rpm1"_pgm, appData, Storage::AVKey::RpmSensor1, 2};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mRpm2{"Rpm2"_pgm, appData, Storage::AVKey::RpmSensor2, 2};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mTemp1{"Temp1"_pgm, appData, Storage::AVKey::TSensor1, 3};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mTemp2{"Temp2"_pgm, appData, Storage::AVKey::TSensor2, 3};
};

class RCMenu final : public Hott::Menu {
public:
    RCMenu() : Menu(this, "RCSL 0.1"_pgm, &mDashboard, &mSensorChoice) {}
private:
    Dashboard mDashboard;
    SensorChoice mSensorChoice;
};

template<typename PA, typename TopMenu>
class HottMenu final {
    HottMenu() = delete;
public:
    inline static void init() {
        clear();
    }
    inline static void periodic() {
        if (auto k = PA::key(); k != Hott::key_t::nokey) {
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
private:
    inline static void clear() {
        for(auto& line : PA::text()) {
            line.clear();
        }
    }
    inline static TopMenu mTopMenu;
    inline static Hott::Menu* mMenu = &mTopMenu;
};

using menu = HottMenu<sensor, RCMenu>;

template<typename Sensor, typename Store, typename Alarm>
struct TempFSM final {
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
            process(Event::ReadComplete);
        });
    }
private:
    TempFSM() = delete;
    enum class State : uint8_t {Start, Wait, WaitRead};
    enum class Event : uint8_t {Measure, WaitOver, ReadComplete};
    
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
            if (e == Event::ReadComplete) {
                Storage::temps[mSensorNumber] = Sensor::temperature();
                mSensorNumber = (mSensorNumber + 1) % Store::dsIds.size();
                mState = State::Start;
            }
            break;
        }
    }
    inline static State mState = State::Start;
    inline static uint8_t mSensorNumber = 0;
    inline static uint_NaN<uint8_t> mMeasureTimer;
    inline static uint_NaN<uint8_t> mTempTimer;
};

using tempFSM = TempFSM<ds18b20, Storage, alarmTimer>;

//template<typename LEDs, typename Alarm, uint8_t Offset = 1>
//struct LedFSM final {
//    enum class State : uint8_t {};
//    inline static void init() {
//        mTimer = Alarm::create(1000_ms, Alarm::flags_type::Periodic);
//    }
//    inline static void tick(alarmTimer::index_type timer) {
//        if (timer == mTimer) {
//            LEDs::template set<false>(Color{});
//            LEDs::set(mActual, Constants::cBlue);
//            ++mActual;
//        }
//    }
//private:    
//    LedFSM() = delete;
//    inline static etl::uint_NaN<uint8_t> mTimer;
//    inline static uint_ranged_circular<uint8_t, Offset, LEDs::size - 1> mActual;
//};

//using ledFSM = LedFSM<leds, alarmTimer>;


struct Measurements final {
//    inline static constexpr uint8_t hottScale = adcController::mcu_adc_type::VRef / 0.02;
//    inline static constexpr auto rawMax = adcController::value_type::Upper;
//    inline static constexpr uint8_t battScale = adcController::mcu_adc_type::VRef / 0.1;
    inline static constexpr uint8_t parts = 5;
    
    static inline void check() {
        if (!rpm1::check()) {
            Storage::rpms[0] = RPM{0};
        }
    }
    
    static inline void update() {
        switch (++part) {
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
    }
private:
    Measurements() = delete;
    
    using adapter_t = Hott::Experimental::Adapter<Hott::GamMsg>;
    
    static inline void start() {
        zmin = std::numeric_limits<uint16_t>::max();
        zminNum = uint_NaN<uint8_t>();
    }
    
    // voltage
    static inline void update0() {
        auto sensorData = Hott::Experimental::Adapter<Hott::GamMsg>(sensor::data());
        
        
        uint16_t v1 = adcController::value(0).toInt() * 4;
        uint16_t v2 = adcController::value(1).toInt() * 8;
        uint16_t v3 = adcController::value(2).toInt() * 12;
        uint16_t v4 = adcController::value(3).toInt() * 16;
        uint16_t v5 = adcController::value(4).toInt() * 20;
        
        uint16_t z1 = cellVoltageConverter::convert(v1).value;
        uint16_t z2 = cellVoltageConverter::convert(v2 - v1).value;
        uint16_t z3 = cellVoltageConverter::convert(v3 - v2).value;
        uint16_t z4 = cellVoltageConverter::convert(v4 - v3).value;
        uint16_t z5 = cellVoltageConverter::convert(v5 - v4).value;
        
        
//        uint16_t z1 = etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(v1);
//        uint16_t z2 = etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(v2 - v1);
//        uint16_t z3 = etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(v3 - v2);
//        uint16_t z4 = etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(v4 - v3);
//        uint16_t z5 = etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(v5 - v4);
        
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
        sensorData.cellVoltageRaw(0, z1);
        sensorData.cellVoltageRaw(1, z2);
        sensorData.cellVoltageRaw(2, z3);
        sensorData.cellVoltageRaw(3, z4);
        sensorData.cellVoltageRaw(4, z5);
        
        uint16_t b1 = etl::maximum(v1, v2, v3, v4, v5);
        uint16_t zmax = etl::maximum(z1, z2, z3, z4, z5);
        
//        uint16_t batt = etl::Rational::RationalDivider<uint16_t, battScale, rawMax>::scale(b1);
        uint16_t batt = battVoltageConverter::convert(b1).value;
        
        //        sensorData.batteryVoltageRaw(adapter_t::Battery::One, batt);
        
        sensorData.battery1Raw(batt);
        sensorData.mainVoltageRaw(batt);
        
        Storage::battMinMax[0] = FixedPoint<int, 4>::fromRaw(etl::Rational::RationalDivider<uint16_t, 16, 50>::scale(zmin) + zmin);
        Storage::battMinMax[1] = FixedPoint<int, 4>::fromRaw(etl::Rational::RationalDivider<uint16_t, 16, 50>::scale(zmax) + zmax);
        
        if (zminNum) {
            sensorData.batteryMinimumRaw(zminNum.toInt(), zmin);
        }
    }
    // temperature
    static inline void update1() {
        auto sensorData = Hott::Experimental::Adapter<Hott::GamMsg>(sensor::data());
        
        if (const auto& s = appData[Storage::AVKey::TSensor1]) {
            sensorData.temp1(etl::select(s.toInt(), Storage::temps[0], Storage::temps[1], Storage::temps[2], Storage::temps[3]));
        }
        if (const auto& s = appData[Storage::AVKey::TSensor2]) {
            sensorData.temp2(etl::select(s.toInt(), Storage::temps[0], Storage::temps[1], Storage::temps[2], Storage::temps[3]));
        }
    }
    // current
    static inline void update2() {
        // todo: Skalierung berechnen    
        auto sensorData = Hott::Experimental::Adapter<Hott::GamMsg>(sensor::data());
        
        uint16_t a = adcController::value(6);
        uint16_t a1 = 0;
        if (a >= 128) {
            a1 = a - 128;
        }
        else {
            a1 = 128 - a;
        }
        
        uint16_t c1 = etl::Rational::RationalDivider<uint16_t, 98, 100>::scale(a1) + a1 + a1;
        
        sensorData.currentRaw(c1);
    }
    // rpm
    static inline void update3() {
        auto sensorData = Hott::Experimental::Adapter<Hott::GamMsg>(sensor::data());
        
        if (auto r = rpm1::rpm()) {
            Storage::rpms[0] = r;
        }
        if (const auto& s = appData[Storage::AVKey::RpmSensor1]) {
            sensorData.rpm1(etl::select(s.toInt(), Storage::rpms[0], Storage::rpms[1], Storage::rpms[2]));
        }
        if (const auto& s = appData[Storage::AVKey::RpmSensor2]) {
            sensorData.rpm2(etl::select(s.toInt(), Storage::rpms[0], Storage::rpms[1], Storage::rpms[2]));
        }
    }
    // gps
    static inline void update4() {
        auto sensorData = Hott::Experimental::Adapter<Hott::GamMsg>(sensor::data());
        
        auto s = etl::StringConverter<FixedPoint<int16_t, 4>>::parse(Storage::speed);
        sensorData.speedRaw(s.raw());
        //        sensorData::forceRaw(hx711::value());
    }
private:
    inline static uint_ranged_circular<uint8_t, 0, parts - 1> part;
    inline static uint16_t zmin = std::numeric_limits<uint16_t>::max();
    inline static uint_NaN<uint8_t> zminNum;
};

struct Actors final {
    Actors() = delete;
    inline static uint_ranged_circular<uint8_t, 0, 7> part;
    inline static void update() {
        switch(++part) {
        case 0: 
        {
        }
            break;
        case 1:
        {
        }
            break;
        case 2:
        {
            auto v0 = sumd::value(0);
            hbridge1::set(v0);
        }
            break;
        case 3:
        {
            auto v0 = sumd::value(3);
            hbridge2::set(v0);
        }
            break;
        case 4:
        {
        }
            break;
        case 5:
        {
        }
            break;
        case 6:
            break;
        case 7:
            break;
        default:
            assert(false);
        }
    }
};

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    testPin1::dir<AVR::Output>();
    testPin1::off();
    testPin2::dir<AVR::Output>();
    testPin2::off();
    testPin3::dir<AVR::Output>();
    testPin3::off();
    testPin4::dir<AVR::Output>();
    testPin4::off();
    
    sensor::init();
    
    systemClock::init();
    
    rcUsart::init<BaudRate<115200>>();
    
    outl<terminal>("test40"_pgm);    
    
    eeprom::init();
    adcController::init();
    
    leds::init();
    leds::off();
    
    leds2::init();
    leds2::off();
    
    Util::delay(1_ms);    
    leds::set(0, Constants::cGreen);
    
    Util::delay(1_ms);    
    //    ledFSM::init();
    
    rpm1::init();
    rpm2::init();
    rpm3::init();
    
    hardPwm::init<Constants::pwmFrequency>();
    
    hbridge1::init();
    hbridge2::init();
    
    hbridge1::mode(hbridge1::Mode::BiDirectonal);
    hbridge2::mode(hbridge2::Mode::BiDirectonal);
    
    ds18b20::init();
    
    gpsUsart::init<BaudRate<9600>>();
    rcUsart::init<BaudRate<115200>>();
    
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        {
            outl<terminal>("1Wire:"_pgm);
            std::array<OneWire::ow_rom_t, Storage::dsIds.capacity> ids;
            oneWireMaster::findDevices(ids, ds18b20::family);
            for(const auto& id : ids) {
                if (id) {
                    Storage::dsIds.push_back(id);
                    outl<terminal>(id);
                    
                    for(uint8_t i = 0; i < 100; ++i) {
                        terminalDevice::periodic();
                    }
                }
            }
        }
        
        tempFSM::init();
        
        const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
        
        bool eepSave = false;
        
        while(true) {
            rcUsart::periodic();
            gpsUsart::periodic();
            sensor::periodic();
            
            menu::periodic();
            rpm1::periodic();
            rpm2::periodic();
            rpm3::periodic();
            
            adcController::periodic();
            tempFSM::periodic();
            
            eepSave |= eeprom::saveIfNeeded();
            
            testPin2::off();
            
            systemClock::periodic([&](){
                
                sensor::ratePeriodic();
                
                oneWireMasterAsync::rateProcess();
                Measurements::update();
                Actors::update();
                
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    testPin4::on();
                    if (timer == periodicTimer) {
                        ++Storage::keepAliveCounter;
                        Measurements::check();
                        outl<terminal>("ar/br/coll "_pgm, sensor::asciiPackages(), Char{','}, sensor::binaryPackages(), Char{','}, sensor::collisions());    
#ifdef OUTPUT
                        //                        External::GPS::RMC::timeRaw(Storage::time);
                        External::GPS::VTG::speedRaw(Storage::speed);
                        //                        uint16_t a = adcController::value(6);
                        //                        auto v0 = sumd::value(0);
                        //                        outl<terminal>("v: "_pgm, v0.toInt());
                        //                        outl<terminal>("t: "_pgm, Storage::time);
                        //                        outl<terminal>("c: "_pgm, sumd::packageCount());
                        //                        outl<terminal>("a: "_pgm, a);
                        //                        outl<terminal>("r: "_pgm, Storage::rpms[0]);
                        //                        outl<terminal>("rm: "_pgm, rpm1::mMeasurements.toInt());
                        //                        outl<terminal>("t1: "_pgm, Storage::temps[0]);
                        //                        outl<terminal>("t2: "_pgm, Storage::temps[1]);
                        //                        outl<terminal>("t3: "_pgm, Storage::temps[2]);
                        if (eepSave) {
                            eepSave = false;
                            // todo: einmal LED grün leuchten lassen
                            outl<terminal>("eeprom save"_pgm);
                        }
#endif // Output
                    }
                    else {
                        tempFSM::tick(timer);
                        //                        ledFSM::tick(timer);
                    }
                    testPin4::off();
                });
                appData.expire();
            });
        }    
    }
}
