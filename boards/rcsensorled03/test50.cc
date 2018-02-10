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

#include "container/tree.h"

#include <vector>

using testPin1 = AVR::Pin<PortB, 5>;

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
#ifdef USE_RPM2
, softPpm::OCAHandler, softPpm::OCBHandler
#endif
,i2cInterruptHandler
>;

using sensorData = Hott::SensorProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using menuData = Hott::SensorTextProtocollBuffer<0>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;


void menuAction(uint8_t index, const auto& callable);

template<auto... II>
struct Indexes {};

template<typename>
struct ValueOf;
template<auto... II>
struct ValueOf<Indexes<II...>> {
    typedef Meta::List<std::integral_constant<uint8_t, II>...> array;
    static uint8_t at(uint8_t index) {
        uint8_t value = 0;
        Meta::visitAt<array>(index, [&](auto w) {
            typedef decltype(w) wtype;
            value = wtype::type::value;
        });
        return value;
    }
};

template<typename OwnIndex, typename ParentIndex, typename Indexes>
struct StaticMenuNodeBase;

template<>
struct StaticMenuNodeBase<void, void, void> {
    inline static constexpr uint8_t own_index = 0;
    inline static constexpr uint8_t parent_index = 0;
    typedef Indexes<> childs_type;
    inline static constexpr uint8_t csize = 0;
};

template<size_t OI, size_t PI, auto... II>
struct StaticMenuNodeBase<Index<OI>, ParentIndex<PI>, Indexes<II...>> {
    inline static constexpr uint8_t own_index = OI;
    inline static constexpr uint8_t parent_index = PI;
    typedef Indexes<II...> childs_type;
    inline static constexpr uint8_t csize = sizeof...(II);
};

template<typename OI = void, typename PI = void, typename CC = void>
class Menu : public StaticMenuNodeBase<OI, PI, CC> {
    using base = StaticMenuNodeBase<OI, PI, CC>;
    using childs = typename base::childs_type;
public:
    constexpr Menu(const PgmStringView& title) : mTitle{title} {}
    constexpr Menu(const Menu<void, void, void>& menu) : mTitle{menu.mTitle} {}
    
    bool isSelected() const {return mSelected;}

    void titleInto(Hott::BufferString& buffer) const {
        buffer.insertAtFill(0, mTitle);
    }
    
    inline void putTextInto(Hott::BufferString& buffer) const {
        buffer[0] = ' ';
        buffer.insertAtFill(1, mTitle);
    }
    
    inline void textTo(Hott::Display& display) const {
        static uint_ranged_circular<uint8_t, 0, Hott::Display::size - 1> line;
        if (line == 0) {
            titleInto(display[0]);
            ++line;
        }
        else {
            if (line <= base::csize) { 
                uint8_t cindex = ValueOf<childs>::at(line - 1);
                menuAction(cindex, [&](auto& child) {
                    child.putTextInto(display[line]);
                    return 0;
                });
                if (mSelectedItem && (mSelectedItem == (line - 1))) {
                    display[line][0] = '>';
                }
                ++line;
            }
            else {
                line = 0;
            }
        }
    }
    
    inline uint_NaN<uint8_t> processKey(Hott::key_t key) {
        uint_NaN<uint8_t> selectedChildIndex;
        if (mSelectedItem) {
            selectedChildIndex = ValueOf<childs>::at(mSelectedItem);
        }
        
        if (selectedChildIndex) {
            menuAction(selectedChildIndex, [&](auto& item){
                if (item.isSelected()) {
                    item.processKey(key);
                }
                return 0;
            });            
        }
        switch (key) {
        case Hott::key_t::down:
            if (mSelectedItem) {
                if ((mSelectedItem + 1) < base::csize) {
                    ++mSelectedItem;
                }
            }
            else {
                mSelectedItem = 0;
            }
            break;
        case Hott::key_t::up:
            if (mSelectedItem) {
                --mSelectedItem;
            }
            else {
                mSelectedItem = 0;
            }
            break;
        case Hott::key_t::left:
            return uint_NaN<uint8_t>{base::parent_index};
            break;
        case Hott::key_t::right:
            break;
        case Hott::key_t::set:
            if (mSelectedItem) {
                bool hasChildren = false;
                menuAction(selectedChildIndex, [&](auto& item){
                    if constexpr(item.csize > 0) {
                        hasChildren = true;
                    }
                    return 0;
                });
                if (hasChildren) {
                    return selectedChildIndex;
                }
                else {
                    menuAction(selectedChildIndex, [&](auto& item){
                        item.processKey(key);
                        return 0;
                    });
                }
            }
            break;
        case Hott::key_t::nokey:
            break;
        }
        return uint_NaN<uint8_t>{base::own_index};
    }
private:
    PgmStringView mTitle;
    uint_ranged_NaN<uint8_t, 0, base::csize> mSelectedItem;
    bool mSelected{false};
};

template<typename OI = void, typename PI = void, typename CC = void>
class MenuItem : public StaticMenuNodeBase<OI, PI, CC> {
    using base = StaticMenuNodeBase<OI, PI, CC>;
public:
//    constexpr MenuItem() {}
    inline constexpr MenuItem(const PgmStringView& title) : mTitle{title} {}
    constexpr MenuItem(const MenuItem<void, void, void>& other) : mTitle{other.mTitle} {}
    bool isSelected() const {return false;}

    inline uint_NaN<uint8_t> processKey(Hott::key_t) {
        return uint_NaN<uint8_t>{base::own_index};
    }
    inline void textTo(Hott::Display& ) const {}
    inline void putTextInto(Hott::BufferString& buffer) const {
        buffer[0] = ' ';
        buffer.insertAtFill(1, "abc"_pgm);
    }
private:
    PgmStringView mTitle;
};
template<typename OI = void, typename PI = void, typename CC = void>
class MenuItemX : public StaticMenuNodeBase<OI, PI, CC> {
    using base = StaticMenuNodeBase<OI, PI, CC>;
public:
//    constexpr MenuItem() {}
    inline constexpr MenuItemX(const PgmStringView& title) : mTitle{title} {}
    constexpr MenuItemX(const MenuItemX<void, void, void>& other) : mTitle{other.mTitle} {}
    bool isSelected() const {return false;}

    inline uint_NaN<uint8_t> processKey(Hott::key_t) {
        return uint_NaN<uint8_t>{base::own_index};
    }
    inline void textTo(Hott::Display& ) const {}
    inline void putTextInto(Hott::BufferString& buffer) const {
        buffer[0] = ' ';
        buffer.insertAtFill(1, "abc"_pgm);
    }
private:
    PgmStringView mTitle;
};

template<typename T, uint8_t L1, uint8_t LField>
class DualValue {
public:
    inline static constexpr uint8_t csize = 0;
    constexpr DualValue(const PgmStringView& text, const T& v1, const T& v2) : mText(text), mData1{v1}, mData2{v2} {}
    
    void putTextInto(Hott::BufferString& buffer) const {
        buffer.clear();
        buffer.insertAt(0, mText);
        putValue(mData1, UI::make_span<L1, LField>(buffer));
        putValue(mData2, UI::make_span<L1 + LField, LField>(buffer));
    }
    bool isSelected() const {return false;}
    inline void textTo(Hott::Display& ) const {}
    inline uint8_t processKey(Hott::key_t) {
        return {}; 
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

auto flat_tree = [&]{
    constexpr auto tree = Node(Menu{"WM SensMod HW 3 SW 50"_pgm}, 
                               Node(Menu{"Uebersicht"_pgm}, 
                                    DualValue<FixedPoint<int, 4>, 5, 8>{"T1/2"_pgm, Storage::temps[0], Storage::temps[1]},
                                    MenuItem{"A1"_pgm},
                                    MenuItemX{"A2"_pgm},
                                    MenuItem{"A3"_pgm},
                                    MenuItem{"A4"_pgm}
                                    ),
                               Node(Menu{"Temperatur"_pgm}, 
                                    MenuItem{"A1"_pgm}
                                    ),
                               Node(Menu{"Spannung"_pgm}, 
                                    MenuItemX{"A1"_pgm}
                                    ),
                               Node(Menu{"Drehzahl"_pgm}, 
                                    MenuItem{"A1"_pgm}
                                    ),
                               Node(Menu{"Strom"_pgm}, 
                                    MenuItem{"A1"_pgm}
                                    ),
                               Node(Menu{"Aktoren"_pgm}, 
                                    MenuItem{"A1"_pgm}
                                    )
                               );
    
    constexpr auto ftree = make_tuple_of_tree(tree);
    return ftree;
};

template<auto... II, Util::Callable L>
constexpr auto inode_to_indexnode(std::index_sequence<II...>, const L& callable) {
    constexpr auto inode = callable();
    static_assert(isInode(inode), "use a callable returning an INode<>");
    typedef typename decltype(inode)::type dataType;
    
    if constexpr(std::is_same<dataType, Menu<void, void, void>>::value) {
        return Menu<Index<inode.mNumber>, ParentIndex<inode.mParent>, Indexes<inode.mChildren[II]...>>{inode.mData};
    }
    else if constexpr(std::is_same<dataType, MenuItem<void, void, void>>::value) {
        return MenuItem<Index<inode.mNumber>, ParentIndex<inode.mParent>, Indexes<inode.mChildren[II]...>>{inode.mData};
    }
    else {
        return dataType{inode.mData};
    }
}

template<Util::Callable L>
constexpr auto transform(const L& callable) {
    constexpr auto tuple = callable();
    static_assert(Util::isTuple(tuple), "use constexpr callabe returning a tuple");
    
    if constexpr(Util::size(tuple) == 0) {
        return std::tuple<>();
    }
    else {
        constexpr auto first = std::get<0>(tuple);    
        constexpr auto rest = [&]{return Util::tuple_tail(tuple);};
        
        constexpr auto indexnode = inode_to_indexnode(std::make_index_sequence<first.mChildren.size>{}, [&]{return first;});
        return std::tuple_cat(std::tuple(indexnode), transform(rest));        
    }
}

auto t2 = transform(flat_tree); 

//decltype(t2)::_;

void menuAction(uint8_t index, const auto& callable) {
    Meta::visitAt(t2, index, callable);
}

template<typename PA>
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
        menuAction(mMenu, [&](auto& i) {
            i.textTo(PA::text());
            return 0;
        });
    }
    inline static void processKey(Hott::key_t key) {
        menuAction(mMenu, [&](auto& i) {
            if (auto m = i.processKey(key); m && (m != mMenu)) {
                mMenu = m;
                for(auto& line : PA::text()) {
                    line.clear();
                }
            }
            return 0;
        });
    }
    inline static void isrKey(std::byte b) {
        mKey = Hott::key_t{b};
    }
private:
    inline static volatile Hott::key_t mKey = Hott::key_t::nokey;
    inline static uint8_t mMenu = std::tuple_size<decltype(t2)>::value - 1;
};

using menu = HottMenu<menuData>;


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

inline void updateMeasurements() {
    constexpr uint8_t hottScale = adcController::mcu_adc_type::VRef / 0.02;
    constexpr auto rawMax = adcController::value_type::Upper;
    constexpr uint8_t battScale = adcController::mcu_adc_type::VRef / 0.1;
    
    //    constexpr uint8_t
    
    uint16_t zmin = std::numeric_limits<uint16_t>::max();
    uint_NaN<uint8_t> zminNum;
    {
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
    zmin = std::numeric_limits<uint16_t>::max();
    zminNum = uint_NaN<uint8_t>();
    {
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

    
    //    constexpr uint8_t tempScale = adcController::mcu_adc_type::VRef / 0.314;
    //    uint8_t t1 = (adcController::value(4) * tempScale * 25) / rawMax + 20;
    
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
    
    const auto upm1 = rpm1::rpm();
    sensorData::rpm1(upm1);
    Storage::rpms[0] = rpm1::rpm();
    
#ifdef USE_RPM2
    const auto upm2 = rpm2::rpm();
    Storage::rpms[1] = rpm2::rpm();
#endif
    
}

inline void updateActors() {
    auto v1 = Hott::SumDProtocollAdapter<0>::value(1);
    auto v3 = Hott::SumDProtocollAdapter<0>::value(3);
#ifdef USE_RPM2
    softPpm::ppm(v1, 0);
#else
    hardPpm::ppm<hardPpm::A>(v1);
    hardPpm::ppm<hardPpm::B>(v3);
#endif                        
    auto v0 = Hott::SumDProtocollAdapter<0>::value(0);
    hbridge1::pwm(v0);
    hbridge2::pwm(v0);
}

inline void updateMultiChannel() {
    if (Hott::SumDProtocollAdapter<0>::hasMultiChannel()) {
        std::byte d{0};
        for(uint8_t i = 0; i < Hott::MultiChannel::size; ++i) {
            if (Hott::SumDProtocollAdapter<0>::mChannel(i) == Hott::MultiChannel::State::Up) {
                d |= std::byte(1 << i);
            }
        }
        mcp23008::startWrite(0x09, d);
    }
}


int main() {
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    eeprom::init();
    adcController::init();
    
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
    
    TwiMaster::init<Constants::fSCL>();
    mcp23008::startWrite(0x00, std::byte{0x00}); // output
    
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("Test50"_pgm);
        
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
        
        tempFSM::init();
        
        while(true){
            testPin1::toggle(); // 25 us -> 40 KHz
            TwiMasterAsync::periodic();
            menu::periodic();
            rpm1::periodic();
#ifdef USE_RPM2
            rpm2::periodic();
#endif
            adcController::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                oneWireMasterAsync::rateProcess();
                updateMeasurements();
                updateActors();
                updateMultiChannel();
                
                alarmTimer::periodic([](uint7_t timer){
                    if (timer == *periodicTimer) {
                        if (Hott::SumDProtocollAdapter<0>::hasMultiChannel()) {
                            std::out<terminal>("Multi["_pgm, Hott::SumDProtocollAdapter<0>::mChannelForMultiChannel, "]: "_pgm);
                            for(uint8_t i = 0; i < Hott::MultiChannel::size; ++i) {
                                std::out<terminal>(Char{' '}, (uint8_t)Hott::SumDProtocollAdapter<0>::mChannel(i));
                            }
                            std::outl<terminal>();
                        }                        
                        rpm1::check();
                        uint16_t a = adcController::value(6);
                        std::outl<terminal>(a);
                        
#ifdef USE_RPM2
                        rpm2::check();
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

