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


#include "std/array"
#include "container/stringbuffer.h"
#include "container/pgmstring.h"
#include "hal/eeprom.h"

struct ApplData : public EEPromBase<ApplData> {
   
    enum class AVKey : uint8_t {TSensor1 = 0, TSensor2, RpmSensor1, RpmSensor2, Spannung1, Spannung2, Strom, PWM, Leds1, Leds2, _Number};
    std::array<uint8_t, static_cast<uint8_t>(AVKey::_Number)> AValues;
    
    std::array<OneWire::ow_rom_t, 4> dsIds;
};

using eeprom = EEProm<ApplData>;
auto& appData = eeprom::data();

namespace Hott {
    static constexpr uint8_t MenuLength = 7;
    static constexpr uint8_t MenuStringLength = 20;
    using MenuString = StringBuffer<MenuStringLength>;
    
    using BufferString = StringBuffer<MenuStringLength + 1>;
    using Display = std::array<BufferString, MenuLength + 1>;
    
    class MenuItem {
    public:
        bool isSelected() const {return mSelected;}
        
        virtual void putTextInto(BufferString& buffer) const = 0;
        virtual MenuItem* processKey(Hott::key_t) {return this;}
        virtual bool hasChildren() const {return false;}
    protected:
        bool mSelected = false;
    private:
//        virtual ~MenuItem() {};
    };

    template<typename Key, uint8_t Max = 16>
    class TextWithValue : public MenuItem {
    public:
        TextWithValue(const PgmStringView& text, Key key) : mTitle(text), mKey(key) {}
        
        virtual void putTextInto(BufferString& buffer) const override {
            buffer[0] = ' ';
            buffer.insertAtFill(1, mTitle);
            
            Util::itoa_r<10>(appData.AValues[static_cast<uint8_t>(mKey)], &buffer[18]);
            
            if (mSelected) {
                buffer[18] |= 0x80;
                buffer[19] |= 0x80;
                buffer[20] |= 0x80;
            }
        }
        virtual MenuItem* processKey(Hott::key_t key) override {
            uint8_t& v = appData.AValues[static_cast<uint8_t>(mKey)];
            switch (key) {
            case Hott::key_t::up:
                if (mSelected) {
                    if (v > 0) {
                        --v;
                    }
                }
                break;
            case Hott::key_t::down:
                if (mSelected) {
                    if (v < Max) {
                        ++v;
                    }
                }
                break;
            case Hott::key_t::left:
                break;
            case Hott::key_t::right:
                break;
            case Hott::key_t::set:
                if (mSelected) {
                    appData.change();
                }
                mSelected = !mSelected;
                break;
            }
            return this;
        }
    private:
        const PgmStringView mTitle;
        const Key mKey;
    };
    
    class Menu : public MenuItem {
    public:
        template<typename... T>
        Menu(Menu* parent, const PgmStringView& title, T*... items) : mParent(parent), mTitle(title), mItems{items...} {}
        
        virtual bool hasChildren() const override {return true;}
        
        virtual void titleInto(BufferString& buffer) const {
            buffer.insertAtFill(0, mTitle);
        }
        
        virtual void putTextInto(BufferString& buffer) const override {
            buffer[0] = ' ';
            buffer.insertAtFill(1, mTitle);
        }
        void textTo(Display& display) const {
            static uint_ranged_circular<uint8_t, 0, Display::size - 1> line;
            if (line == 0) {
                titleInto(display[0]);
                ++line;
            }
            else {
                if (lineToDisplay(display[line], line)) {
                    if (mSelectedLine && (mSelectedLine == (line - 1))) {
                        display[mSelectedLine + 1][0] = '>';
                    }
                    ++line;
                }
            }
        }
        bool lineToDisplay(BufferString& buffer, uint8_t row) const {
            if (mItems[row - 1]) {
                mItems[row - 1]->putTextInto(buffer);
            }
            return true;
        }
        Menu* processKey(Hott::key_t key) override {
            if (mSelectedLine) {
                if (mItems[mSelectedLine]->isSelected()) {
                    mItems[mSelectedLine]->processKey(key);
                    return this;
                }
            }
            switch (key) {
            case Hott::key_t::down:
                if (mSelectedLine) {
                    if (mItems[mSelectedLine + 1]) {
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
                    if (mItems[mSelectedLine]->hasChildren()) {
                        return static_cast<Menu*>(mItems[mSelectedLine]);
                    }        
                    else {
                        mItems[mSelectedLine]->processKey(key);
                    }
                }
                break;
            }
            return this;
        }
    private:
        Menu* mParent = nullptr;
        const PgmStringView mTitle;
        const std::array<MenuItem*, MenuLength> mItems;
        uint_ranged_NaN<uint8_t, 0, 7> mSelectedLine;
    };
    
    class NumberedMenu : public Menu {
    public:
        template<typename...T>
        NumberedMenu(Menu* parent, uint8_t number, const PgmStringView& title, T*... items) : Menu(parent, title, items...), 
            mNumber(number) {}
        virtual void titleInto(BufferString& buffer) const override{
            buffer.insertAtFill(0, mTitle);
            buffer[19] =  '0' + mNumber;
        }
        
        virtual void putTextInto(BufferString& buffer) const override {
            buffer[0] = ' ';
            buffer.insertAtFill(1, mTitle);
            buffer[19] =  '0' + mNumber;
        }
    private:
        uint8_t mNumber;
    };

    class TSensorId : public MenuItem {
    public:
        TSensorId(uint8_t number) : mNumber{number} {
            assert(number < appData.dsIds.size);
        }
        virtual void putTextInto(BufferString& buffer) const override {
            if (appData.dsIds[mNumber]) {
                for(uint8_t i = 0; i < (appData.dsIds[mNumber].size - 2); ++i) {
                    uint8_t d = appData.dsIds[mNumber][i + 1];
                    Util::itoa_r<16>(d, &buffer[i * 3]);
                }
                for(uint8_t i = 0; i < (appData.dsIds[mNumber].size - 3); ++i) {
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
    
    class TSensorMenu : public NumberedMenu {
    public:
        TSensorMenu(Menu* parent, uint8_t n) : NumberedMenu(parent, n, "Sensor"_pgm, &mID), mID{n} {}
    private:
        TSensorId mID{0};
    };
    
    class TemperaturSensorenMenu : public Menu {
    public:
        TemperaturSensorenMenu(Menu* parent) : Menu(parent, "Temp. Sensoren"_pgm, &mTS1, &mTS2, &mTS3, &mTS4) {}
    private:
        TSensorMenu mTS1{this, 0};
        TSensorMenu mTS2{this, 1};
        TSensorMenu mTS3{this, 2};
        TSensorMenu mTS4{this, 3};
    };
    
    class TemperaturMenu : public Menu {
    public:
        TemperaturMenu(Menu* parent) : Menu(parent, "Temperatur"_pgm, &mSensor1, &mSensor2, &mSensoren) {}

    private:
        TextWithValue<ApplData::AVKey, appData.dsIds.size - 1> mSensor1{"Anzeige 1"_pgm, ApplData::AVKey::TSensor1};
        TextWithValue<ApplData::AVKey, appData.dsIds.size - 1> mSensor2{"Anzeige 2"_pgm, ApplData::AVKey::TSensor2};
        TemperaturSensorenMenu mSensoren{this};
    };
    
    class SpannungMenu : public Menu {
    public:
        SpannungMenu(Menu* parent) : Menu(parent, "Spannung"_pgm, &mSensor1) {}
    private:
        TextWithValue<ApplData::AVKey> mSensor1{"Sensor 1"_pgm, ApplData::AVKey::Spannung1};
    };
    
    class RCMenu : public Menu {
    public:
        RCMenu() : Menu(this, "WM SensorLed"_pgm, &mTemperatur, &mSpannung) {}
    private:
        TemperaturMenu mTemperatur{this};                        
        SpannungMenu   mSpannung{this};                        
    };
    
    template<typename PA, typename MenuType>
    class HottMenu final {
        HottMenu() = delete;
    public:
        static void periodic() {
            mMenu->textTo(PA::text());
        }
        static void processKey(Hott::key_t key) {
            assert(mMenu);
            if (auto m = mMenu->processKey(key); m != mMenu) {
                mMenu = m;
                for(auto& line : PA::text()) {
                    line.clear();
                }
            }
        }
    private:
        inline static MenuType mTop;
        inline static Menu* mMenu = &mTop;
    };
    
}
