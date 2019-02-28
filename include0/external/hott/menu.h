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

#include <cstdint>
#include <array>

#include "etl/stringbuffer.h"
#include "etl/format.h"
#include "etl/span.h"

#include "mcu/pgm/pgmstring.h"

#include "sensorprotocoll.h"

#include "external/ui/menu.h"

namespace Hott {
    static constexpr uint8_t MenuLength = 7;
    
    using BufferString = Hott::TextMsg::line_type;
    using Display = Hott::TextMsg::buffer_type;
    
    using MenuItem = UI::MenuItem<BufferString, key_t>;
    
    // fixme: keine mag. Konstanten 18 3 Aufteilung
    
    template<typename Key, typename Provider, uint8_t ValueWidth = 3>
    class TextWithValue : public MenuItem {
    public:
        static inline constexpr uint8_t valueBeginColumn = BufferString::size() - ValueWidth;
        static inline constexpr uint8_t valueWidth = ValueWidth;
        
        using value_span_type = etl::span<valueWidth, etl::Char>;
        
        TextWithValue(const PgmStringView& text, Provider& provider, Key key, uint8_t maxValue) : mTitle(text), mProvider(provider), mKey(key), mMax(maxValue) {}
        
        virtual void valueToText(uint8_t value, value_span_type buffer) const {
            etl::itoa_r<10>(value, buffer);
        }
        
        virtual void putTextInto(BufferString& buffer) const override {
            buffer[0] = Char{' '};
            buffer.insertAtFill(1, mTitle);
            
            auto& value = mProvider[mKey];
            if (value) {
                valueToText(*value, etl::make_span<valueBeginColumn, valueWidth>(buffer));
            }
            else {
                etl::fill(etl::make_span<valueBeginColumn, valueWidth>(buffer), Char{'-'});
            }
            
            if (mSelected) {
                etl::apply(etl::make_span<valueBeginColumn, valueWidth>(buffer), [](auto& c) {c |= Char{0x80};});
            }
        }
        virtual MenuItem* processKey(Hott::key_t key) override {
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
                        *v = mMax;
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
            return this;
        }
    private:
        const PgmStringView mTitle;
        Provider& mProvider;
        const Key mKey = Hott::key_t::nokey;
        const uint8_t mMax;
    };
    
    class Menu : public MenuItem {
    public:
        template<typename... T>
        Menu(Menu* parent, const PgmStringView& title, T*... items) : mParent(parent), mTitle(title), mItems{items...} {
            static_assert(sizeof...(T) <= MenuLength, "too much entries");
        }
        virtual bool hasChildren() const override {return true;}
        
        virtual void titleInto(BufferString& buffer) const {
            buffer.insertAtFill(0, mTitle);
        }
        
        virtual void putTextInto(BufferString& buffer) const override {
            buffer[0] = Char{' '};
            buffer.insertAtFill(1, mTitle);
        }
        void textTo(Display& display) const {
            static uint_ranged_circular<uint8_t, 0, display.size() - 1> line;
            if (line == 0) {
                titleInto(display[0]);
                ++line;
            }
            else {
                if (lineToDisplay(display[line], line)) {
                    if (mSelectedLine && (mSelectedLine.toInt() == (line - 1))) {
                        display[mSelectedLine.toInt() + 1][0] = Char{'>'};
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
                if (mItems[mSelectedLine.toInt()]->isSelected()) {
                    mItems[mSelectedLine.toInt()]->processKey(key);
                    return this;
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
                    if (mItems[mSelectedLine.toInt()]->hasChildren()) {
                        return static_cast<Menu*>(mItems[mSelectedLine.toInt()]);
                    }        
                    else {
                        mItems[mSelectedLine.toInt()]->processKey(key);
                    }
                }
                break;
            case Hott::key_t::nokey:
                break;
            }
            return this;
        }
    protected:
        Menu* mParent = nullptr;
        const PgmStringView mTitle;
        const std::array<MenuItem*, MenuLength> mItems{};
        uint_ranged_NaN<uint8_t, 0, MenuLength> mSelectedLine{};
    };

    /*
    class Menu2 : public MenuItem {
    public:
        Menu2(const PgmStringView& title, std::initializer_list<MenuItem*> items) : mTitle(title) {
            auto it = std::begin(items);
            for(uint8_t i = 0; i < items.size(); ++i) {
                if (it == std::end(items)) {
                    break;
                }
                mItems[i] = *it++;
                mItems[i]->parent(this);
            }
        }
        
        virtual void parent(MenuItem* p) override {
            mParent = static_cast<Menu2*>(p);
        }
        
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
                    if (mSelectedLine && (mSelectedLine.toInt() == (line - 1))) {
                        display[mSelectedLine.toInt() + 1][0] = '>';
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
        Menu2* processKey(Hott::key_t key) override {
            if (mSelectedLine) {
                if (mItems[mSelectedLine.toInt()]->isSelected()) {
                    mItems[mSelectedLine.toInt()]->processKey(key);
                    return this;
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
                    if (mItems[mSelectedLine.toInt()]->hasChildren()) {
                        return static_cast<Menu2*>(mItems[mSelectedLine.toInt()]);
                    }        
                    else {
                        mItems[mSelectedLine.toInt()]->processKey(key);
                    }
                }
                break;
            case Hott::key_t::nokey:
                break;
            }
            return this;
        }
    protected:
        Menu2* mParent = nullptr;
        const PgmStringView mTitle;
        std::array<MenuItem*, MenuLength> mItems{};
        uint_ranged_NaN<uint8_t, 0, MenuLength> mSelectedLine{};
    };
    */
    class NumberedMenu : public Menu {
    public:
        template<typename...T>
        NumberedMenu(Menu* parent, uint8_t number, const PgmStringView& title, T*... items) : Menu(parent, title, items...), 
            mNumber(number) {}
        virtual void titleInto(BufferString& buffer) const override{
            buffer.insertAtFill(0, mTitle);
            buffer[19] =  Char('0' + mNumber);
        }
        
        virtual void putTextInto(BufferString& buffer) const override {
            buffer[0] = Char{' '};
            buffer.insertAtFill(1, mTitle);
            buffer[19] = Char('0' + mNumber);
        }
    private:
        uint8_t mNumber = 0;
    };
/*
    class NumberedMenu2 : public Menu2 {
    public:
        NumberedMenu2(uint8_t number, const PgmStringView& title, std::initializer_list<MenuItem*> items) : 
            Menu2(title, items), 
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
        uint8_t mNumber = 0;
    };
  */  
    
}
