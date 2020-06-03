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
    using BufferString = Hott::TextMsg::line_type;
//    using Display = Hott::TextMsg::buffer_type;
    
    using MenuItem = UI::MenuItem<BufferString, key_t>;

    // fixme: keine mag. Konstanten 18 3 Aufteilung
    
    template<typename Key, typename Provider, uint8_t ValueWidth = 3, typename F = void>
    requires requires (Provider p, Key k) {
        p[k];
        p.change();
    }
    class TextWithValue : public MenuItem {
    public:
        static inline constexpr uint8_t valueBeginColumn = BufferString::size() - ValueWidth;
        static inline constexpr uint8_t valueWidth = ValueWidth;
        
        using value_span_type = etl::span<valueWidth, etl::Char>;
        
        constexpr TextWithValue(const AVR::Pgm::StringView& text, Provider& provider, const Key key, const uint8_t maxValue) : mTitle{text}, mProvider{provider}, mKey{key}, mMax{maxValue} {}
        
        virtual void valueToText(const uint8_t value, value_span_type buffer) const {
            if constexpr(std::is_same_v<F, void>) {
                etl::itoa_r<10>(value, buffer);
            }
            else {
                F::format(value, buffer);
            }
        }
        
        virtual void putTextInto(BufferString& buffer) const override {
            buffer[0] = Char{' '};
            buffer.insertAtFill(1, mTitle);
            
            const auto& value = mProvider[mKey];
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
        virtual MenuItem* processKey(const Hott::key_t key) override {
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
        const AVR::Pgm::StringView mTitle;
        Provider& mProvider;
        const Key mKey = Hott::key_t::nokey;
        const uint8_t mMax;
    };

    
    template<uint8_t MenuLength = 8>
    class IMenu : public MenuItem {
    public:
        using Display = std::array<BufferString, MenuLength>;
        virtual inline void textTo(Display& display) = 0;
        virtual IMenu<MenuLength>* processKey(const Hott::key_t key) override = 0;
    };
    
template<uint8_t MenuLength = 8, bool useTitle = true, uint8_t SL = MenuLength>
    requires(MenuLength <= 8)
    class Menu : public IMenu<MenuLength> {
    public:
        template<typename... T>
        constexpr Menu(IMenu<MenuLength>* const parent, const AVR::Pgm::StringView& title, T*... items) : mParent{parent}, mTitle{title}, mItems{items...} {
                            constexpr uint8_t linesToBeShown = []{
                                if constexpr(useTitle) {
                                    return sizeof...(T) + 1;
                                }
                                else {
                                    return sizeof...(T);
                                }
                            }();
            static_assert(linesToBeShown <= MenuLength, "too much entries for display");
            static_assert(SL == linesToBeShown, "shown lines must match (if title incl. title)");
        }
        virtual bool hasChildren() const override {return true;}
        
        virtual void titleInto(BufferString& buffer) const {
               buffer.insertAtFill(0, mTitle);
        }
        
        virtual void putTextInto(BufferString& buffer) const override {
            buffer[0] = Char{' '};
            auto p = buffer.insertAtFill(1, mTitle);
            buffer[p] = etl::Char{'>'};
        }
        using Display = std::array<BufferString, MenuLength>;

inline static constexpr uint8_t selectableLinesMax = []{
    if constexpr(useTitle) {
        return SL - 2;
    }
    else {
        return SL - 1;
    }
}();

//    std::integral_constant<uint8_t, selectableLinesMax>::_;

        using line_type = uint_ranged_circular<uint8_t, 0, SL - 1>;

        inline void textTo(Display& display)  override {
            static line_type line{0};
            if (useTitle) {
                if (line == 0) {
                    titleInto(display[0]);
                }
                else {
                    lineToDisplay(display[line], line - 1);
                    if (mSelectedLine == (line - 1)) {
                        display[line][0] = Char{'>'};
                    }
                }
            }
            else {
                lineToDisplay(display[line], line);
                if (mSelectedLine == line) {
                    display[line][0] = Char{'>'};
                }
            }
            ++line;
        }
        void lineToDisplay(BufferString& buffer, const uint8_t row) const {
            if ((row < mItems.size()) && mItems[row]) {
                mItems[row]->putTextInto(buffer);
            }
        }
        IMenu<MenuLength>* processKey(const Hott::key_t key) override {
            if (mItems[mSelectedLine.toInt()]->isSelected()) {
                mItems[mSelectedLine.toInt()]->processKey(key);
                return this;
            }
            switch (key) {
            case Hott::key_t::down:
                ++mSelectedLine;
                break;
            case Hott::key_t::up:
                --mSelectedLine;
                break;
            case Hott::key_t::left:
                return mParent;
                break;
            case Hott::key_t::right:
                break;
            case Hott::key_t::set:
                if (mItems[mSelectedLine.toInt()]->hasChildren()) {
                    return static_cast<IMenu<MenuLength>*>(mItems[mSelectedLine.toInt()]);
                }        
                else {
                    mItems[mSelectedLine.toInt()]->processKey(key);
                }
                break;
            case Hott::key_t::nokey:
                break;
            }
            return this;
        }
    protected:
        IMenu<MenuLength>* mParent{nullptr};
        const AVR::Pgm::StringView mTitle;
        etl::uint_ranged<uint8_t, 0, selectableLinesMax> mSelectedLine{0};
        const std::array<MenuItem*, selectableLinesMax + 1> mItems{};
    };

        
    template<uint8_t MenuLength>
    class NumberedMenu : public Menu<MenuLength> {
        using base = Menu<MenuLength>;
    public:
        template<typename...T>
        constexpr NumberedMenu(Menu<MenuLength>* parent, uint8_t number, const AVR::Pgm::StringView& title, T*... items) : Menu<MenuLength>(parent, title, items...), 
            mNumber(number) {}
        virtual void titleInto(BufferString& buffer) const override{
            buffer.insertAtFill(0, base::mTitle);
            buffer[19] =  Char('0' + mNumber);
        }
        
        virtual void putTextInto(BufferString& buffer) const override {
            buffer[0] = Char{' '};
            buffer.insertAtFill(1, base::mTitle);
            buffer[19] = Char('0' + mNumber);
        }
    private:
        uint8_t mNumber = 0;
    };

    template<uint8_t L>
    class TextItem final : public Hott::MenuItem {
    public:
        constexpr TextItem(const AVR::Pgm::StringView& title, const etl::StringBuffer<L>& text) : mTitle{title}, mText{text} {}
        
        void putTextInto(Hott::BufferString& buffer) const {
            buffer.insertAt(1, mTitle);
            buffer.insertAt(10, mText);
        }
        auto& text() {
            return mText;
        }
    private:
        const AVR::Pgm::StringView mTitle;
        const etl::StringBuffer<L>& mText;   
    };
    
    
    template<typename PA, typename TopMenu>
    class BasePage final {
        BasePage() = delete;
    public:
        inline static void init() {
            clear();
        }
        inline static void periodic() {
            PA::processKey([&](Hott::key_t k){
                if (const auto n = mMenu->processKey(k); n != mMenu) {
                    clear();
                    if (n) {
                        mMenu = n;
                    }
                    else {
                        PA::esc();
                    }
                }
            });
            PA::notSending([&]{
                mMenu->textTo(PA::text());
            });
        }
    private:
        inline static void clear() {
            for(auto& line : PA::text()) {
                line.clear();
            }
        }
        inline static TopMenu mTopMenu;
        inline static Hott::IMenu<PA::menuLines>* mMenu = &mTopMenu;
    };    
}
