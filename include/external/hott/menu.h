/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#pragma once

#include "sensortextprotocollbuffer.h"
#include "std/types.h"

namespace Hott {

    enum class AVKey : uint8_t {TSensor1, TSensor2, RpmSensor1, RpmSensor2, Spannung1, Spannung2, Strom, PWM, Leds1, Leds2, _Number};
    std::array<uint8_t, static_cast<uint8_t>(AVKey::_Number)> AValues;
    
    template<typename Buffer>
    class MenuSystem {
        typedef uint_ranged<uint8_t, 0, Buffer::rows> row_type;
        typedef uint_ranged<uint8_t, 0, Buffer::columns> column_type;
        
        inline static constexpr auto rows = Buffer::rows;
        inline static constexpr auto columns = Buffer::columns;
        
        struct MenuItem {
            enum class Type : uint8_t {Title, // Titelzeile
                                       Info, // Infotext (fix)
                                       Presence, // Text gefolgt von --|ja 
                                       Menu, // nä. Menu
                                       YesNo, // Text und Auswahlbox: j/n
                                       Value100, // Text und Auswahlbox: 0...100
                                       Value8, // Text und Auswahlbox: 0...7
                                       Value4, // Text und Auswahlbox: 0...3
                                       Value2, // Text und Auswahlbox: 0, 1
                                       Empty, // keine Anzeige
                                      };
            Type mType = Type::Empty;
            PgmStringView mText = ""_pgm;
            const uintptr_t mMenu = 0; // Zeiger auf Menu oder AVKey
        };
        
        struct Menu {
            std::array<MenuItem, 8> mItems;
        };        
        
        inline static Menu tsensMenu = {MenuItem{MenuItem::Type::Title, "Liste Sensoren"_pgm},
                                        MenuItem{MenuItem::Type::Presence, "Sensor 1"_pgm},
                                        MenuItem{MenuItem::Type::Presence, "Sensor 2"_pgm},
                                        MenuItem{MenuItem::Type::Presence, "Sensor 3"_pgm},
                                        MenuItem{MenuItem::Type::Presence, "Sensor 4"_pgm},
                                      };

        inline static Menu tempMenu = {MenuItem{MenuItem::Type::Title, "Temp-Sensoren"_pgm},
                                       MenuItem{MenuItem::Type::Menu, "Liste"_pgm, (uintptr_t)&tsensMenu},
                                       MenuItem{MenuItem::Type::Value4, "Anzeige 1"_pgm, (uintptr_t)AVKey::TSensor1},
                                       MenuItem{MenuItem::Type::Value4, "Anzeige 2"_pgm, (uintptr_t)AVKey::TSensor2}
                                      };
        
        inline static Menu startMenu = {MenuItem{MenuItem::Type::Title, "WM Multi RC V0.1"_pgm}, 
                                        MenuItem{MenuItem::Type::Menu, "Temperatur"_pgm, (uintptr_t)&tempMenu},
                                        MenuItem{MenuItem::Type::Menu, "Drehzahl"_pgm},
                                        MenuItem{MenuItem::Type::Menu, "Spannung"_pgm},
                                        MenuItem{MenuItem::Type::Menu, "Strom"_pgm},
                                        MenuItem{MenuItem::Type::Menu, "Steller"_pgm},
                                        MenuItem{MenuItem::Type::Menu, "Leds 1"_pgm},
                                        MenuItem{MenuItem::Type::Menu, "Leds 2"_pgm}
                                       };
        struct Position {
            uint8_t mRow = 0;
            uint8_t mColumn = 0;
            operator bool() const {
                return (mRow == 0) && (mColumn == 0);
            }
            Position& operator++() {
                mColumn = (mColumn + 1) % columns;
                if (mColumn == 0) {
                    mRow = (mRow + 1) % rows;
                }
                return *this;
            }
        };
        
        enum class State: uint8_t {Filled, Filling};
        
    public:
        static void key(Hott::key_t k) {
            switch (k) {
            case Hott::key_t::down:
                mCursorPosition.mRow = (mCursorPosition.mRow + 1) % rows; 
                break;
            case Hott::key_t::up:
                mCursorPosition.mRow = (mCursorPosition.mRow - 1) % rows; 
                break;
            case Hott::key_t::left:
                mCursorPosition.mColumn = (mCursorPosition.mColumn - 1) % columns; 
                break;
            case Hott::key_t::right:
                mCursorPosition.mColumn = (mCursorPosition.mColumn + 1) % columns; 
                break;
            case Hott::key_t::set:
                break;
            }
        }
        
        static void init() {
            mMenu = &startMenu;
            startFilling();
        }
        
        static void periodic() {
            switch (mState) {
            case State::Filled:
                break;
            case State::Filling:
                loadMenuData();
                break;
            }
        }
        
    private:
        static void startFilling() {
            mNextLoadPosition = {0, 0};
            mState = State::Filling;
        }

        struct Transfer {
            Transfer() = default;
            Transfer(const MenuItem* mi) : mItem(mi) {};
            char operator()() {
                char next = ' ';
                if (!mItem) return next;
                if (!position) return next;
                
                switch(mItem->mType) {
                case MenuItem::Type::Empty:
                    break;
                case MenuItem::Type::Info:
                    next = mItem->mText[position];
                    if (next == '\0') {
                        
                    }
                    break;
                }
                ++position;
                return next;
            }
            const MenuItem* mItem = nullptr;
            uint_NaN<uint8_t> position{0};
        };
        
        inline static Transfer tf;
        
        static void loadMenuData() {
            if (mMenu == nullptr) return;
            if (!mNextLoadPosition) {
                tf = Transfer(&mMenu->mItems[mNextLoadPosition.mRow]);
            }    
            Buffer::text()[mNextLoadPosition.mRow][mNextLoadPosition.mColumn] = tf();
            ++mNextLoadPosition;
            if (!mNextLoadPosition) {
                mState = State::Filled;
            }            
        }
        
        
        
        inline static Position mCursorPosition;
        inline static Position mNextLoadPosition;
        inline static Menu* mMenu = nullptr;
        inline static State mState = State::Filled;
    };    
}
