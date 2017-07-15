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
#include "std/tuple.h"

namespace Hott {

    enum class AVKey : uint8_t {TSensor1, TSensor2, RpmSensor1, RpmSensor2, Spannung1, Spannung2, Strom, PWM, Leds1, Leds2, _Number};
    std::array<uint8_t, static_cast<uint8_t>(AVKey::_Number)> AValues;

    namespace Menu {
        struct Info {
            explicit constexpr Info(const PgmStringView& s) : mText(s) {}
            const PgmStringView mText = ""_pgm;
        };
        struct Presence {
            explicit constexpr Presence(const PgmStringView& s) : mText(s) {}
            const PgmStringView mText = ""_pgm;
        };

        template<typename... T>
        struct Menu {
            explicit constexpr Menu(const PgmStringView& title, const T&... items) : mTitle(title), mItems(items...) {}
            const PgmStringView mTitle = ""_pgm;
            std::tuple<T...> mItems;
        };        
    }    


    
    
    template<typename Buffer>
    class MenuSystem {
        inline static constexpr auto rows = Buffer::rows;
        inline static constexpr auto columns = Buffer::columns;
        
        inline static Menu::Menu startMenu = Menu::Menu{"WM Multi RC V0.1"_pgm, 
                                                         Menu::Menu("Temperatur"_pgm,
                                                                     Menu::Info("Bla"_pgm)
                                                                    ),
                                                         Menu::Menu("Spannung"_pgm)
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

        
        static void loadMenuData() {
//            if (mMenu == nullptr) return;
            if (!mNextLoadPosition) {
            }    
            ++mNextLoadPosition;
            if (!mNextLoadPosition) {
                mState = State::Filled;
            }            
        }
        
        
        
        inline static Position mCursorPosition;
        inline static Position mNextLoadPosition;
//        inline static Menu* mMenu = nullptr;
        inline static State mState = State::Filled;
    };    
}
