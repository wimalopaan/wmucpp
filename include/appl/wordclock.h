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

#pragma once

#include <stdint.h>

#include "std/time.h"
#include "external/ws2812.h"
#include "container/pgmarray.h"

#include "timedisplays.h"

template<typename LedPin, typename ColorSequence = ColorSequenceRGB>
class WordclockDisplay final {
    WordclockDisplay() = delete;
public:
    struct WordLeds final {
        const uint8_t startPosition = 0;
        const uint8_t length = 0;
        static WordLeds createFrom(const std::array<uint8_t, 2>& bytes) {
            return {bytes[0], bytes[1]};
        }
    };

    static constexpr uint8_t mColumns = 11;
    static constexpr uint8_t mRows = 10;
    static constexpr uint8_t mNumberOfLeds = mColumns * mRows + 4;
    
    using leds = WS2812<mNumberOfLeds, LedPin, ColorSequence, true>;
    typedef typename leds::color_type Color;
    
    enum class WordColor : uint8_t {cSubst = 0, cVerb, cMinute, cPrep, cQuarter, cHour, cMinuteRemainder, cOff, cNumberOfColors = cOff};
    static constexpr uint8_t numberOfColors = static_cast<uint8_t>(WordColor::cNumberOfColors);
    
    enum class  Word : uint8_t {Es = 0, Ist, Fuenf1, Zehn1, Zwanzig, Drei1, Viertel, Nach, Vor, Halb, Zwoelf, Zwei, Ein, Eins, Sieben, Drei2,
                               Fuenf2, Elf, Neun, Vier, Acht, Zehn2, Sechs, Uhr, Minute1, Minute2, Minute3, Minute4, NumberOfWords};
    static constexpr uint8_t numberOfWords = static_cast<uint8_t>(Word::NumberOfWords);

    struct Constants {
        struct Words {
            static constexpr WordLeds Es{0 + 0 * mColumns, 2};
            static constexpr WordLeds Ist{3 + 0 * mColumns, 3}; // Ist
            static constexpr WordLeds Fuenf1{7 + 0 * mColumns, 4}; // Fünf1
            static constexpr WordLeds Zehn1{7 + 1 * mColumns, 4}; // Zehn1
            static constexpr WordLeds Zwanzig{0 + 1 * mColumns, 7}; // Zwanzig
            static constexpr WordLeds Drei1{0 + 2 * mColumns, 4}; // Drei1
            static constexpr WordLeds Viertel{4 + 2 * mColumns, 7}; // Viertel
            static constexpr WordLeds Nach{5 + 3 * mColumns, 4}; // Nach
            static constexpr WordLeds Vor{2 + 3 * mColumns, 3}; // Vor
            static constexpr WordLeds Halb{0 + 4 * mColumns, 4}; // Halb
            static constexpr WordLeds Zwoelf{5 + 4 * mColumns, 5}; // Zwölf
            static constexpr WordLeds Zwei{7 + 5 * mColumns, 4}; // Zwei
            static constexpr WordLeds Ein{6 + 5 * mColumns, 3}; // Ein
            static constexpr WordLeds Eins{5 + 5 * mColumns, 4}; // Eins
            static constexpr WordLeds Sieben{0 + 5 * mColumns, 6}; // Sieben
            static constexpr WordLeds Drei2{1 + 6 * mColumns, 4}; // Drei2
            static constexpr WordLeds Fuenf2{7 + 6 * mColumns, 4}; // Fünf2
            static constexpr WordLeds Elf{8 + 7 * mColumns, 3}; // Elf
            static constexpr WordLeds Neun{4 + 7 * mColumns, 4}; // Neun
            static constexpr WordLeds Vier{0 + 7 * mColumns, 4}; // Vier
            static constexpr WordLeds Acht{1 + 8 * mColumns, 4}; // Acht
            static constexpr WordLeds Zehn2{5 + 8 * mColumns, 4}; // Zehn2
            static constexpr WordLeds Sechs{5 + 9 * mColumns, 5}; // Sechs
            static constexpr WordLeds Uhr{0 + 9 * mColumns, 3}; // Uhr
            static constexpr WordLeds Minute1{0 + 10 * mColumns, 1}; // Minute1
            static constexpr WordLeds Minute2{0 + 10 * mColumns, 2}; // Minute2
            static constexpr WordLeds Minute3{0 + 10 * mColumns, 3}; // Minute3
            static constexpr WordLeds Minute4{0 + 10 * mColumns, 4}; // Minute4
            static constexpr WordLeds K{2 + 0 * mColumns, 1}; // K
            
            static constexpr PgmArray<WordLeds, Es, Ist, Fuenf1, Zehn1, Zwanzig, Drei1, Viertel, Nach, Vor, Halb, Zwoelf, Zwei, Ein, Eins,
            Sieben, Drei2, Fuenf2, Elf, Neun, Vier, Acht, Zehn2, Sechs, Uhr, Minute1, Minute2, Minute3, Minute4, K> words{};
        };
        struct Colors {
            static constexpr Color Subst{Red{255}};  // cSubst
            static constexpr Color Verb{Red{255}, Green{255}, Blue{0}};  // cVerb
            static constexpr Color Minute{Red{255},   Green{0}, Blue{255}};  // cMinute
            static constexpr Color Prep{Green{255}};  // cPrep
            static constexpr Color Quarter{Blue{255}};  // cQuarter
            static constexpr Color Hour{Red{0},   Green{255}, Blue{255}};  // cHour
            static constexpr Color MinuteRemainder{64};  // cMinuteRemainder
            static constexpr Color Off{0};  // cNumberOfColors = cOff
            static constexpr PgmArray<Color, Subst, Verb, Minute, Prep, Quarter, Hour, MinuteRemainder, Off> colors{};
        };
    };
    
    static void init() {
        leds::init();
        clear();
    }
    static void clear() {
        leds::off();
    }

    template<std::Clock Clock>
    static void set(const Clock& clock, TimeDisplay::Mode = TimeDisplay::Mode::Time) {
        if (!clock) return;
        
        DateTime::TimeTm t = clock.dateTime();
        auto minute = t.minutes().value;
        auto hour   = t.hours().value;
        auto seconds = t.seconds().value;
        
        leds::template set<false>(Color{0});
    
        if (seconds == 0) {
            switchColorBase();
        }   
        
        hour %= 12;
    
        if (hour == 0) {
            hour = 12;
        }
    
        wordClockSetLeds(Word::Es, WordColor::cSubst);
        wordClockSetLeds(Word::Ist, WordColor::cVerb);
    
        if (minute < 5) {
            wordClockSetLeds(Word::Uhr, WordColor::cHour);
        }
        else if (minute < 10) {
            wordClockSetLeds(Word::Fuenf1, WordColor::cMinute);
            wordClockSetLeds(Word::Nach, WordColor::cPrep);
        }
        else if (minute < 15) {
            wordClockSetLeds(Word::Zehn1, WordColor::cMinute);
            wordClockSetLeds(Word::Nach, WordColor::cPrep);
        }
        else if (minute < 20) {
            wordClockSetLeds(Word::Viertel, WordColor::cQuarter);
            wordClockSetLeds(Word::Nach, WordColor::cPrep);
        }
        else if (minute < 25) {
            wordClockSetLeds(Word::Zwanzig, WordColor::cMinute);
            wordClockSetLeds(Word::Nach, WordColor::cPrep);
        }
        else if (minute < 30) {
            wordClockSetLeds(Word::Fuenf1, WordColor::cMinute);
            wordClockSetLeds(Word::Vor, WordColor::cPrep);
            wordClockSetLeds(Word::Halb, WordColor::cQuarter);
            ++hour;
        }
        else if (minute < 35) {
            wordClockSetLeds(Word::Halb, WordColor::cQuarter);
            ++hour;
        }
        else if (minute < 40) {
            wordClockSetLeds(Word::Fuenf1, WordColor::cMinute);
            wordClockSetLeds(Word::Nach, WordColor::cPrep);
            wordClockSetLeds(Word::Halb, WordColor::cQuarter);
            ++hour;
        }
        else if (minute < 45) {
            wordClockSetLeds(Word::Zehn1, WordColor::cMinute);
            wordClockSetLeds(Word::Nach, WordColor::cPrep);
            wordClockSetLeds(Word::Halb, WordColor::cQuarter);
            ++hour;
        }
        else if (minute < 50) {
            wordClockSetLeds(Word::Viertel, WordColor::cQuarter);
            wordClockSetLeds(Word::Vor, WordColor::cPrep);
            ++hour;
        }
        else if (minute < 55) {
            wordClockSetLeds(Word::Zehn1, WordColor::cMinute);
            wordClockSetLeds(Word::Vor, WordColor::cPrep);
            ++hour;
        }
        else if (minute < 60) {
            wordClockSetLeds(Word::Fuenf1, WordColor::cMinute);
            wordClockSetLeds(Word::Vor, WordColor::cPrep);
            ++hour;
        }
    
        switch(hour) {
        case 1:
        case 13:
            if (minute < 5) {
                wordClockSetLeds(Word::Ein, WordColor::cHour);
            }
            else {
                wordClockSetLeds(Word::Eins, WordColor::cHour);
            }
            break;
        case 2:
            wordClockSetLeds(Word::Zwei, WordColor::cHour);
            break;
        case 3:
            wordClockSetLeds(Word::Drei2, WordColor::cHour);
            break;
        case 4:
            wordClockSetLeds(Word::Vier, WordColor::cHour);
            break;
        case 5:
            wordClockSetLeds(Word::Fuenf2, WordColor::cHour);
            break;
        case 6:
            wordClockSetLeds(Word::Sechs, WordColor::cHour);
            break;
        case 7:
            wordClockSetLeds(Word::Sieben, WordColor::cHour);
            break;
        case 8:
            wordClockSetLeds(Word::Acht, WordColor::cHour);
            break;
        case 9:
            wordClockSetLeds(Word::Neun, WordColor::cHour);
            break;
        case 10:
            wordClockSetLeds(Word::Zehn2, WordColor::cHour);
            break;
        case 11:
            wordClockSetLeds(Word::Elf, WordColor::cHour);
            break;
        case 12:
            wordClockSetLeds(Word::Zwoelf, WordColor::cHour);
            break;
        default:
            break;
        }
    
        switch (minute % 5) {
        case 1:
            wordClockSetLeds(Word::Minute1, WordColor::cMinuteRemainder);
            break;
        case 2:
            wordClockSetLeds(Word::Minute2, WordColor::cMinuteRemainder);
            break;
        case 3:
            wordClockSetLeds(Word::Minute3, WordColor::cMinuteRemainder);
            break;
        case 4:
            wordClockSetLeds(Word::Minute4, WordColor::cMinuteRemainder);
            break;
        default:
            break;
        }
        leds::write();
    }
    static const std::percent& brightness() {
        return mBrightness;
    }
    static void brightness(const std::percent& b) {
        mBrightness = b;
    }
 
private:
    inline static std::percent mBrightness = 100_ppc;
    inline static uint8_t mColorBase = 0;
    
    static inline void switchColorBase() {
        mColorBase = (mColorBase + 1) % numberOfColors;
    }
    
    static void wordClockSetLeds(Word w, WordColor color){
        auto c = Constants::Colors::colors[(mColorBase + static_cast<int>(color)) % numberOfColors] * brightness();

        for(uint8_t i = 0; (i < Constants::Words::words[static_cast<uint8_t>(w)].length) && (i < mColumns); ++i) {
            leds::template set<false>(Constants::Words::words[static_cast<uint8_t>(w)].startPosition + i, c);
        }
    }
};

// Z-Verdrahtung
/*
static wordCode_t words[] = {
    {0 + 0 * ML, 2}, // Es
    {3 + 0 * ML, 3}, // Ist
    {7 + 0 * ML, 4}, // Fünf1
    {0 + 1 * ML, 4}, // Zehn1
    {4 + 1 * ML, 6}, // Zwanzig
    {0 + 2 * ML, 4}, // Drei1
    {4 + 2 * ML, 6}, // Viertel
    {2 + 3 * ML, 4}, // Nach
    {6 + 3 * ML, 3}, // Vor
    {0 + 4 * ML, 4}, // Halb
    {5 + 4 * ML, 5}, // Zwölf
    {0 + 5 * ML, 4}, // Zwei
    {2 + 5 * ML, 3}, // Ein
    {2 + 5 * ML, 4}, // Eins
    {5 + 5 * ML, 6}, // Sieben
    {1 + 6 * ML, 4}, // Drei2
    {7 + 6 * ML, 4}, // Fünf2
    {0 + 7 * ML, 3}, // Elf
    {3 + 7 * ML, 4}, // Neun
    {7 + 7 * ML, 4}, // Vier
    {1 + 8 * ML, 4}, // Acht
    {5 + 8 * ML, 4}, // Zehn2
    {1 + 9 * ML, 5}, // Sechs
    {8 + 9 * ML, 3}, // Uhr
    {0 + 10 * ML, 1}, // Minute1
    {1 + 10 * ML, 1}, // Minute2
    {2 + 10 * ML, 1}, // Minute3
    {3 + 10 * ML, 1}, // Minute4
    {2 + 0 * ML, 1}, // K
};
*/




