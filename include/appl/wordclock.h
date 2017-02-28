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

enum wColor {cSubst = 0, cVerb, cMinute, cPrep, cQuarter, cHour, cMinuteRemainder, cOff, cNumberOfColors = cOff};
typedef enum wColor color_t;

enum word {Es = 0,
           Ist,
           Fuenf1,
           Zehn1,
           Zwanzig,
           Drei1,
           Viertel,
           Nach,
           Vor,
           Halb,
           Zwoelf,
           Zwei,
           Ein,
           Eins,
           Sieben,
           Drei2,
           Fuenf2,
           Elf,
           Neun,
           Vier,
           Acht,
           Zehn2,
           Sechs,
           Uhr,
           Minute1,
           Minute2,
           Minute3,
           Minute4,
           UNDEF
          };


static uint8_t testCounter = 0;

struct wordCode {
    uint8_t startPosition;
    uint8_t length;
};

typedef struct wordCode wordCode_t;

#define ML 11
#define MR 10

#define LEDS  ((ML * MR) + 4)

static struct cRGB leds[LEDS] = {};

// Schlangenverdrahtung!
static const wordCode_t words[] PROGMEM = {
    {0 + 0 * ML, 2}, // Es
    {3 + 0 * ML, 3}, // Ist
    {7 + 0 * ML, 4}, // Fünf1
    {7 + 1 * ML, 4}, // Zehn1
    {0 + 1 * ML, 7}, // Zwanzig
    {0 + 2 * ML, 4}, // Drei1
    {4 + 2 * ML, 7}, // Viertel
    {5 + 3 * ML, 4}, // Nach
    {2 + 3 * ML, 3}, // Vor
    {0 + 4 * ML, 4}, // Halb
    {5 + 4 * ML, 5}, // Zwölf
    {7 + 5 * ML, 4}, // Zwei
    {6 + 5 * ML, 3}, // Ein
    {5 + 5 * ML, 4}, // Eins
    {0 + 5 * ML, 6}, // Sieben
    {1 + 6 * ML, 4}, // Drei2
    {7 + 6 * ML, 4}, // Fünf2
    {8 + 7 * ML, 3}, // Elf
    {4 + 7 * ML, 4}, // Neun
    {0 + 7 * ML, 4}, // Vier
    {1 + 8 * ML, 4}, // Acht
    {5 + 8 * ML, 4}, // Zehn2
    {5 + 9 * ML, 5}, // Sechs
    {0 + 9 * ML, 3}, // Uhr
    {0 + 10 * ML, 1}, // Minute1
    {0 + 10 * ML, 2}, // Minute2
    {0 + 10 * ML, 3}, // Minute3
    {0 + 10 * ML, 4}, // Minute4
    {2 + 0 * ML, 1}, // K
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

static struct cRGB colors[cNumberOfColors + 1] = {
{255,   0,   0},  // cSubst
{255, 255,   0},  // cVerb
{255,   0, 255},  // cMinute
{0,   255, 0  },  // cPrep
{0,   0,   255},  // cQuarter
{0,   255, 255},  // cHour
{ 64,  64,  64},  // cMinuteRemainder
{  0,   0,   0},  // cNumberOfColors = cOff
};

void scaleColors(uint8_t brightness)
{
    uint8_t c = 0;
    if (brightness > 150) {
        c = 255;
    }
    else if (brightness < 50) {
        c = 32;
    }
    else {
        c = (((int)brightness - 50) * (255 - 32)) / (150 - 50) + 32;
    }

    for(uint8_t i = 0; i < cNumberOfColors; ++i) {

    }
}

static uint8_t colorBase = 0;
static bool needToChangeColor = true;

void switchColorBase()
{
    colorBase = (colorBase + 1) % cNumberOfColors;
}

void wordClockSetLeds(enum word w, color_t color)
{
    if (w >= UNDEF) {
        return;
    }
    struct cRGB c = colors[(colorBase + color) % cNumberOfColors];
    for(uint8_t i = 0; (i < pgm_read_byte(&words[w].length)) && (i < ML); ++i) {
        leds[pgm_read_byte(&words[w].startPosition) + i] = c;
    }
}

void wordClockDisplay(uint8_t hour, uint8_t minute, uint8_t second)
{
    clearLeds();

    uint8_t sx = second / 10;
    uint8_t sy = second % 10;

    if (sy > 0) {
        for(uint8_t x = 0; x <= sx; ++x) {
            if (x < sx) {
                for(uint8_t y = 0; y < 10; ++y) {
                    uint8_t index = 0;
                    if ((y % 2) == 0) {
                        index = (MR - y) * 11 + x;
                    }
                    else {
                        index = 10 + (MR - y) * 11 - x;
                    }
                    const struct cRGB cs = {16, 16, 16};
                    leds[index] = cs;
                }
            }
            else {
                for(uint8_t y = 0; y <= sy; ++y) {
                    uint8_t index = 0;
                    if ((y % 2) == 0) {
                        index = (MR - y) * 11 + x;
                    }
                    else {
                        index = 10 + (MR - y) * 11 - x;
                    }
                    const struct cRGB cs = {16, 16, 16};
                    leds[index] = cs;
                }
            }
        }
    }

    if ((minute % 5) == 0) {
        if (needToChangeColor) {
            switchColorBase();
            needToChangeColor = false;
        }
    }
    else {
        needToChangeColor = true;
    }

    hour %= 12;

    if (hour == 0) {
        hour = 12;
    }

    wordClockSetLeds(Es, cSubst);
    wordClockSetLeds(Ist, cVerb);

    if (minute < 5) {
        wordClockSetLeds(Uhr, cHour);
    }
    else if (minute < 10) {
        wordClockSetLeds(Fuenf1, cMinute);
        wordClockSetLeds(Nach, cPrep);
    }
    else if (minute < 15) {
        wordClockSetLeds(Zehn1, cMinute);
        wordClockSetLeds(Nach, cPrep);
    }
    else if (minute < 20) {
        wordClockSetLeds(Viertel, cQuarter);
        wordClockSetLeds(Nach, cPrep);
    }
    else if (minute < 25) {
        wordClockSetLeds(Zwanzig, cMinute);
        wordClockSetLeds(Nach, cPrep);
    }
    else if (minute < 30) {
        wordClockSetLeds(Fuenf1, cMinute);
        wordClockSetLeds(Vor, cPrep);
        wordClockSetLeds(Halb, cQuarter);
        ++hour;
    }
    else if (minute < 35) {
        wordClockSetLeds(Halb, cQuarter);
        ++hour;
    }
    else if (minute < 40) {
        wordClockSetLeds(Fuenf1, cMinute);
        wordClockSetLeds(Nach, cPrep);
        wordClockSetLeds(Halb, cQuarter);
        ++hour;
    }
    else if (minute < 45) {
        wordClockSetLeds(Zehn1, cMinute);
        wordClockSetLeds(Nach, cPrep);
        wordClockSetLeds(Halb, cQuarter);
        ++hour;
    }
    else if (minute < 50) {
        wordClockSetLeds(Viertel, cQuarter);
        wordClockSetLeds(Vor, cPrep);
        ++hour;
    }
    else if (minute < 55) {
        wordClockSetLeds(Zehn1, cMinute);
        wordClockSetLeds(Vor, cPrep);
        ++hour;
    }
    else if (minute < 60) {
        wordClockSetLeds(Fuenf1, cMinute);
        wordClockSetLeds(Vor, cPrep);
        ++hour;
    }

    switch(hour) {
    case 1:
    case 13:
        if (minute < 5) {
            wordClockSetLeds(Ein, cHour);
        }
        else {
            wordClockSetLeds(Eins, cHour);
        }
        break;
    case 2:
        wordClockSetLeds(Zwei, cHour);
        break;
    case 3:
        wordClockSetLeds(Drei2, cHour);
        break;
    case 4:
        wordClockSetLeds(Vier, cHour);
        break;
    case 5:
        wordClockSetLeds(Fuenf2, cHour);
        break;
    case 6:
        wordClockSetLeds(Sechs, cHour);
        break;
    case 7:
        wordClockSetLeds(Sieben, cHour);
        break;
    case 8:
        wordClockSetLeds(Acht, cHour);
        break;
    case 9:
        wordClockSetLeds(Neun, cHour);
        break;
    case 10:
        wordClockSetLeds(Zehn2, cHour);
        break;
    case 11:
        wordClockSetLeds(Elf, cHour);
        break;
    case 12:
        wordClockSetLeds(Zwoelf, cHour);
        break;
    default:
        break;
    }

    switch (minute % 5) {
    case 1:
        wordClockSetLeds(Minute1, cMinuteRemainder);
        break;
    case 2:
        wordClockSetLeds(Minute2, cMinuteRemainder);
        break;
    case 3:
        wordClockSetLeds(Minute3, cMinuteRemainder);
        break;
    case 4:
        wordClockSetLeds(Minute4, cMinuteRemainder);
        needToChangeColor = true;
        break;
    default:
        break;
    }
    ws2812_setleds(leds, LEDS);
}

