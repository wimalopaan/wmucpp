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
#include "mcu/ports.h"
#include "mcu/avr/delay.h"

#include "std/pair.h"
#include "std/types.h"

namespace LCD { 

struct Lcd1x8  {};
struct Lcd1x16 {};
struct Lcd1x20 {};
struct Lcd1x40 {};
struct Lcd2x8  {};
struct Lcd2x12 {};
struct Lcd2x16 {};
struct Lcd2x20 {};
struct Lcd2x24 {};
struct Lcd2x40 {};
struct Lcd4x16 {};
struct Lcd4x20 {};

template<typename Type>
struct Parameter;

template<>
struct Parameter<Lcd2x8> {
    static constexpr uint8_t rows = 2;
    static constexpr uint8_t cols = 8;
    static constexpr uint8_t rowStartAddress[rows] = {0x00, 0x40};
    typedef Splitted_NaN<uint8_t, 1, 3> position_t;
};
template<>
struct Parameter<Lcd2x16> {
    static constexpr uint8_t rows = 2;
    static constexpr uint8_t cols = 16;
    static constexpr uint8_t rowStartAddress[rows] = {0x00, 0x40};
    typedef Splitted_NaN<uint8_t, 1, 4> position_t;
};
template<>
struct Parameter<Lcd2x20> {
    static constexpr uint8_t rows = 2;
    static constexpr uint8_t cols = 20;
    static constexpr uint8_t rowStartAddress[rows] = {0x00, 0x40};
    typedef Splitted_NaN<uint8_t, 1, 5> position_t;
};


enum class Instruction : uint8_t {
    clear    = (1 << 0),
    home     = (1 << 1),
    
    mode     = (1 << 2),
    shift    = (1 << 0),
    increment= (1 << 1),
    decrement= (0 << 1),
    
    control  = (1 << 3),
    blink    = (1 << 0),
    cursorOn = (1 << 1),
    displayOn= (1 << 2),
    
    cdshift  = (1 << 4),
    dshift   = (1 << 3),
    right    = (1 << 2),
    
    function = (1 << 5),
    I8bit    = (1 << 4),
    I4bit    = (0 << 4),
    twoLines = (1 << 3),
    oneLine  = (0 << 3),
    bigFont  = (1 << 2),
    
    cgram    = (1 << 6),
    ddram    = (1 << 7),
    readBusy = (1 << 7)
};
}

template<>
struct std::enable_bitmask_operators<LCD::Instruction> {
    static constexpr const bool enable = true;    
};

namespace LCD {

struct Row {
    uint8_t value = 0;
};
struct Column {
    uint8_t value = 0;
};

template<typename Data, typename RS, typename RW, typename E, typename Type>
class HD44780 final {
    HD44780() = delete;
    
    static_assert(Data::size == 4, "wrong number of pins in PinSet");

public:
    static constexpr auto enableDelay = 20_us;
    
    typedef Parameter<Type> param_type;
    typedef typename Parameter<Type>::position_t position_t;
    
    static void init() {
        Data::template dir<AVR::Output>();
        RS::template dir<AVR::Output>();
        RW::template dir<AVR::Output>();
        E::template dir<AVR::Output>();
        Data::allOff();
        RS::off();
        RW::off();
        E::off();
        Util::delay(16_ms);
        
        Data::set(static_cast<uint8_t>(Instruction::function | Instruction::I8bit) >> 4);
        toggle<E>();
        Util::delay(5_ms);
        
        Data::set(static_cast<uint8_t>(Instruction::function | Instruction::I8bit) >> 4);
        toggle<E>();
        Util::delay(1_ms);
        
        Data::set(static_cast<uint8_t>(Instruction::function | Instruction::I8bit) >> 4);
        toggle<E>();
        Util::delay(1_ms);
        
        Data::set(static_cast<uint8_t>(Instruction::function | Instruction::I4bit) >> 4);
        toggle<E>();
        Util::delay(5_ms);
        
        writeCommand(Instruction::control | Instruction::displayOn | Instruction::cursorOn | Instruction::blink);
        writeCommand(Instruction::clear);
        writeCommand(Instruction::home);
        writeCommand(Instruction::mode | Instruction::increment);
        
        Data::template dir<AVR::Input>();
    }
    static void clear() {
        writeCommand(Instruction::clear);
    }
    static void home() {
        writeCommand(Instruction::home);
        actualRow = 0;
    }
    static void writeData(uint8_t data) {
        waitBusy();
        RS::high();
        write(data);
    }
    static void writeCommand(Instruction instruction) {
        waitBusy();
        RS::low();
        write(static_cast<uint8_t>(instruction));
    }
    static void writeAddress(uint8_t a) {
        waitBusy();
        RS::low();
        write(static_cast<uint8_t>(Instruction::ddram) | (a & 0x7f));
    }
    static uint8_t readData() {
        RS::high();
        return read();
    }
    static uint8_t readCommand() {
        RS::low();
        return read();
    }
    static bool put(char c) {
        if (c == '\n') {
            actualRow = (actualRow + 1) % param_type::rows;
            setPosition(Row{actualRow}, Column{0});
        }
        else {
            auto xy = position();            
            if (xy) {
                writeData(c);        
            }
            else {
                return false;
            }
        }
        return true;
    }
    template<uint16_t Size>
    static void put(const volatile std::array<uint8_t, Size>& data) {
        setPosition(Row{0}, Column{0});
        auto it = data.begin();
        for(uint8_t row = 0; row < param_type::rows; ++row) {
            for(uint8_t column = 0; column < param_type::cols; ++column) {
                put(*it++);
            }
            put('\n');
        }
    }

    static position_t position() {
        uint8_t address = waitBusy();
        for(uint8_t row = 0; row < Parameter<Type>::rows; ++row) {
            if ((param_type::rowStartAddress[row] <= address) && 
                    (address < (param_type::rowStartAddress[row] + param_type::cols))) {
                actualRow = row;
                uint8_t column = address - param_type::rowStartAddress[row];
                return position_t(row, column);
            }
        }
        return position_t();
    }
//    static std::optional<std::pair<uint8_t, uint8_t>> position() {
//        uint8_t address = waitBusy();
//        for(uint8_t row = 0; row < Parameter<Type>::rows; ++row) {
//            if ((param_type::rowStartAddress[row] <= address) && 
//                    (address < (param_type::rowStartAddress[row] + param_type::cols))) {
//                actualRow = row;
//                return std::pair<uint8_t, uint8_t>{row, static_cast<uint8_t>(address - param_type::rowStartAddress[row])};
//            }
//        }
//        return{};
//    }
    static void setPosition(Row row, Column column) {
        actualRow = row.value;
        writeAddress(param_type::rowStartAddress[row.value] + column.value);
    }

private:
    static void write(uint8_t data) {
        RW::low();        
        Data::template dir<AVR::Output>();
        Data::set((data >> 4) & 0x0f);
        toggle<E>();
        Data::set(data & 0x0f);
        toggle<E>();
        Data::template dir<AVR::Input>();
    }
    static uint8_t read() {
        RW::high();
        Data::template dir<AVR::Input>();
        E::high();
        Util::delay(enableDelay);
        uint8_t data = Data::read() << 4;
        E::low();
        Util::delay(enableDelay);
        E::high();
        Util::delay(enableDelay);
        data |= Data::read() & 0x0f;
        E::low();
        return data;
    }
    static uint8_t waitBusy() {
        while (readCommand() & static_cast<uint8_t>(Instruction::readBusy));
        Util::delay(4_us);
        return readCommand(); // Address Counter
    }
    template<typename Pin>
    static void toggle() {
        Pin::on();
        Util::delay(enableDelay);
        Pin::off();
    }
    
    inline static uint8_t actualRow = 0;
};

}
