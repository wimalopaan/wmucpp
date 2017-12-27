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

#include <cstdint>
#include <utility>

#include "mcu/ports.h"
#include "mcu/avr/delay.h"
#include "util/types.h"
#include "appl/command.h"
#include "container/pgmarray.h"

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
        static constexpr Util::PgmArray<uint8_t, 0x00, 0x40> rowStartAddress{};
        typedef Splitted_NaN<uint8_t, 1, 3> position_t; // todo: calculate bits
    };
    template<>
    struct Parameter<Lcd2x16> {
        static constexpr uint8_t rows = 2;
        static constexpr uint8_t cols = 16;
        static constexpr Util::PgmArray<uint8_t, 0x00, 0x40> rowStartAddress{};
        typedef Splitted_NaN<uint8_t, 1, 4> position_t;
    };
    template<>
    struct Parameter<Lcd2x20> {
        static constexpr uint8_t rows = 2;
        static constexpr uint8_t cols = 20;
        static constexpr Util::PgmArray<uint8_t, 0x00, 0x40> rowStartAddress{};
        typedef Splitted_NaN<uint8_t, 1, 5> position_t;
    };

    struct Row {
        uint8_t value = 0;
    };
    struct Column {
        uint8_t value = 0;
    };
    
    namespace HD44780 {
        namespace Instructions {
            struct Clear : Command::Value<0x01_B> {};
            struct Home : Command::Value<0x02_B>{};
            struct Mode : Command::Value<0x04_B>{};
            struct Control : Command::Value<0x08_B>{};
            struct Shift : Command::Value<0x10_B>{};
            struct Function : Command::Value<0x20_B>{};
            struct CGRam : Command::Value<0x40_B>{};
            struct DDRam : Command::Value<0x80_B>{};
            
            struct ShiftDisplay : Command::Option<Mode, 0x01_B> {};
            struct CursorIncrement : Command::Option<Mode, 0x02_B> {};
            struct CursorDecrement : Command::Option<Mode, 0x00_B> {};
            
            struct Blink : Command::Option<Control, 0x01_B> {};
            struct CursorOn : Command::Option<Control, 0x02_B> {};
            struct DisplayOn : Command::Option<Control, 0x04_B> {};
            
            struct DisplayShift : Command::Option<Shift, 0x08_B> {};
            struct ShiftRight : Command::Option<Shift, 0x04_B> {};
            
            struct I8bit : Command::Option<Function, 0x10_B> {};
            struct I4bit : Command::Option<Function, 0x00_B> {};
            struct TwoLines : Command::Option<Function, 0x08_B> {};
            struct BigFont : Command::Option<Function, 0x04_B> {};
            
            using commands = Command::CommandSet<Meta::List<Clear, Home, Mode, Control, Shift, Function, CGRam, DDRam>, 
                                                 Meta::List<ShiftDisplay, CursorIncrement, CursorDecrement, 
                                                            Blink, CursorOn, DisplayOn,
                                                            DisplayShift, ShiftRight, 
                                                            I8bit, TwoLines, BigFont>>;
        }
    }
    
    template<typename Data, typename RS, typename RW, typename E, typename Type>
    class HD44780Port final {
        HD44780Port() = delete;
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
            
            using namespace LCD::HD44780::Instructions;
            
            Data::set(UpperNibble::convert(commands::template value<Function, I8bit>()));
            toggle<E>();
            Util::delay(5_ms);
            
            Data::set(UpperNibble::convert(commands::template value<Function, I8bit>()));
            toggle<E>();
            Util::delay(1_ms);
            
            Data::set(UpperNibble::convert(commands::template value<Function, I8bit>()));
            toggle<E>();
            Util::delay(1_ms);
            
            Data::set(UpperNibble::convert(commands::template value<Function, I4bit>()));
            toggle<E>();
            Util::delay(5_ms);
            
            writeCommand<Control, DisplayOn, CursorOn, Blink>();
            writeCommand<Clear>();
            writeCommand<Home>();
            writeCommand<Mode, CursorIncrement>();
            
            Data::template dir<AVR::Input>();
        }
        static inline void clear() {
            writeCommand<HD44780::Instructions::Clear>();
        }
        static inline void home() {
            writeCommand<HD44780::Instructions::Home>();
            actualRow = 0;
        }
        static inline void writeData(std::byte data) {
            waitBusy();
            RS::high();
            write(data);
        }
        template<typename C, typename... OO>
        static inline void writeCommand() {
            waitBusy();
            RS::low();
            write(HD44780::Instructions::commands::template value<C, OO...>());
        }
        static inline void writeAddress(uint8_t a) {
            waitBusy();
            RS::low();
            write(HD44780::Instructions::DDRam::value | std::byte(a & 0x7f));
        }
        static inline std::byte readData() {
            RS::high();
            return read();
        }
        static inline std::byte readCommand() {
            RS::low();
            return read();
        }
        //todo: actualRow -> einer Typ
        static inline bool put(std::byte c) {
            if (c == std::byte{'\n'}) {
                uint8_t r = (actualRow + 1) % param_type::rows;
                setPosition(Row{r}, Column{0});
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
//        template<uint16_t Size>
//        static void put(const volatile std::array<std::byte, Size>& data) {
//            setPosition(Row{0}, Column{0});
//            auto it = data.begin();
//            for(uint8_t row = 0; row < param_type::rows; ++row) {
//                for(uint8_t column = 0; column < param_type::cols; ++column) {
//                    put(*it++);
//                }
//                put(std::byte{'\n'});
//            }
//        }
        
        static inline position_t position() {
            uint8_t address = std::to_integer<uint8_t>(waitBusy());
            for(uint8_t row = 0; row < Parameter<Type>::rows; ++row) {
                if ((param_type::rowStartAddress[row] <= address) && 
                        (address < (param_type::rowStartAddress[row] + param_type::cols))) {
                    actualRow = row;
                    uint8_t column = address - param_type::rowStartAddress[row];
                    return position_t{row, column};
                }
            }
            return position_t{};
        }
        static inline void setPosition(Row row, Column column) {
            actualRow = row.value;
            uint8_t address = param_type::rowStartAddress[actualRow] + column.value;
            assert(address <= 0x7f);
            writeAddress(address);
        }
        static inline uint8_t address() {
            return std::to_integer<uint8_t>(waitBusy());
        }        
    private:
        static inline void write(std::byte data) {
            RW::low();        
            Data::template dir<AVR::Output>();
            Data::set(UpperNibble::convert(data));
            toggle<E>();
            Data::set(LowerNibble::convert(data));
            toggle<E>();
            Data::template dir<AVR::Input>();
        }
        static inline std::byte read() {
            RW::high();
            Data::template dir<AVR::Input>();
            E::high();
            Util::delay(enableDelay);
            std::byte data = Data::read() << 4;
            E::low();
            Util::delay(enableDelay);
            E::high();
            Util::delay(enableDelay);
            data |= Data::read() & std::byte{0x0f};
            E::low();
            return data;
        }
        static inline std::byte waitBusy() {
            while (std::any(readCommand() & 0x80_B));
            Util::delay(4_us); // todo: paramtrierbar
            return readCommand(); // Address Counter
        }
        template<typename Pin>
        static inline void toggle() {
            Pin::on();
            Util::delay(enableDelay);
            Pin::off();
        }
        inline static uint8_t actualRow = 0;
    };
    
}
