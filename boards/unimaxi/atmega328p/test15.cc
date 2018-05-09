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

//#define NDEBUG

#include <stdlib.h>

#include "main.h"

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/twislave.h"
#include "mcu/avr/spi.h"
#include "hal/alarmtimer.h"
#include "hal/eeprom.h"
#include "hal/ressource.h"
#include "hal/softpwm.h"
#include "external/lcd.h"
#include "external/ws2812.h"
#include "console.h"

// 16 MHz full swing
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

// 8Mhz int
// sudo avrdude -p atmega328p -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using fetPin = AVR::Pin<PortB, 1>;
using lcdPinSet = AVR::PinSet<fetPin>;
using lcdPwm = HAL::SoftPWM<lcdPinSet, uint8_t>;

using ledPin = AVR::Pin<PortD, 4>;
using led = WS2812<1, ledPin>;
typedef led::color_type Color;

// Timer0
using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer = AlarmTimer<systemTimer, UseEvents<false>>;

const auto periodicTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);

using terminalDevice = AVR::Usart<0, NullProtocollAdapter, MCU::UseInterrupts<false>, UseEvents<false>, AVR::ReceiveQueueLength<64>>;
using terminal = std::basic_ostream<terminalDevice>;

using flagRegister = AVR::RegisterFlags<typename DefaultMcuType::GPIOR, 0, std::byte>;

constexpr TWI::Address address{0x59_B};
template<typename RessourceFlags>
using i2c_r = TWI::Slave<0, address, 2 * 16, MCU::UseInterrupts<false>, RessourceFlags>;

template<typename Flags>
struct EEPromData : EEProm::DataBase<EEPromData<Flags>, Flags> {
    uint8_t value1;
    uint8_t value2;
};

template<typename Flags>
using spi_f = AVR::Spi<0, AVR::SpiSlave<MCU::UseInterrupts<false>>, Flags>;

using controller = Hal::Controller<flagRegister, spi_f, EEPromData, i2c_r>;

using eedata = controller::get<EEPromData>;
using i2c = controller::get<i2c_r>;
using spi = controller::get<spi_f>;

using eeprom = EEProm::Controller<eedata>;
auto& appData = eeprom::data();

using LcdDB4 = AVR::Pin<PortC, 0>;
using LcdDB5 = AVR::Pin<PortC, 1>;
using LcdDB6 = AVR::Pin<PortC, 2>;
using LcdDB7 = AVR::Pin<PortC, 3>;

using LcdRS = AVR::Pin<PortD, 6>;
using LcdRW = AVR::Pin<PortD, 5>;
using LcdE  = AVR::Pin<PortD, 7>;

using LcdData = AVR::PinSet<AVR::UsePgmTable, LcdDB4, LcdDB5, LcdDB6, LcdDB7>;

using lcd = LCD::HD44780Port<LcdData, LcdRS, LcdRW, LcdE, LCD::Lcd2x16>;

int main() {
    led::init();
    eeprom::init();
    i2c::init();
    spi::init();
    lcdPwm::init();    
    lcd::init();
    
    led::off();
    
    {
        using namespace std::literals::quantity;
        lcdPwm::pwm(20_ppc, 0);
    }
    
    terminalDevice::init<9600>();
    alarmTimer::init(AVR::TimerMode::CTCNoInt);

    std::outl<terminal>("Test15"_pgm);
    Util::put<lcd>("Test15\nabc"_pgm);
    
    
    while(true) {
        lcdPwm::freeRun();
        terminalDevice::periodic();
        if (auto c = terminalDevice::get()) {
            appData.change();
            std::outl<terminal>("*"_pgm);
        }
        systemTimer::periodic<systemTimer::flags_type::ocfa>([](){
            alarmTimer::periodic([](uint7_t timer) {
                if (timer == *periodicTimer) {
                    std::outl<terminal>("tick"_pgm);
                    appData.expire();
                }
            });
        });
        while(eeprom::saveIfNeeded()) {
            std::outl<terminal>("."_pgm);
        }
        i2c::whenReady([]{
            i2c::changed(false);
        });
        spi::whenReady([](std::byte b){
            terminalDevice::put(b);
        });
    }
}
#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif
