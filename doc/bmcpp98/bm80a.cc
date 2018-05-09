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

//#define MEM
#define NDEBUG

#include "config.h"
#include <stdlib.h>
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "hal/constantrate.h"
#include "external/hott/hott.h"
#include "hal/alarmtimer.h"

#ifdef MEM
# include "util/memory.h"
#endif

using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

using ledPin = AVR::Pin<PortD, 4>;

using systemClock = AVR::Timer8Bit<0>; // timer 2
using alarmTimer = AlarmTimer<systemClock>;

#include "console.h"
#include "util/meta.h"
#include "external/hott/menu.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

namespace {
    constexpr bool useTerminal = true;
}
namespace Constants {
    static constexpr auto title = "Test 80"_pgm;
}

class TSensorId final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
public:
    TSensorId(uint8_t number) : mNumber{number} {
    }
    virtual void putTextInto(Hott::BufferString& buffer) const override {
        buffer.insertAtFill(0, "--:--:--:--:--:--"_pgm);
    }
private:
    uint8_t mNumber = 0;
};

class TSensorMenu final : public Hott::NumberedMenu {
public:
    TSensorMenu(Menu* parent, uint8_t n) : NumberedMenu(parent, n, "Sensor"_pgm, &mID), mID{n} {}
private:
    TSensorId mID{0};
};

class TemperaturSensorenMenu final : public Hott::Menu {
public:
    TemperaturSensorenMenu(Menu* parent) : Menu(parent, "Temp. Sensoren"_pgm, &mTS1, &mTS2, &mTS3, &mTS4) {}
private:
    TSensorMenu mTS1{this, 0};
    TSensorMenu mTS2{this, 1};
    TSensorMenu mTS3{this, 2};
    TSensorMenu mTS4{this, 3};
};

class TemperaturMenu final : public Hott::Menu {
public:
    TemperaturMenu(Hott::Menu* parent) : Hott::Menu(parent, "Temperatur"_pgm, &mSensoren) {}

private:
    TemperaturSensorenMenu mSensoren{this};
};

class SpannungMenu final : public Hott::Menu {
public:
    SpannungMenu(Hott::Menu* parent) : Menu(parent, "Spannung"_pgm) {}
private:
};

class DrehzahlMenu final : public Hott::Menu {
public:
    DrehzahlMenu(Hott::Menu* parent) : Menu(parent, "Drehzahl"_pgm) {}
private:
};

class StromMenu final : public Hott::Menu {
public:
    StromMenu(Hott::Menu* parent) : Menu(parent, "Strom"_pgm) {}
private:
};

class RCMenu final : public Hott::Menu {
public:
    RCMenu() : Menu(this, "WM SensorLed"_pgm, &mTemperatur, &mSpannung, &mDrehzahl, &mStrom) {}
private:
    TemperaturMenu mTemperatur{this};                        
    SpannungMenu   mSpannung{this};                        
    DrehzahlMenu   mDrehzahl{this};                        
    StromMenu   mStrom{this};                        
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
    inline static Hott::Menu* mMenu = &mTop;
};

struct MenuData {
    static Hott::Display& text() {
        return mData;
    }
    inline static Hott::Display mData;
};
using menuData = MenuData;

using menu = HottMenu<menuData, RCMenu>;

const auto periodicTimer = alarmTimer::create(1500_ms, AlarmFlags::Periodic);

struct TimerHandler : public EventHandler<EventType::Timer> {
    inline static bool process(std::byte b) {
        auto timer = std::to_integer<uint7_t>(b);
        if (timer == *periodicTimer) {
            std::outl<terminal>(Constants::title);
        }
        return true;
    }
    inline static uint8_t mCounter = 0;
};


int main() {
    using namespace std::literals::quantity;
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 

    using allEventHandler = EventHandlerGroup<TimerHandler>;

    {
        Scoped<EnableInterrupt<>> ei;
        EventManager::run2<allEventHandler>([](){
            menu::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([](){
                alarmTimer::rateProcess();
            });
            
        });
    }    
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
//    while(true) {}
}
#endif

