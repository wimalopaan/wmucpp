/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#define NDEBUG

#include <mcu/avr.h>

#include <mcu/common/util.h>

#include <mcu/internals/systemclock.h>
#include <mcu/internals/cppm.h>
#include <mcu/internals/constantrate.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>

#include <etl/output.h>
#include <etl/fixedpoint.h>
#include <etl/scoped.h>
#include <etl/meta.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/hott.h>
#include <external/hott/menu.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/solutions/ifx007.h>
#include <external/solutions/rpm.h>
#include <external/solutions/ppm.h>

#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>

namespace  {
    using namespace std::literals::chrono;
    using namespace External::Units;
    constexpr auto interval = Hott::hottDelayBetweenBytes;
//    constexpr auto fpwm = hertz{1000};
    constexpr auto fpwm = Project::Config::fMcu;
    
    constexpr RPM MaximumRpm{12000};
    constexpr RPM MinimumRpm{100};
}

// Timer 0: Systemclock
// Timer 1: Pwm
// Timer 3: Rpm
// Timer 4: Ppm-In

using systemClock = AVR::SystemTimer<AVR::Component::Timer<0>, interval>;
using alarmTimer = External::Hal::AlarmTimer<systemClock>;

using PortB = AVR::Port<AVR::B>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;
using PortE = AVR::Port<AVR::E>;

using pc1 = AVR::Pin<PortC, 3>; // debug: nc
using pc0 = AVR::Pin<PortC, 4>; // debug: nc

using sensorTxEnable = AVR::ActiveLow<AVR::Pin<PortD, 3>, AVR::Output>;

using inh1 = AVR::Pin<PortD, 5>;
using inh2 = AVR::Pin<PortB, 0>;

using qtPA = External::QtRobo::ProtocollAdapter<0, 16>;
using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;

using sensor = Hott::Experimental::Sensor<AVR::Component::Usart<0>, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemClock>;
using rcUsart = AVR::Usart<AVR::Component::Usart<1>, qtPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using terminalDevice = rcUsart;
using terminal = etl::basic_ostream<terminalDevice>;
using log = External::QtRobo::Logging<terminal, 0>;

using pwm = AVR::PWM::DynamicPwm<AVR::Component::Timer<1>>;
using hbridge = External::IFX007::HBridge<pwm, sumd::value_type>;

using adc = AVR::Adc<AVR::Component::Adc<0>>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0>>;

using rpm = External::RpmWithIcp<AVR::Component::Timer<3>, MinimumRpm, MaximumRpm>;

using ppm = External::Ppm::IcpPpm<AVR::Component::Timer<1>, External::Ppm::RisingEdge<false>>; 


struct Storage {
    enum class AVKey : uint8_t {Input = 0, Channel, PWMF1, PWMF2, PWMMode, PWMThr1, PWMThr2, SplitMode, _Number};
    
    class ApplData : public EEProm::DataBase<ApplData> {
    public:
        etl::uint_NaN<uint8_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<etl::uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues{};
    };
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

class Dashboard final : public Hott::Menu<> {
public:
    Dashboard() : Menu(this, "Dashboard"_pgm
                    ) {}
private:
//    Hott::TextWithValue<Storage::AVKey, 3, Storage::ApplData> mMode{"Mode"_pgm, appData, Storage::AVKey::SplitMode};
};

// todo: Zusammenfassen: DropDown(...)

struct YesNo : public Hott::TextWithValue<Storage::AVKey, Storage::ApplData> {
    YesNo(const AVR::Pgm::StringView& title, Storage::ApplData& data, Storage::AVKey k) :
        TextWithValue(title, data, k, 1) {}

    virtual void valueToText(uint8_t value, etl::span<3, etl::Char> buffer) const override{
        if (value == 0) {
            buffer.insertLeftFill("No"_pgm);
        }
        else {
            buffer.insertLeftFill("Yes"_pgm);
        }
    }
};

struct Connection : public Hott::TextWithValue<Storage::AVKey, Storage::ApplData, 4> {
    Connection(const AVR::Pgm::StringView& title, Storage::ApplData& data, Storage::AVKey k) :
        TextWithValue(title, data, k, 2) {}

    virtual void valueToText(uint8_t value, etl::span<4, etl::Char> buffer) const override{
        if (value == 0) {
            buffer.insertLeftFill("SUMD"_pgm);
        }
        else if (value == 1) {
            buffer.insertLeftFill("CPPM"_pgm);
        }
        else if (value == 2) {
            buffer.insertLeftFill("PPM"_pgm);
        }
        else {
            buffer.insertLeftFill("err"_pgm);
        }
    }
};

struct PWMMode : public Hott::TextWithValue<Storage::AVKey, Storage::ApplData, 5> {
    PWMMode(const AVR::Pgm::StringView& title, Storage::ApplData& data, Storage::AVKey k) :
        TextWithValue(title, data, k, 1) {}

    virtual void valueToText(uint8_t value, etl::span<5, etl::Char> buffer) const override{
        if (value == 0) {
            buffer.insertLeftFill("Dual"_pgm);
        }
        else if (value == 1) {
            buffer.insertLeftFill("Split"_pgm);
        }
        else {
            buffer.insertLeftFill("err"_pgm);
        }
    }
};

class SplitModeMenu final : public Hott::Menu<> {
public:
    SplitModeMenu() : Menu(this, "SplitMode"_pgm,
                    &mMode
                    ) {}
private:
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mMode{"Mode"_pgm, appData, Storage::AVKey::SplitMode, 2};
};

class PWMMenu final : public Hott::Menu<> {
public:
    PWMMenu() : Menu(this, "PWM"_pgm,
                    &mPWMF1, &mPWMF2, &mPWMThr1, &mPWMThr2, &mPWMMode
                    ) {}
private:
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mPWMF1{"PWM Freq 1"_pgm, appData, Storage::AVKey::PWMF1, 20};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mPWMF2{"PWM Freq 2"_pgm, appData, Storage::AVKey::PWMF2, 20};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mPWMThr1{"PWM Thr 1"_pgm, appData, Storage::AVKey::PWMThr1, 100};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mPWMThr2{"PWM Thr 2"_pgm, appData, Storage::AVKey::PWMThr2, 100};
    PWMMode mPWMMode{"PWM Mode"_pgm, appData, Storage::AVKey::PWMMode};
};


class RCMenu final : public Hott::Menu<> {
public:
    RCMenu() : Menu(this, "WM BDC ESC 1.0"_pgm,
                    &mSource, &mChannel, &mPwm, &mSplitMode, &mDashboard
                    ) {}
private:
    Connection mSource{"Input Conn."_pgm, appData, Storage::AVKey::Input};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mChannel{"Channel"_pgm, appData, Storage::AVKey::Channel, 8};
    PWMMenu mPwm;
    SplitModeMenu mSplitMode;
    Dashboard mDashboard;
};

template<typename PA, typename TopMenu>
class HottMenu final {
    HottMenu() = delete;
public:
    inline static void init() {
        clear();
    }
    inline static void periodic() {
        if (auto k = PA::key(); k != Hott::key_t::nokey) {
            processKey(k);
        }
        mMenu->textTo(PA::text());
    }
    inline static void processKey(Hott::key_t key) {
        assert(mMenu);
        if (auto m = mMenu->processKey(key); m != mMenu) {
            mMenu = m;
            clear();
        }
    }
private:
    inline static void clear() {
        for(auto& line : PA::text()) {
            line.clear();
        }
    }
    inline static TopMenu mTopMenu;
    inline static Hott::Menu<>* mMenu = &mTopMenu;
};

using menu = HottMenu<sensor, RCMenu>;

namespace {
    using Initializer = AVR::Util::StaticInitializer<systemClock, sensor, sensorTxEnable, eeprom, hbridge, rpm, menu, ppm>;
    Initializer initializer;
}


int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    pc0::dir<Output>();
    pc1::dir<Output>();

    inh1::dir<Output>();
    inh1::on();

    inh2::dir<Output>();
    inh2::on();

    rcUsart::init<BaudRate<115200>>();
    
    pwm::init<fpwm>();
    
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

    {
        Scoped<EnableInterrupt<>> ei;

//        sensorTxEnable::activate();
        
        hbridge::duty(Hott::SumDMsg::Mid);
        
        auto escData = Hott::Experimental::Adapter<Hott::EscMsg>(sensor::data());
        
        escData.rpmRaw(121);
        escData.rpmMaxRaw(632);
        escData.voltageRaw(134);
        escData.voltageMinRaw(114);
        escData.currentRaw(3);
        escData.currentMaxRaw(27);
        escData.tempRaw(600);
        escData.tempMaxRaw(400);
        
        while(true) {
            pc1::toggle();
            
            AVR::Util::Periodic<sensor, rcUsart, hbridge, menu, rpm, ppm>::periodic();
            
            systemClock::periodic([&](){ // every exact_intervall ms
                sensor::ratePeriodic();
                pc0::toggle();
//                auto v = sumd::value(0);
                auto v = qtPA::propValues[0];
                hbridge::duty(v);
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t) {
                        rpm::check();
//                        etl::outl<terminal>("p: "_pgm, hbridge::x1);
//                        etl::outl<terminal>("t: "_pgm, hbridge::x2);
                        
//                        etl::outl<terminal>("v: "_pgm, v.toInt());
                        etl::outl<terminal>("v: "_pgm, v);
                        appData.expire();
                    }
                });
            });
            if(eeprom::saveIfNeeded()) {
                etl::out<terminal>("."_pgm);
            }    
        }
    }
}

