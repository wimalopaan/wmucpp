/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/event.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/sigrow.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>

#include <external/solutions/series01/sppm_in.h>
#include <external/solutions/tick.h>
#include <external/solutions/rc/tiptip.h>
#include <external/solutions/rc/grmulti.h>

#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>

#ifndef AUTO_BUS
# include <external/ibus/ibus.h>
#endif
#include <external/sbus/sbus.h>

#include <external/ibus/ibus2.h>
#include <external/solutions/rc/busscan.h>
#include <external/solutions/rc/multi.h>

#include <external/solutions/series01/swuart.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using led7 = Pin<Port<B>, 3>; 
using led6 = Pin<Port<A>, 7>; 
using led5 = Pin<Port<A>, 5>; // tca0 wo5
using led4 = Pin<Port<A>, 4>; // tca0 wo4
using led3 = Pin<Port<A>, 3>; // tca0 wo3
using led2 = Pin<Port<B>, 2>; // tca0 wo2
using led1 = Pin<Port<B>, 1>; // tca0 wo1
using led0 = Pin<Port<B>, 0>; // tca0 wo0

using ppmIn = Pin<Port<A>, 2>; 

using lvPin = Pin<Port<A>, 6>; 

using ledList = Meta::List<led0, led1, led2, led3, led4, led5, led6, led7>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using tcaPosition    = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using tcb0Position    = Portmux::Position<Component::Tcb<0>, Portmux::Default>;

using pwm = PWM::DynamicPwm8Bit<tcaPosition>;

using ppm = External::Ppm::SinglePpmIn<Component::Tcb<0>>; 

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcb0Position>>;
using portmux1 = Portmux::StaticMapper<Meta::List<usart0Position>>;

namespace  {
    constexpr auto dt = 2_ms;
#ifdef AUTO_BUS
    constexpr auto fRtc = 1000_Hz; // 1ms
#elif defined(USE_HOTT)
    constexpr auto fRtc = 500_Hz;
#elif defined(USE_SBUS)
    constexpr auto fRtc = 1000_Hz;
#elif defined(USE_IBUS)
    constexpr auto fRtc = 128_Hz;
#elif defined(USE_PPM)
    constexpr auto fRtc = 128_Hz;
#else
    constexpr auto fRtc = 1000_Hz; // 1ms
#endif
    
    constexpr double Ro = 4'700;
    constexpr auto RPs = []{
        const std::array<double, 3> Rx {4'700, 10'000, 20'000}; // 24K???
        std::array<double, 1 << Rx.size()> R;
        auto parallel = [](const double a, const double b){
            double z = a * b;
            double n = a + b;
            if (n != 0) {
                return z / n; 
            }
            return 0.0;
        };
        for(uint8_t i = 0; i < R.size(); ++i) {
            double r1 = 10'000'000.0;
            double r2 = 10'000'000.0;
            double r3 = 10'000'000.0;
            if (i & 0x01) {
                r1 = Rx[0];
            }
            if (i & 0x02) {
                r2 = Rx[1];
            }
            if (i & 0x04) {
                r3 = Rx[2];
            }
            const double ra = parallel(parallel(r1, r2), r3);
            R[i] = ra;
        }
        return R;
    }();
    
    constexpr auto JumperIntervalls = []{
        const double Vref = 4.3;
        const double amax = 1023;
        const double vmax = 5.0;
        std::array<std::pair<uint16_t, uint16_t>, RPs.size()> ii;
        for(uint8_t i = 0; const auto r: RPs) {
            const double v = vmax * r / (r + Ro); 
            const double vl = v * 0.96;
            const double al = vl * amax / Vref;
            const double vh = v * 1.04;
            const double ah = vh * amax / Vref;
            
            ii[i].first = std::min(amax, al);
            ii[i].second= std::min(amax, ah);
            ++i;
        }
        return ii;
    }();
       
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using ppm_channel = Event::Channel<0, Event::Generators::Pin<ppmIn>>; 
using ppm_user = Event::Route<ppm_channel, Event::Users::Tcb<0>>;
using evrouter = Event::Router<Event::Channels<ppm_channel>, Event::Routes<ppm_user>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<6, 0x1e>>; // 1e = temp

namespace  {
    inline uint8_t getConfigValue() {
        constexpr auto ch0 = adcController::index_type{0};
        const auto cv = adcController::value(ch0).toInt();
        for(uint8_t i = 0; i < JumperIntervalls.size(); ++i) {
            if ((cv >= JumperIntervalls[i].first) && (cv <= JumperIntervalls[i].second)) {
                return i;
            }
        }        
        return std::numeric_limits<uint8_t>::max();
    }
}

struct SoftTimer {
#ifdef USE_SBUS
    static constexpr auto intervall = 1_ms;
#else
    static constexpr auto intervall = 50_ms;
#endif
//    static constexpr auto intervall = 1 / fRtc;
};


namespace Storage {
#ifdef USE_SBUS
    using tick_type = External::Tick<SoftTimer, uint16_t, 8000>; // max 8 Seconds
#else
    using tick_type = External::Tick<SoftTimer, uint8_t>;
#endif
    struct Blink {
        tick_type duration{};
        tick_type intervall{};
    };
    enum class Mode : uint8_t {Graupner, // 2 long
                               Robbe,  // 1 short
                               CP, // 1 long
                               XXX // 2 short
                              }; 

    template<typename Channel, typename Address>
    struct SwitchConfig {
//        Address::_;
        using pwm_type = etl::uint_ranged_NaN<uint8_t, 0, pwm::pwmMax>;
        using channel_type = etl::uint_ranged_NaN<typename Channel::value_type, Channel::Lower, Channel::Upper>;
        using addr_type = etl::uint_ranged_NaN<typename Address::value_type, Address::Lower, Address::Upper>;
        using tick_t = tick_type;
        
        const auto& pwmValue() const {
            return mPwm;
        }
        auto& pwmValue() {
            return mPwm;
        }
        void pwmValue(const pwm_type v){
            mPwm = v;
        }
        auto& blinks() {
            return mBlinks;
        }
        auto& passThru() {
            return mPassThruChannel;
        }
    private:
        channel_type mPassThruChannel{};
        pwm_type mPwm{};
        std::array<Blink, 4> mBlinks;
    };
    
    enum class AVKey : uint8_t {Magic, 
                                Ch0, Ch1, Ch2, Ch3, Ch4, Ch5, Ch6, Ch7,
                                Undefined, 
                                _Number};
    
    static inline uint8_t keyToInt(const AVKey k) {
        return (uint8_t)k - (uint8_t)AVKey::Ch0;
    }
    
    template<typename Channel, typename Address = etl::uint_ranged_NaN<uint8_t, 0, 7>>
    struct ApplData final : public EEProm::DataBase<ApplData<Channel, Address>> {
        using value_type = SwitchConfig<Channel, Address>;
        
        void clear() {
            for(auto& v : AValues) {
                v = value_type{};
            }
        }
        
        value_type& operator[](const AVKey key) {
            if (key == AVKey::Undefined) {
                AValues[static_cast<uint8_t>(AVKey::Undefined)] = value_type{};
            }
            return {AValues[static_cast<uint8_t>(key)]};
        }
        template<typename T>
        value_type& operator[](const T i) {
            return operator[](mapToKey(i));
        }
        uint8_t& magic() {
            return mMagic;
        }
        value_type::channel_type& channel() {
            return mChannel;
        }
        value_type::addr_type& address() {
            return mAddress;
        }
        void mpxMode(uint8_t addressOffset, auto v) { // only for Protocoll check (not used here)
            if (addressOffset < mMpxModes.size()) {
                if (v == 0) {
                    mMpxModes[addressOffset] = Mode::Graupner;
                }
                else if (v == 1) {
                    mMpxModes[addressOffset] = Mode::Robbe;
                }
                else if (v == 2) {
                    mMpxModes[addressOffset] = Mode::CP;
                }
                else if (v == 3) {
                    mMpxModes[addressOffset] = Mode::XXX;
                }
                
            }
        }
        std::array<Mode, 5> mMpxModes; // only for Protocoll check (not used here)
    private:
        uint8_t mMagic;
        value_type::channel_type mChannel;
        value_type::addr_type    mAddress;
        template<typename T>
        inline Storage::AVKey mapToKey(const T i) {
            switch(i) {
            case 0:
                return Storage::AVKey::Ch0;
            case 1:
                return Storage::AVKey::Ch1;
            case 2:
                return Storage::AVKey::Ch2;
            case 3:
                return Storage::AVKey::Ch3;
            case 4:
                return Storage::AVKey::Ch4;
            case 5:
                return Storage::AVKey::Ch5;
            case 6:
                return Storage::AVKey::Ch6;
            case 7:
                return Storage::AVKey::Ch7;
            }
            return Storage::AVKey::Undefined;
        }
        std::array<value_type, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
}

