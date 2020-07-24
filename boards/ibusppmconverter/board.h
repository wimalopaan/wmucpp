#pragma once

#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/sleep.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/reset.h>
#include <mcu/internals/watchdog.h>
#include <mcu/internals/event.h>
#include <mcu/common/uninitialized.h>

#include <mcu/pgm/pgmarray.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>
#include <external/ibus/ibus.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/units/music.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/series01/sppm_out.h>

#include <std/chrono>
#include <etl/output.h>

//#define FS_I6S

// Config
// A        Sensor (I-Bus)
// ----
// A4t      4xTemperatur
// A2t2c    2xTemperatur, 2xStrom 
// A3t2r    3xTemperatur, 2xDrehzahl
// A2c2r    2xStrom, 2xDrehzahl
// A3c2r    3xStrom, 2xDrehzahl

// H        Sensor (Hott)

// S        Sensor (S.Port)

// B Pwm-Out
// ----
// B 10c5      5-Kanäle ab Kanal 10

// PA1: IBUS / SBUS 
// PA2: DaisyEnable
// PA3: SO4 (AIN3) (rpm0) (tcb1 wo) (adr4)
// PA4:
// PA5: SO3 (AIN5) (tcb0 wo) (adr3)
// PA6: 
// PA7: 
// PB3:
// PB2: Q0   (WO2) (rpm1)  (adr2)
// PB1: SO1  (WO1) (AIN10) (adr1)
// PB0: SO2  (WO0) (AIN11) (adr0)

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;
using PortB = Port<B>;

using so4Pin = Pin<PortA, 3>;
using so3Pin = Pin<PortA, 5>; // watch out for gps 
using q0Pin = Pin<PortB, 2>; 

using daisyChain= Pin<PortA, 2>; 

namespace Parameter {
    constexpr uint8_t menuLines = 8;
#ifdef USE_IBUS
    constexpr auto fRtc = 2000_Hz;
#endif
#ifdef USE_HOTT
    constexpr auto fRtc = 500_Hz;
#endif
#ifdef USE_SBUS
    // SBUS-Frame: 3ms
    // SBUS Rate: 7ms
    constexpr auto fRtc = 1000_Hz; // 1ms
#endif
    constexpr uint16_t R1vd = 10'000;
    constexpr uint16_t R2vd = 1'000;

    constexpr uint16_t Rc = 1'000;
    constexpr uint16_t Kc = 16'450;
}

using reset = Reset<>;

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

using wdt   = WatchDog<systemTimer::intervall>;
using uninitialzed = Uninitialized<>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;

template<typename Counter>
struct WdtProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::ARMED;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        return Counter::counter;
    }
};

using wdtP = WdtProvider<uninitialzed>;

struct IBusThrough {
    inline static void init() {
        daisyChain::template dir<Output>();
    }
    inline static void on() {
        daisyChain::on();
    }
    inline static void off() {
        daisyChain::off();
    }
};

using ibt = IBusThrough;

namespace Storage {
    
    enum class Mode : uint8_t {
        Graupner8K, // 2 long
        Graupner4K, // 2 long
        Robbe,  // 1 short
        CP, // 1 long
        XXX // 2 short
    }; 
    
//    using tick_type = External::Tick<SoftTimer, uint8_t>;
//    struct Blink {
//        tick_type duration{};
//        tick_type intervall{};
//    };

    template<typename Channel, typename Address>
    struct SwitchConfig {
//        using pwm_type = etl::uint_ranged_NaN<uint8_t, 0, pwm::pwmMax>;
//        using channel_type = etl::uint_ranged_NaN<uint8_t, 0, 17>;
//        Channel::_;
        using channel_type = Channel;
        using addr_type = Address;
//        using tick_t = tick_type;
        
//        const auto& pwmValue() const {
//            return mPwm;
//        }
//        auto& pwmValue() {
//            return mPwm;
//        }
//        void pwmValue(const pwm_type v){
//            mPwm = v;
//        }
//        auto& blinks() {
//            return mBlinks;
//        }
//        channel_type::_;
        channel_type& passThru() {
            return mPassThruChannel;
        }
    private:
        channel_type mPassThruChannel{};
//        pwm_type mPwm{};
//        std::array<Blink, 4> mBlinks;
    };
    
    enum class AVKey : uint8_t {Magic, 
                                Ch0, Ch1, Ch2, Ch3, Ch4, Ch5, Ch6, Ch7,
                                Undefined, 
                                _Number};
    
    template<typename Channel, typename Address>
    struct ApplData final : public EEProm::DataBase<ApplData<Channel, Address>> {
        using value_type = SwitchConfig<Channel, Address>;
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
        void clear() {
            for(auto& v : AValues) {
                v = value_type{};
            }
        }
        value_type::channel_type& channel() {
            return mChannel;
        }
        value_type::addr_type& address() {
            return mAddress;
        }
        void mpxMode(uint8_t addressOffset, auto v) {
            if (addressOffset < mMpxModes.size()) {
                if (v == 0) {
                    mMpxModes[addressOffset] = Mode::Graupner8K;
                }
                else if (v == 1) {
                    mMpxModes[addressOffset] = Mode::Graupner4K;
                }
                else if (v == 2) {
                    mMpxModes[addressOffset] = Mode::Robbe;
                }
                else if (v == 3) {
                    mMpxModes[addressOffset] = Mode::CP;
                }
                else if (v == 4) {
                    mMpxModes[addressOffset] = Mode::XXX;
                }
                else {
                    mMpxModes[addressOffset] = Mode::Graupner8K;
                }
            }
        }
        Mode mpxMode(uint8_t addressOffset) {
            if (addressOffset < mMpxModes.size()) {
                const Mode v = mMpxModes[addressOffset];
                if (v > Mode::XXX) {
                    return Mode::Graupner8K;
                }
                return v;
            }
            return Mode::Graupner8K;
        }
    private:
        std::array<Mode, 5> mMpxModes;
        uint8_t mMagic;
        value_type::channel_type mChannel;
        value_type::addr_type mAddress;
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

