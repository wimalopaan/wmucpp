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
// A2c2r    2xStrom, 2xDrehzahl
// A2c2r1g  2xStrom, 2xDrehzahl, 1xGPS
// A2t2c    2xTemperatur, 2xStrom 
// A3t2r    2xTemperatur, 2xDrehzahl
// A4t      4xTemperatur

// H        Sensor (Hott)

// S        Sensor (S.Port)

// B Pwm-Out
// ----
// B 10c5      5-Kanäle ab Kanal 10

// PA1: IBUS / SBUS 
// PA2: DaisyEnable
// PA3: SO4 (AIN3) (rpm0) (tcb1 wo) (adr4)
// PA4:
// PA5: SO3 (AIN5) (tcb0 wo) (adr3) (GPS)
// PA6: 
// PA7: 
// PB3:
// PB2: Q0   (WO2) (rpm1)  (adr2)  (LED für Adress-Impulse)k
// PB1: SO1  (WO1) (AIN10) (adr1) (temp0)
// PB0: SO2  (WO0) (AIN11) (adr0) (temp1) (uni)

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;
using PortB = Port<B>;

using so4Pin = Pin<PortA, 3>; // beim Lernen kann hier externe LED angeschlossen werden
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
    inline static constexpr uint8_t NAdresses = 5;
    
    enum class Mode : uint8_t {
        Graupner8K, // 2 long
        Graupner4K, // 2 long
        Robbe,  // 1 short
        CP, // 1 long
        XXX // 2 short
    }; 
    
    template<typename Channel>
    struct SwitchConfig {
        using channel_type = Channel;
        channel_type& passThru() {
            return mPassThruChannel;
        }
    private:
        channel_type mPassThruChannel{};
    };
    
    enum class AVKey : uint8_t {Magic, 
                                Ch0, Ch1, Ch2, Ch3, Ch4, Ch5, Ch6, Ch7,
                                Undefined, 
                                _Number};
    
    struct ChannelIndex {
        using channel_type = etl::uint_ranged<uint8_t, 0, 7>;
        using addr_type = etl::uint_ranged<uint8_t, 0, 4>;
        addr_type address;
        channel_type channel;
    };
    
    template<typename Channel, typename AddressR>
    struct ApplData final : public EEProm::DataBase<ApplData<Channel, AddressR>> {
//                Channel::_;
        using Address = etl::uint_ranged_NaN<typename AddressR::value_type, AddressR::Lower, AddressR::Upper>;
//        Address::_;
        using value_type = SwitchConfig<Channel>;

        Channel& passThru(const ChannelIndex& i) {
            return AValues[i.address][i.channel].passThru();               
        }
        
        value_type& sw(const ChannelIndex& i) {
            return AValues[i.address][i.channel];    
        }
        uint8_t& magic() {
            return mMagic;
        }
        void clear() {
            for(auto& adr : AValues) {
                for(auto& v : adr) {
                    v = value_type{};
                }
            }
            for(auto& v : mMpxModes) {
                v = Mode::Graupner8K;
            }
        }
        Channel& channel() {
            return mChannel;
        }
        Address& address() {
            return mAddress;
        }
        void mpxMode(uint8_t addressOffset, uint8_t v) {
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
                return  mMpxModes[addressOffset];
            }
            return Mode::Graupner8K;
        }
    private:
        std::array<Mode, NAdresses> mMpxModes {};
        uint8_t mMagic;
        Channel mChannel;
        Address mAddress;
        std::array<std::array<value_type, static_cast<uint8_t>(AVKey::_Number)>, NAdresses> AValues;
    };
}

