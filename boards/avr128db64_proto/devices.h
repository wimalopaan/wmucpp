#pragma once

#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/spi.h>
#include <mcu/internals/twi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/dac.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>

#include <external/solutions/rotaryencoder.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/apa102.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>

#include <external/sbus/sbus.h>
#include <external/ibus/ibus2.h>
#include <external/sbus/sport.h>

#include <external/hal/adccontroller.h>

#include "hc05.h"
#include "ds3231.h"
#include "ssd1306.h"

namespace {
    using namespace std::literals::chrono;
    using namespace External::Units::literals;
    inline static constexpr auto fRtc = 1000_Hz;
}

struct VersionProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::DIY;
    inline static constexpr auto ibus_type = IBus2::Type::type::ARMED;
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
#if defined(GITMAJOR) && defined(GITMINOR)
        static_assert(GITMINOR < 100);
        return GITMAJOR * 100 + GITMINOR;
#else
        return VERSION_NUMBER;
#endif
    }
};

template<auto HWRev = 0, typename MCU = DefaultMcuType>
struct Devices;

// HWRev 5
template<typename MCU>
struct Devices<4, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;
    
    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; 
    using usart1Position = AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Alt1>; 
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Default>; 
    using usart3Position = AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Default>; 
    using usart4Position = AVR::Portmux::Position<AVR::Component::Usart<4>, AVR::Portmux::Default>; 
    
    using spi0Position = AVR::Portmux::Position<AVR::Component::Spi<0>, AVR::Portmux::Default>;
    using spi1Position = AVR::Portmux::Position<AVR::Component::Spi<1>, AVR::Portmux::Default>;
    
    using twi0Position = AVR::Portmux::Position<AVR::Component::Twi<0>, AVR::Portmux::Default>;
    using twi1Position = AVR::Portmux::Position<AVR::Component::Twi<1>, AVR::Portmux::Default>;
    
    using la0 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 4>, AVR::Output>; 
    using la1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 5>, AVR::Output>; 
    using la2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 6>, AVR::Output>; 
    using la3 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 7>, AVR::Output>; 
    
    using led1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 2>, AVR::Output>; 
    using led2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 3>, AVR::Output>; 
    
    using blinkLed1 = External::Blinker2<led1, systemTimer, 200_ms, 2000_ms>;
    using blinkLed2 = External::Blinker2<led2, systemTimer, 300_ms, 2000_ms>;
    
    using serial1 = AVR::Usart<usart3Position, External::Hal::NullProtocollAdapter<>, etl::NamedFlag<false>>;
    using serial2 = AVR::Usart<usart4Position, External::Hal::NullProtocollAdapter<>, etl::NamedFlag<false>>;
    
    using terminal1 = etl::basic_ostream<serial1>;
    using terminal2 = etl::basic_ostream<serial2>;
    
    using atbuffer = External::AT::Response<16>;
    using proto = External::QtRobo::ProtocollAdapter<0, 16, atbuffer>;
    using bluetoothUsart  = AVR::Usart<usart1Position, proto, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    using bluetoothEnable = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::C>, 1>, AVR::Output>; 
    using bluetoothPower  = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::G>, 1>, AVR::Output>; 
    using hc05 = External::HC05<"abc"_pgm, bluetoothUsart, bluetoothEnable, bluetoothPower, systemTimer, terminal1>;
    
    using ccl4Position = AVR::Portmux::Position<AVR::Component::Ccl<4>, AVR::Portmux::Default>; 
    using lut4 = AVR::Ccl::SimpleLut<4, AVR::Ccl::Input::Usart<0>, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask>;
    
    using sbus_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using sbus = AVR::Usart<usart0Position, sbus_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;
    
    using ccl2Position = AVR::Portmux::Position<AVR::Component::Ccl<2>, AVR::Portmux::Default>; 
    using lut2 = AVR::Ccl::SimpleLut<2, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<2> >;   
    //    using tempiP = External::InternalTempProvider<typename Devs::adcController, 2, typename Devs::sigrow, bus_type>;
    template<typename PA>
    using sensorUsart = AVR::Usart<usart2Position, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, Meta::List<VersionProvider>>;
        
    using ledSpi = AVR::Spi<spi0Position, AVR::QueueLength<128>,  AVR::UseInterrupts<false>>;
    using externSpi = AVR::Spi<spi1Position, AVR::QueueLength<128>,  AVR::UseInterrupts<false>>;
    
    using ledStripe = External::LedStripe<ledSpi, External::APA102, 8>;
    
    using oledI2C = AVR::Twi::Master<twi0Position>;
    using rtcI2C = AVR::Twi::Master<twi1Position>;
    
    using ds3231 = External::DS3231<rtcI2C, systemTimer>;
    
    using oled = External::SSD1306<oledI2C, systemTimer>;
    
    using buzz = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 2>, AVR::Output>; 
    
    using rotaryA = AVR::Pin<AVR::Port<AVR::G>, 4>; 
    using rotaryB = AVR::Pin<AVR::Port<AVR::G>, 5>; 
    using rotary = External::RotaryEncoder<rotaryA, rotaryB, uint8_t>;
    
    using rotaryButtonPin = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::G>, 6>, AVR::Input>; 
    using rotaryButton = External::Button2<rotaryButtonPin, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(3000_ms)>;
    
    using powerOn = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::G>, 2>, AVR::Output>;
    using powerSwitch = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::G>, 7>, AVR::Input>;
    
    using dac = AVR::DAC<0>;
    using dacpin = AVR::Pin<AVR::Port<AVR::D>, 6>;
    
    using sqwpin = AVR::Pin<AVR::Port<AVR::D>, 1>;
    
    using bec_en = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::A>, 5>, AVR::Output>; 
    
    using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<12>, AVR::Vref::V2_048>;
    using adcController = External::Hal::AdcController<adc, Meta::NList<4, 5, 0x42>>; // PD4=A_I, PD5 = A_Vin, 0x42 = temp
    using adc_i_t = adcController::index_type;
    
    // pwmA1 = PA3 = TCA0-WO3 (def) = LUT0
    // pwmA2 = PD0 = TCA0-WO0 (alt) 
    // pwmB1 = PG0 = TCA1-WO0 (alt)
    // pwmB2 = PG3 = TCA1-WO3 (alt) = LUT5
    
    using tca0Position = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltD>;    
    using pwm1 = AVR::PWM::DynamicPwm<tca0Position>;
    using ccl0Position = AVR::Portmux::Position<AVR::Component::Ccl<0>, AVR::Portmux::Default>;
    using lut0 = AVR::Ccl::SimpleLut<0, AVR::Ccl::Input::Tca0<0>, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask >;
    
    using tca1Position = AVR::Portmux::Position<AVR::Component::Tca<1>, AVR::Portmux::AltG>;    
    using pwm2 = AVR::PWM::DynamicPwm<tca1Position>;
    using ccl5Position = AVR::Portmux::Position<AVR::Component::Ccl<5>, AVR::Portmux::Default>;
    using lut5 = AVR::Ccl::SimpleLut<5, AVR::Ccl::Input::Tca1<0>, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask >;   
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, usart3Position, usart4Position, 
                                                          spi0Position, spi1Position, 
                                                          twi0Position, twi1Position,
                                                          ccl2Position, ccl4Position, ccl5Position,
                                                          tca0Position, tca1Position>>;
    
    static inline void init() {
        portmux::init();
        
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDxSeriesAll<MCU>) {
                clock::template init<Project::Config::fMcuMhz>();
            }
            else {
                static_assert(std::false_v<MCU>);
            }
        });
        systemTimer::init(); 
        
        la0::init();
        la1::init();
        la2::init();
        la3::init();
    }
};

// HWRev 3
template<typename MCU>
struct Devices<2, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;

    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; 
    using usart1Position = AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Alt1>; 
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Default>; 
    using usart3Position = AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Default>; 
    using usart4Position = AVR::Portmux::Position<AVR::Component::Usart<4>, AVR::Portmux::Default>; 
    
    using spi0Position = AVR::Portmux::Position<AVR::Component::Spi<0>, AVR::Portmux::Default>;
    using spi1Position = AVR::Portmux::Position<AVR::Component::Spi<1>, AVR::Portmux::Default>;
    
    using twi0Position = AVR::Portmux::Position<AVR::Component::Twi<0>, AVR::Portmux::Default>;
    using twi1Position = AVR::Portmux::Position<AVR::Component::Twi<1>, AVR::Portmux::Default>;
    
    using la0 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 4>, AVR::Output>; 
    using la1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 5>, AVR::Output>; 
    using la2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 6>, AVR::Output>; 
    using la3 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 7>, AVR::Output>; 
    
    using led1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 2>, AVR::Output>; 
    using led2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 3>, AVR::Output>; 

    using blinkLed1 = External::Blinker2<led1, systemTimer, 200_ms, 2000_ms>;
    using blinkLed2 = External::Blinker2<led2, systemTimer, 300_ms, 2000_ms>;

    using serial1 = AVR::Usart<usart3Position, External::Hal::NullProtocollAdapter<>, etl::NamedFlag<false>>;
    using serial2 = AVR::Usart<usart4Position, External::Hal::NullProtocollAdapter<>, etl::NamedFlag<false>>;

    using terminal1 = etl::basic_ostream<serial1>;
    using terminal2 = etl::basic_ostream<serial2>;
        
    using atbuffer = External::AT::Response<16>;
    using proto = External::QtRobo::ProtocollAdapter<0, 16, atbuffer>;
    using bluetoothUsart  = AVR::Usart<usart1Position, proto, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    using bluetoothEnable = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::C>, 1>, AVR::Output>; 
    using bluetoothPower  = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::G>, 1>, AVR::Output>; 
    using hc05 = External::HC05<"abc"_pgm, bluetoothUsart, bluetoothEnable, bluetoothPower, systemTimer, terminal1>;
    
    using ccl4Position = AVR::Portmux::Position<AVR::Component::Ccl<4>, AVR::Portmux::Default>; 
    using lut4 = AVR::Ccl::SimpleLut<4, AVR::Ccl::Input::Usart<0>, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask>;

    using sbus_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using sbus = AVR::Usart<usart0Position, sbus_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;
    
    using ccl2Position = AVR::Portmux::Position<AVR::Component::Ccl<2>, AVR::Portmux::Default>; 
    using lut2 = AVR::Ccl::SimpleLut<2, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<2> >;   
//    using tempiP = External::InternalTempProvider<typename Devs::adcController, 2, typename Devs::sigrow, bus_type>;
    template<typename PA>
    using sensorUsart = AVR::Usart<usart2Position, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, Meta::List<VersionProvider>>;
    
    
    using ledSpi = AVR::Spi<spi0Position, AVR::QueueLength<128>,  AVR::UseInterrupts<false>>;
    using externSpi = AVR::Spi<spi1Position, AVR::QueueLength<128>,  AVR::UseInterrupts<false>>;

    using ledStripe = External::LedStripe<ledSpi, External::APA102, 8>;
    
    using oledI2C = AVR::Twi::Master<twi0Position>;
    using rtcI2C = AVR::Twi::Master<twi1Position>;
    
    using ds3231 = External::DS3231<rtcI2C, systemTimer>;
    
    using oled = External::SSD1306<oledI2C, systemTimer>;
    
    using buzz = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 2>, AVR::Output>; 
    
    using rotaryA = AVR::Pin<AVR::Port<AVR::G>, 4>; 
    using rotaryB = AVR::Pin<AVR::Port<AVR::G>, 5>; 
    using rotary = External::RotaryEncoder<rotaryA, rotaryB, uint8_t>;

    using rotaryButtonPin = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::G>, 6>, AVR::Input>; 
    using rotaryButton = External::Button2<rotaryButtonPin, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(3000_ms)>;
    
    using powerOn = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::G>, 2>, AVR::Output>; 
    using powerSwitch = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::G>, 7>, AVR::Input>; 
    
    using dac = AVR::DAC<0>;
    using dacpin = AVR::Pin<AVR::Port<AVR::D>, 6>; 

    using sqwpin = AVR::Pin<AVR::Port<AVR::D>, 1>; 

    using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<12>, AVR::Vref::V2_048>;
    using adcController = External::Hal::AdcController<adc, Meta::NList<4, 5, 0x42>>; // PD4=A_I, PD5 = A_Vin, 0x42 = temp
    using adc_i_t = adcController::index_type;
    
    // pwmA1 = PA3 = TCA0-WO3 (def) = LUT0
    // pwmA2 = PD0 = TCA0-WO0 (alt D) 
    // pwmB1 = PG0 = TCA1-WO0 (alt G)
    // pwmB2 = PG3 = TCA1-WO3 (alt) = LUT5
    
    using tca0Position = AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltD>;    
    using pwm1 = AVR::PWM::DynamicPwm<tca0Position>;
    using ccl0Position = AVR::Portmux::Position<AVR::Component::Ccl<0>, AVR::Portmux::Default>;
    using lut0 = AVR::Ccl::SimpleLut<0, AVR::Ccl::Input::Tca0<0>, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask >;   
    
    using tca1Position = AVR::Portmux::Position<AVR::Component::Tca<1>, AVR::Portmux::AltG>;    
    using pwm2 = AVR::PWM::DynamicPwm<tca1Position>;
    using ccl5Position = AVR::Portmux::Position<AVR::Component::Ccl<5>, AVR::Portmux::Default>;
    using lut5 = AVR::Ccl::SimpleLut<5, AVR::Ccl::Input::Tca1<0>, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Mask >;   
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, usart3Position, usart4Position, 
                                                          spi0Position, spi1Position, 
                                                          twi0Position, twi1Position,
                                                          ccl2Position, ccl4Position, ccl5Position,
                                                          tca0Position, tca1Position>>;
    
    static inline void init() {
        portmux::init();
        
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDxSeriesAll<MCU>) {
                clock::template init<Project::Config::fMcuMhz>();
            }
            else {
                static_assert(std::false_v<MCU>);
            }
        });
        systemTimer::init(); 
        
        la0::init();
        la1::init();
        la2::init();
        la3::init();
    }
};

// HWRev 2
template<typename MCU>
struct Devices<1, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;

    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; 
    using usart1Position = AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Alt1>; 
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Default>; 
    using usart3Position = AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Default>; 
    using usart4Position = AVR::Portmux::Position<AVR::Component::Usart<4>, AVR::Portmux::Default>; 
    
    using spi0Position = AVR::Portmux::Position<AVR::Component::Spi<0>, AVR::Portmux::Default>;
    using spi1Position = AVR::Portmux::Position<AVR::Component::Spi<1>, AVR::Portmux::Default>;
    
    using twi0Position = AVR::Portmux::Position<AVR::Component::Twi<0>, AVR::Portmux::Default>;
    using twi1Position = AVR::Portmux::Position<AVR::Component::Twi<1>, AVR::Portmux::Default>;
    
    using la0 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 4>, AVR::Output>; 
    using la1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 5>, AVR::Output>; 
    using la2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 6>, AVR::Output>; 
    using la3 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 7>, AVR::Output>; 
    
    using led1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 2>, AVR::Output>; 
    using led2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 3>, AVR::Output>; 

    using blinkLed1 = External::Blinker2<led1, systemTimer, 200_ms, 2000_ms>;
    using blinkLed2 = External::Blinker2<led2, systemTimer, 300_ms, 2000_ms>;
    
    using serial1 = AVR::Usart<usart3Position, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>>;
    using serial2 = AVR::Usart<usart4Position, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>>;

    using terminal = etl::basic_ostream<serial1>;
    
    using atbuffer = External::AT::Response<16>;
    using bluetoothUsart  = AVR::Usart<usart1Position, External::Hal::NullProtocollAdapter<atbuffer>, etl::NamedFlag<false>>;
    using bluetoothEnable = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::C>, 1>, AVR::Output>; 
    using bluetoothPower  = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::G>, 1>, AVR::Output>; 
    using hc05 = External::HC05<"abc"_pgm, bluetoothUsart, bluetoothEnable, bluetoothPower, systemTimer, terminal>;
    
    using ccl4Position = AVR::Portmux::Position<AVR::Component::Ccl<4>, AVR::Portmux::Default>; 
    using lut4 = AVR::Ccl::SimpleLut<4, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<0>, AVR::Ccl::Input::Mask>;
    
    using ledSpi = AVR::Spi<spi0Position, AVR::QueueLength<128>,  AVR::UseInterrupts<false>>;
    using externSpi = AVR::Spi<spi1Position, AVR::QueueLength<128>,  AVR::UseInterrupts<false>>;

    using ledStripe = External::LedStripe<ledSpi, External::APA102, 8>;
    
    using oledI2C = AVR::Twi::Master<twi0Position>;
    using rtcI2C = AVR::Twi::Master<twi1Position>;
    
    using ds3231 = External::DS3231<rtcI2C, systemTimer>;
    
    using buzz = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 2>, AVR::Output>; 
    
    using rotaryA = AVR::Pin<AVR::Port<AVR::G>, 4>; 
    using rotaryB = AVR::Pin<AVR::Port<AVR::G>, 5>; 
    using rotary = External::RotaryEncoder<rotaryA, rotaryB, uint8_t>;

    using rotaryButtonPin = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::G>, 6>, AVR::Input>; 
    using rotaryButton = External::Button2<rotaryButtonPin, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(3000_ms)>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, usart3Position, usart4Position, 
                                                          spi0Position, spi1Position, 
                                                          twi0Position, twi1Position, ccl4Position>>;

    using powerOn = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::G>, 2>, AVR::Output>; 
    using powerSwitch = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::G>, 7>, AVR::Input>; 
    
    using dac = AVR::DAC<0>;
    using dacpin = AVR::Pin<AVR::Port<AVR::D>, 6>; 

    static inline void init() {
        portmux::init();
        
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDxSeriesAll<MCU>) {
                clock::template init<Project::Config::fMcuMhz>();
            }
            else {
                static_assert(std::false_v<MCU>);
            }
        });
        systemTimer::init(); 
        
        la0::init();
        la1::init();
        la2::init();
        la3::init();
    }
};

// HWRev 1
template<typename MCU>
struct Devices<0, MCU> {
    using ccp = AVR::Cpu::Ccp<>;
    using clock = AVR::Clock<>;
    using sigrow = AVR::SigRow<>;
    
    using systemTimer = AVR::SystemTimer<AVR::Component::Rtc<0>, fRtc>;

    using usart0Position = AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>; 
    using usart1Position = AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Alt1>; 
    using usart2Position = AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Default>; 
    using usart3Position = AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Default>; 
    using usart4Position = AVR::Portmux::Position<AVR::Component::Usart<4>, AVR::Portmux::Default>; 
    
    using spi0Position = AVR::Portmux::Position<AVR::Component::Spi<0>, AVR::Portmux::Default>;
    using spi1Position = AVR::Portmux::Position<AVR::Component::Spi<1>, AVR::Portmux::Default>;
    
    using twi0Position = AVR::Portmux::Position<AVR::Component::Twi<0>, AVR::Portmux::Default>;
    using twi1Position = AVR::Portmux::Position<AVR::Component::Twi<1>, AVR::Portmux::Default>;
    
    using la0 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 4>, AVR::Output>; 
    using la1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 5>, AVR::Output>; 
    using la2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 6>, AVR::Output>; 
    using la3 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 7>, AVR::Output>; 
    
    using led1 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 2>, AVR::Output>; 
    using led2 = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::E>, 3>, AVR::Output>; 

    using blinkLed1 = External::Blinker2<led1, systemTimer, 200_ms, 2000_ms>;
    using blinkLed2 = External::Blinker2<led2, systemTimer, 300_ms, 2000_ms>;
    
    using atbuffer = External::AT::Response<16>;
    using bluetoothUsart  = AVR::Usart<usart1Position, External::Hal::NullProtocollAdapter<atbuffer>, etl::NamedFlag<false>>;
    using bluetoothEnable = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::C>, 1>, AVR::Output>; 
    using bluetoothPower  = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::G>, 1>, AVR::Output>; 
    using hc05 = External::HC05<"abc"_pgm, bluetoothUsart, bluetoothEnable, bluetoothPower, systemTimer>;
    
    using ccl4Position = AVR::Portmux::Position<AVR::Component::Ccl<4>, AVR::Portmux::Default>; 
    using lut4 = AVR::Ccl::SimpleLut<4, AVR::Ccl::Input::Mask, AVR::Ccl::Input::Usart<0>, AVR::Ccl::Input::Mask>;
    
    using ledSpi = AVR::Spi<spi0Position, AVR::QueueLength<128>,  AVR::UseInterrupts<false>>;
    using externSpi = AVR::Spi<spi1Position, AVR::QueueLength<128>,  AVR::UseInterrupts<false>>;

    using ledStripe = External::LedStripe<ledSpi, External::APA102, 8>;
    
    using oledI2C = AVR::Twi::Master<twi0Position>;
    using rtcI2C = AVR::Twi::Master<twi1Position>;
    
    using buzz = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::A>, 2>, AVR::Output>; 
    
    using rotaryA = AVR::Pin<AVR::Port<AVR::G>, 4>; 
    using rotaryB = AVR::Pin<AVR::Port<AVR::G>, 5>; 
    using rotary = External::RotaryEncoder<rotaryA, rotaryB, uint8_t>;

    using rotaryButtonPin = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::G>, 6>, AVR::Input>; 
    using rotaryButton = External::Button2<rotaryButtonPin, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(3000_ms)>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, usart3Position, usart4Position, 
                                                          spi0Position, spi1Position, 
                                                          twi0Position, twi1Position, ccl4Position>>;

    using powerOn = AVR::ActiveHigh<AVR::Pin<AVR::Port<AVR::G>, 2>, AVR::Output>; 
    using powerSwitch = AVR::ActiveLow<AVR::Pin<AVR::Port<AVR::G>, 7>, AVR::Input>; 
    
    using dac = AVR::DAC<0>;

    using serial1 = AVR::Usart<usart3Position, External::Hal::NullProtocollAdapter<>, etl::NamedFlag<false>>;
    using serial2 = AVR::Usart<usart4Position, External::Hal::NullProtocollAdapter<>, etl::NamedFlag<false>>;
    
    static inline void init() {
        portmux::init();
        
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDxSeriesAll<MCU>) {
                clock::template init<Project::Config::fMcuMhz>();
            }
            else {
                static_assert(std::false_v<MCU>);
            }
        });
        systemTimer::init(); 
        
        la0::init();
        la1::init();
    }
};
