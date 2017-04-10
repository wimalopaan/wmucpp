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

//#define NDEBUG

#include "main.h"
#include "std/limits.h"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/usart.h"
#include "hal/event.h"
#include "mcu/ports.h"
#include "units/physical.h"
#include "hal/alarmtimer.h"
#include "std/literals.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/spi.h"
#include "mcu/avr/swusart.h"
#include "external/ws2812.h"
#include "mcu/avr/pinchange.h"
#include "mcu/avr/ppm.h"
#include "mcu/avr/adc.h"
#include "hal/ppmswitch.h"
#include "util/fsm.h"
#include "util/delay.h"
#include "external/hott/hott.h"
#include "console.h"
#include "hal/softspimaster.h"
#include "hal/button.h"
#include "external/dcf77.h"
#include "hal/softppm.h"
#include "mcu/avr/mcupwm.h"
#include "hal/softpwm.h"
#include "hal/constantrate.h"
#include "hal/variablerate.h"
#include "hal/bufferedstream.h"
#include "hal/adccontroller.h"
#include "external/onewire.h"
#include "external/ds18b20.h"
#include "external/ds1307.h"
#include "external/lm35.h"
#include "external/i2cram.h"
#include "external/rpm.h"
#include "mcu/avr/twimaster.h"
#include "std/array.h"

#define MEM
#ifdef MEM
#include "util/memory.h"
#endif

// 20MHz full-swing
// sudo avrdude -p atmega1284P -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd1:m -U efuse:w:0xfc:m

#define OW
#define I2C
#define PPMIN
#define PPMOUT
#define HOTT
#define DCF
#define LMADC
#define SLAVE84
#define SLAVE85
#define SLAVE25

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using SoftSPIData = AVR::Pin<PortA, 0>;
using SoftSPIClock = AVR::Pin<PortA, 1>;
using SoftSPISS = AVR::Pin<PortA, 2>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;

#ifdef OW
using oneWirePin = AVR::Pin<PortA, 5>;
using oneWireMaster = OneWire::Master<oneWirePin, OneWire::Normal>;
using oneWireMasterAsync = OneWire::MasterAsync<oneWireMaster, Hott::hottDelayBetweenBytes>;
using ds18b20 = DS18B20<oneWireMasterAsync>;
std::array<OneWire::ow_rom_t, 5> dsIds;
#endif

//using terminal = SSpi0;
using bufferedTerminal = BufferedStream<SSpi0, 512>;

namespace std {
    std::basic_ostream<bufferedTerminal> cout;
//    std::basic_ostream<terminal> cout;
    std::lineTerminator<CRLF> endl;
}

using button0 = Button<0, AVR::Pin<PortA, 3>>;
using button1 = Button<1, AVR::Pin<PortA, 4>>;
using buttonController = ButtonController<button0, button1>;

#ifdef DCF
using dcfPin = AVR::Pin<PortA, 7>;
using dcfDecoder = DCF77<dcfPin, Config::Timer::frequency, EventManager, true>;
#endif

#ifdef LMADC
using adc = AVR::Adc<0>;
using adcController = AdcController<adc, 6>;
using lm35 = LM35<adcController, 0>;
#endif

#ifdef I2C
using TwiMaster = TWI::Master<0>;
using TwiMasterAsync = TWI::MasterAsync<TwiMaster>;
using ds1307 = DS1307<TwiMasterAsync>;

#ifdef SLAVE84
static constexpr TWI::Address i2cramAddress{0x53};
using i2cram = I2CGeneric<TwiMasterAsync, i2cramAddress>;
#endif

#ifdef SLAVE85
static constexpr TWI::Address i2cledAddress{0x54};
using i2cled = I2CGeneric<TwiMasterAsync, i2cledAddress, I2CLedParameter>;
#endif

#ifdef SLAVE25
static constexpr TWI::Address i2crpmAddress{0x55};
using i2crpm = I2CGeneric<TwiMasterAsync, i2crpmAddress, I2CRpmParameter>;
#endif

#endif

using vrAdapter = VariableRateAdapter<bufferedTerminal
#ifdef LMADC
, adcController
#endif
>;

using ws2812_A = AVR::Pin<PortC, 2>;
//using ws2812_B = AVR::Pin<PortC, 2>;

#ifdef PPMIN
using ppmInputPin = AVR::Pin<PortC, 3>;
using ppmPinSet = AVR::PinSet<ppmInputPin>;
using pinChangeHandlerPpm = AVR::PinChange<ppmPinSet>;
using ppmTimerInput = AVR::Timer8Bit<2>;
using ppm1 = PpmDecoder<pinChangeHandlerPpm, ppmTimerInput>;
using ppmSwitch = PpmSwitch<0, ppm1>;
#endif

using systemClock = AVR::Timer8Bit<0>;
using systemTimer = AlarmTimer<systemClock>;

#ifdef HOTT
using sensorUsart = AVR::Usart<0, Hott::SensorProtocollAdapter<0>> ;
using rcUsart = AVR::Usart<1, Hott::SumDProtocollAdapter<0>>;
//using sensorUsart = AVR::Usart<0, Hott::NullPA<0>> ;
//using rcUsart = AVR::Usart<1, Hott::NullPA<1>>;
#endif

#ifdef PPMOUT
using ppmTimerOutput = AVR::Timer16Bit<3>;
using ppmPin1 = AVR::Pin<PortC, 4>;
using ppmPin2 = AVR::Pin<PortC, 5>;
using softPpm = SoftPPM<ppmTimerOutput, ppmPin1, ppmPin2>;
#endif

#ifdef HPWM
using hardPwm = AVR::PWM<2>;
#endif

using constantRateTimer = AVR::Timer16Bit<1>;
#ifdef HOTT
using crWriterSensorBinary = ConstanteRateWriter<Hott::SensorProtocollBuffer<0>, sensorUsart>;
using crWriterSensorText = ConstanteRateWriter<Hott::SensorTextProtocollBuffer<0>, sensorUsart>;
//using crAdapter = ConstantRateAdapter<crTimer, AVR::ISR::Timer<1>::CompareA, crWriterSensorBinary, crWriterSensorText, TestBitShifter<crTestPin, 0x55>>;
using crAdapterHott = ConstantRateAdapter<constantRateTimer, AVR::ISR::Timer<1>::CompareA, crWriterSensorBinary, crWriterSensorText>;
#else
using crAdapterHott = ConstantRateAdapter<crTimer, AVR::ISR::Timer<1>::CompareA>;
#endif
#ifdef OW
#ifndef HOTT
using crAdapterOneWire = ConstantRateAdapter<crTimer, AVR::ISR::Timer<1>::CompareA, oneWireMasterAsync>;
#else
using crAdapterOneWire = ConstantRateAdapter<void, AVR::ISR::Timer<1>::CompareA, oneWireMasterAsync>;
#endif
#endif
using isrDistributor = IsrDistributor<AVR::ISR::Timer<1>::CompareA 
#ifdef HOTT
, crAdapterHott
#endif
#ifdef OW
, crAdapterOneWire
#endif
>;

#ifdef SPWM
using softPwmPin1 = AVR::Pin<PortB, 5>;
using softPwmPin2 = AVR::Pin<PortB, 6>;
using softPwm = SoftPWM<softPwmPin1, softPwmPin2>;
#endif

using sampler = PeriodicGroup<AVR::ISR::Timer<0>::CompareA, buttonController, systemTimer
#ifdef DCF
, dcfDecoder
#endif
#ifdef SPWM
, softPwm
#endif
                                >; // werden alle resolution ms aufgerufen

using led0 = AVR::Pin<PortC, 6>;
using led1 = AVR::Pin<PortC, 7>;

//using rpm = RpmFromInterruptSource<0, ppmTimerInput>;

struct EventHandlerParameter {
    std::optional<uint7_t> timerId1;
};

struct NullPAHandler: public EventHandler<EventType::NullPAEvent> {
    static bool process(uint8_t v) {
        std::cout << "Null PA"_pgm << v << std::endl;
        return true;
    }  
};

struct I2CRpmHandler: public EventHandler<EventType::I2CRpmValueAvailable> {
    static bool process(uint8_t v) {
        std::cout << "i2c rpm value: "_pgm << v << std::endl;
        return true;
    }  
};

struct I2CRpmErrorHandler: public EventHandler<EventType::I2CRpmError> {
    static bool process(uint8_t) {
        std::cout << "i2c rpm error"_pgm << std::endl;
        return true;
    }
};

struct I2CLedHandler: public EventHandler<EventType::I2CLedValueAvailable> {
    static bool process(uint8_t v) {
        std::cout << "i2c led value: "_pgm << v << std::endl;
        return true;
    }  
};

struct I2CLedErrorHandler: public EventHandler<EventType::I2CLedError> {
    static bool process(uint8_t) {
        std::cout << "i2c led error"_pgm << std::endl;
        return true;
    }
};

struct I2CRamHandler: public EventHandler<EventType::I2CRamValueAvailable> {
    static bool process(uint8_t v) {
        std::cout << "i2c ram value: "_pgm << v << std::endl;
        return true;
    }  
};

struct I2CRamErrorHandler: public EventHandler<EventType::I2CRamError> {
    static bool process(uint8_t) {
        std::cout << "i2c ram error"_pgm << std::endl;
        return true;
    }
};

struct DCFReceive0Handler : public EventHandler<EventType::DCFReceive0> {
    static bool process(uint8_t n) {
        std::cout << "dcf 0 : "_pgm << n << std::endl;
        return true;
    }  
};
struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
    static bool process(uint8_t n) {
        std::cout << "dcf 1 : "_pgm << n << std::endl;
        return true;
    }  
};
struct DCFSyncHandler : public EventHandler<EventType::DCFSync> {
    static bool process(uint8_t) {
#ifdef DCF
        std::cout << "dcf sync  "_pgm << dcfDecoder::dateTime() << std::endl;
#endif
        return true;
    }  
};
struct DCFErrorHandler : public EventHandler<EventType::DCFError> {
    static bool process(uint8_t) {
        std::cout << "dcf error"_pgm << std::endl;
        return true;
    }  
};
struct DCFParityHandler : public EventHandler<EventType::DCFParityError> {
    static bool process(uint8_t) {
        std::cout << "dcf parity error"_pgm << std::endl;
        return true;
    }  
};

struct DS1307handler: public EventHandler<EventType::DS1307TimeAvailable> {
    static bool process(uint8_t) {
        std::cout << "ds1307 time"_pgm << std::endl;
        return true;
    }  
};

struct DS1307handlerError: public EventHandler<EventType::DS1307Error> {
    static bool process(uint8_t) {
        std::cout << "ds1307 error"_pgm << std::endl;
        return true;
    }  
};

struct TWIHandlerError: public EventHandler<EventType::TWIError> {
    static bool process(uint8_t) {
        std::cout << "twi error"_pgm << std::endl;
        return true;
    }  
};

struct DS18B20MeasurementHandler: public EventHandler<EventType::DS18B20Measurement> {
    static bool process(uint8_t) {
#ifdef OW
        std::cout << "t: " << ds18b20::temperature() << std::endl;
#endif
        return true;
    }
};
struct DS18B20ErrorHandler: public EventHandler<EventType::DS18B20Error> {
    static bool process(uint8_t) {
        std::cout << "t: error" << std::endl;
        return true;
    }
};

struct  Button0Handler: public EventHandler<EventType::ButtonPress0> {
    static bool process(uint8_t) {
        std::cout << "button 0 press"_pgm << std::endl;
        return true;
    }
};

struct HottBinaryHandler : public EventHandler<EventType::HottBinaryRequest> {
    static bool process(uint8_t) {
//        std::cout << "hbb"_pgm << std::endl;
#ifdef HOTT
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        crAdapterHott::start();
#endif
        return true;
    }
};

struct HottKeyHandler : public EventHandler<EventType::HottAsciiKey> {
    static bool process(uint8_t v) {
        std::cout << "k: "_pgm << v << std::endl;
//        Hott::SensorProtocoll<sensorUsart>::key(v);
        return true;
    }
};

struct HottBroadcastHandler : public EventHandler<EventType::HottSensorBroadcast> {
    static bool process(uint8_t) {
        std::cout << "hbr"_pgm << std::endl;
#ifdef HOTT
        crWriterSensorBinary::enable<true>();
        crWriterSensorText::enable<false>();
        crAdapterHott::start();
#endif
        return true;
    }
};

struct HottTextHandler : public EventHandler<EventType::HottAsciiRequest> {
    static bool process(uint8_t) {
        std::cout << "hba"_pgm << std::endl;
#ifdef HOTT
        crWriterSensorBinary::enable<false>();
        crWriterSensorText::enable<true>();
        crAdapterHott::start();
#endif
        return true;
    }
};

decltype(systemTimer::create(1000_ms, AlarmFlags::Periodic)) pTimer;
decltype(systemTimer::create(1000_ms, AlarmFlags::Periodic)) mTimer;
decltype(systemTimer::create(1000_ms, AlarmFlags::Periodic)) tTimer;

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(uint8_t timer) {
        if (timer == *pTimer) {

#ifdef MEM
            std::cout << "unused: "_pgm << Util::Memory::getUnusedMemory() << std::endl;   
#endif

            static uint8_t count = 0;
//            WS2812<2, ws2812_A>::set({16, (uint8_t)((count++ % 2) * 16), 16});
            
#ifdef I2C
            if (count % 2) {
#ifdef SLAVE84
                i2cram::startWrite(0, count);
#endif
#ifdef SLAVE85
                i2cled::startWrite(0, count * 64);
#endif
#ifdef SLAVE25
                i2crpm::startWrite(0, 0);
#endif
            }
            else {
#ifdef SLAVE84
                if (!i2cram::startRead(0)) {
                    std::cout << "start read ram error"_pgm << std::endl;
                }
#endif
#ifdef SLAVE85
                if (!i2cled::startRead(0)) {
                    std::cout << "start read led error"_pgm << std::endl;
                }
#endif
#ifdef SLAVE25
                if (!i2crpm::startRead(0)) {
                    std::cout << "start read rpm error"_pgm << std::endl;
                }
#endif
            }
            ds1307::startReadTimeInfo();
#endif
#ifdef LMADC
            std::cout << "Temp lm35: "_pgm << lm35::temperature() << std::endl;
#endif
#ifdef PPMIN
            std::cout << "ppm:"_pgm << ppm1::value<0>() << std::endl;
#endif
            std::cout << "c0: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(0) << std::endl;
    //        std::cout << "c1: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(1) << std::endl;
    //        std::cout << "c2: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(2) << std::endl;
    //        std::cout << "c3: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(3) << std::endl;
    //        std::cout << "c4: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(4) << std::endl;
    //        std::cout << "c5: "_pgm << Hott::SumDProtocollAdapter<0>::value8Bit(5) << std::endl;
    //        std::cout << "nn: "_pgm << Hott::SumDProtocollAdapter<0>::numberOfChannels() << std::endl;
    
            std::percent pv = std::scale(Hott::SumDProtocollAdapter<0>::value8Bit(0),
                                   Hott::SumDMsg::Low8Bit, Hott::SumDMsg::High8Bit);
            std::cout << "pv: " << pv.value() << std::endl;
            
#ifdef PPMOUT
            softPpm::ppm(pv, 0);
#endif
#ifdef SPWM
            softPwm::pwm(pv, 0);
            softPwm::pwm(pv, 1);
            std::cout << "spwm period: "_pgm << softPwm::period() << std::endl;
#endif
        }
        else if (timer == *tTimer) {
#ifdef OW
            if (ds18b20::convert()) {
                systemTimer::start(*mTimer);
            }
#endif
        }
        else if (timer == *mTimer) {
#ifdef OW
            ds18b20::startGet(dsIds[0]);
#endif
        }
        return true;
    }
};

struct TestHandler : public EventHandler<EventType::Test> {
    static bool process(uint8_t) {
        std::cout << "t"_pgm << std::endl;
        return true;
    }
};

struct PpmUpHandler : public EventHandler<EventType::Ppm1Up> {
    static bool process(uint8_t) {
        std::cout << "ppm1up"_pgm << std::endl;
        return true;
    }
};
struct PpmDownHandler : public EventHandler<EventType::Ppm1Down> {
    static bool  process(uint8_t) {
        std::cout << "ppm1down"_pgm << std::endl;
        return true;
    }
};
struct UsartHandler : public EventHandler<EventType::UsartRecv0> {
    static bool process(uint8_t v) {
        std::cout << "u: "_pgm << v << std::endl;
        return true;
    }
};
struct UsartFeHandler : public EventHandler<EventType::UsartFe> {
    static bool process(uint8_t v) {
        std::cout << "usart fe: "_pgm << v << std::endl;
        return true;
    }
};
struct UsartUpeHandler : public EventHandler<EventType::UsartUpe> {
    static bool process(uint8_t v) {
        std::cout << "usart upe: "_pgm << v << std::endl;
        return true;
    }
};
struct UsartDorHandler : public EventHandler<EventType::UsartDor> {
    static bool process(uint8_t v) {
        std::cout << "usart dor: "_pgm << v << std::endl;
        return true;
    }
};

using isrRegistrar = IsrRegistrar<
#ifdef PPMIN
            ppm1, 
#endif
                                isrDistributor, sampler
#ifdef HOTT
, sensorUsart::RxHandler, sensorUsart::TxHandler, rcUsart::RxHandler, rcUsart::TxHandler
#endif
#ifdef PPMOUT
                                  ,softPpm::OCAHandler, softPpm::OCBHandler
#endif
>;

int main()
{
    isrRegistrar::init();

    systemTimer::init();
//    terminal::init<0>();
    bufferedTerminal::init<0>();

    SSpi0::init();
    buttonController::init();
#ifdef LMADC
    adcController::init();
#endif
//    rpm::init();
    
    using namespace std::literals::quantity;
#ifdef PPMOUT
    softPpm::init();
    softPpm::ppm(50_ppc, 0);
#endif
#ifdef SPWM
    softPwm::init();
#endif
    
#ifdef HPWM
    hardPwm::init();
    hardPwm::pwm<hardPwm::A>(90_ppc);
    hardPwm::pwm<hardPwm::B>(50_ppc);
#endif
    
//    testPin::dir<AVR::Output>();

#ifdef I2C
    ds1307::init();
    ds1307::squareWave<true>();

    TwiMaster::init<ds1307::fSCL>();
    
#ifdef SLAVE84
    i2cram::init<ds1307::fSCL>();
#endif
#ifdef SLAVE85
    i2cled::init<ds1307::fSCL>();
#endif
#ifdef SLAVE25
    i2crpm::init<ds1307::fSCL>();
#endif
    
    std::array<TWI::Address, 5> i2cAddresses;
    TwiMaster::findDevices(i2cAddresses);
    for(const auto& d : i2cAddresses) {
        std::cout << d << std::endl;
    }
#endif
    
#ifdef DCF
    dcfDecoder::init();
#endif
    
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    constexpr auto tsd = AVR::Util::calculate<constantRateTimer>(fCr);
    static_assert(tsd, "wrong parameter");
    constantRateTimer::prescale<tsd.prescaler>();
    constantRateTimer::ocra<tsd.ocr>();

    std::cout << "pre: "_pgm << tsd.prescaler << std::endl;
    std::cout << "ocr: "_pgm << tsd.ocr << std::endl;
    
#ifdef HOTT
    crAdapterHott::init();
#endif
#ifdef OW
    crAdapterOneWire::init();
    ds18b20::init();

    oneWireMaster::findDevices(dsIds);
    for(const auto& id : dsIds) {
        std::cout << id << std::endl;
    }
#endif
            
    led0::dir<AVR::Output>();
    led1::dir<AVR::Output>();

#ifdef PPMIN
    pinChangeHandlerPpm::init();
    PpmDecoder<pinChangeHandlerPpm, ppmTimerInput>::init();
#endif
    
    std::cout << "RC Controller 0.1"_pgm << std::endl;

    WS2812<2, ws2812_A>::init();
    WS2812<2, ws2812_A>::off();

//    WS2812<2, ws2812_B>::init();
//    WS2812<2, ws2812_B>::off();

    pTimer = systemTimer::create(1000_ms, AlarmFlags::Periodic);
    tTimer = systemTimer::create(5000_ms, AlarmFlags::Periodic);
    mTimer = systemTimer::create(750_ms, AlarmFlags::OneShot);
    systemTimer::stop(*mTimer);
    
    std::cout << "---" << std::endl;

    using handler = EventHandlerGroup<TimerHandler,
                                NullPAHandler,
                                UsartFeHandler, UsartUpeHandler, UsartDorHandler,
                                HottBinaryHandler, HottBroadcastHandler, HottTextHandler, TestHandler,
                                PpmDownHandler, PpmUpHandler,
                                UsartHandler, HottKeyHandler,
                                Button0Handler, 
                                DCFReceive0Handler, DCFReceive1Handler, DCFSyncHandler, DCFErrorHandler, DCFParityHandler
#ifdef OW
                                ,ds18b20, DS18B20ErrorHandler, DS18B20MeasurementHandler
#endif
#ifdef I2C
                                ,TWIHandlerError, 
                                ds1307, DS1307handler, DS1307handlerError
#ifdef SLAVE84
    , i2cram
#endif
#ifdef SLAVE85
    , i2cled
#endif
#ifdef SLAVE25
    , i2crpm
#endif
                              , I2CRamHandler, I2CRamErrorHandler,
                                I2CLedHandler, I2CLedErrorHandler,
                                I2CRpmHandler, I2CRpmErrorHandler
#endif
    >;

    static_assert(!handler::uniqueEvents);
    
#ifdef HOTT
    sensorUsart::init<19200>();
    rcUsart::init<115200>();
#endif
    
    {
//        Scoped<EnableInterrupt> interruptEnabler;
        
        EventManager::run<sampler, handler>([](){
    //        led0::toggle();
    #ifdef PPMIN
            ppmSwitch::process(ppm1::value<0>());
    #endif
    #ifdef SPWM
            softPwm::freeRun();
    #endif
    #ifdef HOTT
            crAdapterHott::periodic();
    #endif
            vrAdapter::periodic();
    #ifdef OW
            crAdapterOneWire::periodic();
    #endif
    #ifdef I2C
            TwiMasterAsync::periodic();
    #endif
        });
    }

    return 0;
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::cout << "Assertion failed: "_pgm << expr << ',' << file << ',' << line << std::endl;
    while(true) {}
}
#endif

ISR(PCINT0_vect) {
//    isrRegistrar::isr<AVR::ISR::PcInt<0>>();
}
ISR(PCINT1_vect) {
//    isrRegistrar::isr<AVR::ISR::PcInt<1>>();
}
ISR(PCINT2_vect) {
#ifdef PPMIN
    led1::toggle();
    isrRegistrar::isr<AVR::ISR::PcInt<2>>();
#endif
}
ISR(PCINT3_vect) {
//    isrRegistrar::isr<AVR::ISR::PcInt<3>>();
}
ISR(TIMER1_COMPA_vect) {
//    led0::toggle();
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}
ISR(TIMER1_COMPB_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareB>();
}
ISR(TIMER1_CAPT_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<1>::Capture>();
}
ISR(SPI_STC_vect) {
//    isrRegistrar::isr<AVR::ISR::Spi<0>::Stc>();
}
ISR(TIMER0_COMPA_vect) {
    led0::toggle();
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(TIMER3_COMPA_vect) {
#ifdef PPMOUT
    isrRegistrar::isr<AVR::ISR::Timer<3>::CompareA>();
#endif
}
ISR(TIMER3_COMPB_vect) {
#ifdef PPMOUT
    isrRegistrar::isr<AVR::ISR::Timer<3>::CompareB>();
#endif
}

ISR(USART0_RX_vect) {
#ifdef HOTT
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
#endif
}
ISR(USART0_UDRE_vect){
#ifdef HOTT
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
#endif
}
ISR(USART1_RX_vect) {
#ifdef HOTT
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
#endif
}
ISR(USART1_UDRE_vect){
#ifdef HOTT
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
#endif
}
ISR(ANALOG_COMP_vect) {
//    led1::toggle();
//    isrRegistrar::isr<AVR::ISR::AdComparator<0>::Edge>();
}

