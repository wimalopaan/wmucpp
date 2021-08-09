#pragma once

#include <mcu/avr.h>

#include <mcu/internals/usart.h>

#include <external/sbus/sbus.h>
#include <external/ibus/ibus2.h>

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    static inline uint8_t magic = 42;
    using bus_type = External::Bus::IBusIBus<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    template<typename ADC, uint8_t Channel, typename SigRow>
    struct InternalTempProvider {
        using index_t = ADC::index_type;
        static_assert(Channel <= index_t::Upper);
        inline static constexpr auto channel = index_t{Channel};
        inline static constexpr auto ibus_type = IBus2::Type::type::TEMPERATURE;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            return SigRow::template adcValueToTemperature<std::ratio<1,10>, 40, typename ADC::VRef_type>(ADC::value(channel)).value;
        }
    };

    using adcController = devs::adcController;
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 6, typename Devs::sigrow>;
    
    using sensor = IBus2::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<115200>, 
                                Meta::List<tempiP>, 
                                systemTimer, typename devs::ibt>;
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    static inline uint8_t magic = 43;
    using bus_type = External::Bus::SBusSPort<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    template<typename PA>
    using sensorUsart = AVR::Usart<typename Devs::sensorPosition, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
    
    template<typename ADC, uint8_t Channel, typename SigRow>
    struct InternalTempProvider {
        using index_t = ADC::index_type;
        static_assert(Channel <= index_t::Upper);
        inline static constexpr auto channel = index_t{Channel};
        inline static constexpr auto valueId = External::SPort::ValueId::Temp2;
        inline static constexpr void init() {}
        inline static constexpr uint32_t value() {
            return SigRow::template adcValueToTemperature<std::ratio<1,1>, 0, typename ADC::VRef_type>(ADC::value(channel)).value;
        }
    };


    using adcController = devs::adcController;
    
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 6, typename Devs::sigrow>;
    
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, 
                                           Meta::List<tempiP>>;

};

template<typename Devs>
struct BusDevs<External::Bus::SumDHott<Devs>> {
    static inline uint8_t magic = 44;
    using bus_type = External::Bus::SumDHott<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
       
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif

    using configProvider = void;
    
    using sensor = Hott::Experimental::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;

    using calibratePs = Meta::List<>;

};
