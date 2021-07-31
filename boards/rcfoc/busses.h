#pragma once

#include "devices.h"
#include "link.h"

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::NoBus<Devs>> {
    static inline uint8_t magic = 42;
    using devs = Devs;
    
    using systemTimer = Devs::systemTimer;
    
//    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
//    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using terminal = etl::basic_ostream<typename devs::scan_term_dev>;
    
    using adcController = devs::adcController;
    
    using sbusGen = External::SBus::Output::Generator<typename devs::sensorPosition, systemTimer>;

    using link = SimpleProtocol::Sender<2>;
    using link_pa = SimpleProtocol::Adapter<2>;
    
    using link_dev = AVR::Usart<typename devs::servoPosition, link_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<8>>;
    
};

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    static inline uint8_t magic = 43;
    using bus_type = External::Bus::IBusIBus<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using terminal = etl::basic_ostream<typename devs::scan_term_dev>;
    
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
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 3, typename Devs::sigrow>;
    
    using sensor = IBus2::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<115200>, 
    Meta::List<tempiP>, 
    systemTimer, void>;
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    static inline uint8_t magic = 44;
    using bus_type = External::Bus::SBusSPort<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = AVR::Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using terminal = etl::basic_ostream<typename devs::scan_term_dev>;
    
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
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 3, typename Devs::sigrow>;
    
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, 
    Meta::List<tempiP>>;
    
};

