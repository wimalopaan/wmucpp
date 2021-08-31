#pragma once

#include <external/sbus/sbus.h>
#include <external/ibus/ibus2.h>

namespace External {
    template<typename ADC, uint8_t Channel, typename SigRow, typename BusType>
    struct InternalTempProvider;
    
    template<typename ADC, uint8_t Channel, typename SigRow, typename Devs>
    struct InternalTempProvider<ADC, Channel, SigRow, External::Bus::IBusIBus<Devs>> {
        using index_t = ADC::index_type;
        static_assert(Channel <= index_t::Upper);
        inline static constexpr auto channel = index_t{Channel};
        inline static constexpr auto ibus_type = IBus2::Type::type::TEMPERATURE;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            return SigRow::template adcValueToTemperature<std::ratio<1,10>, 40, typename ADC::VRef_type>(ADC::value(channel)).value;
        }
    };


    template<typename ADC, uint8_t Channel, typename SigRow, typename Devs>
    struct InternalTempProvider<ADC, Channel, SigRow, External::Bus::SBusSPort<Devs>> {
        using index_t = ADC::index_type;
        static_assert(Channel <= index_t::Upper);
        inline static constexpr auto channel = index_t{Channel};
        inline static constexpr auto valueId = External::SPort::ValueId::Temp2;
        inline static constexpr void init() {}
        inline static constexpr uint32_t value() {
            return SigRow::template adcValueToTemperature<std::ratio<1,1>, 0, typename ADC::VRef_type>(ADC::value(channel)).value;
        }
    };
    
}
