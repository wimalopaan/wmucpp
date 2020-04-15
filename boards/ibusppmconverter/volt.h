#pragma once

template<typename ADC, uint8_t Channel>
struct VoltageProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
//    index_t::_;
    inline static constexpr auto ibus_type = IBus::Type::type::EXTERNAL_VOLTAGE;
    inline static constexpr void init() {}
    
    using battVoltageConverter = Hott::Units::Converter<adc, IBus::battery_voltage_t, std::ratio<11000,1000>>; // Voltage divider 10k/1k 
//    using battVoltageConverter = Hott::Units::Converter<adc, IBus::battery_voltage_t, std::ratio<121,21>>; // Voltage divider 10k/1k 
    
    inline static constexpr uint16_t value() {
        return battVoltageConverter::convert(ADC::value(channel)).value;
    }
};

using voltageP = VoltageProvider<adcController, 0>;
