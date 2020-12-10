#pragma once

template<typename ADC, uint8_t Channel>
struct InternalTempProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        return sigrow::adcValueToTemperature<std::ratio<1,10>, 40 - 15>(ADC::value(channel)).value;
    }
};
template<typename ADC, uint8_t Channel>
struct Mcp9700aProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        auto raw = ADC::value(channel);
        if (raw.isTop()) {
            return 0;
        }
        else {
            auto v = raw.toInt(); 
            v *= 34;
            v >>= 3;
            if (v >= 100) {
                return v - 100;
            }
            else {
                return 0;
            }
        }
    }
};

