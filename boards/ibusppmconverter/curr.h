#pragma once

template<typename ADC, uint8_t Channel>
struct ACS723U40Provider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
#ifdef FS_I6S
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE; // FS-I6S zeigt keinen Strom an
#else
    inline static constexpr auto ibus_type = IBus::Type::type::BAT_CURR;
#endif
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
#ifdef FS_I6S
        return currentConverter::convert(ADC::value(channel)).value / 10 + 400;
#else
        auto raw = ADC::value(channel).toInt();
        raw *= 34;
        raw >>= 3;
        if (raw > 500) {
            return raw - 500;
        }
        return 0;
#endif
    }
};

