#pragma once

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
