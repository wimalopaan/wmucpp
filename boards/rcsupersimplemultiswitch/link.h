#pragma once

namespace Crsf {
    template<typename Config>
    struct Link {
        using callback = Config::callback;

        static inline bool decode(const std::byte type, const auto& /*data*/) {
            if (type != Crsf::Type::Link) {
                return false;
            }
            ++mPackagesCounter;
            callback::gotLink();
            return true;
        }
        private:
        static inline uint16_t mPackagesCounter{};
    };
}
