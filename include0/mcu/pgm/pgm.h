#pragma once

#include <cstddef>
#include <cstdint>

namespace AVR {
    namespace Pgm {
        template<typename P = std::byte>
        struct Ptr {
            inline constexpr explicit Ptr(const P* ptr) : value{ptr} {}
            
            inline constexpr const P* operator->() const {
                return value;
            }
        private:
            const P* const value;
        };
    }
}

