#pragma once

namespace Util {
    namespace detail {
        struct Generatorx {
            constexpr auto operator()() {
                std::array<bool, 256> lut;
                for(uint8_t v{0}; auto& l : lut) {
                    l = etl::numberOfOnes(v) & 0x01;
                    ++v;
                }
                return lut;
            }
        };
        using EvenParity = AVR::Pgm::Util::Converter<Generatorx>::pgm_type;
    }    
    struct ParityGenerator {
        static constexpr inline bool isEvenParity(const std::byte v) {
            return detail::EvenParity::value(uint8_t(v));
        }
        static constexpr inline bool isEvenParity(const std::array<std::byte, 2>& v) {
            const std::byte x{v[0] ^ v[1]};
            return detail::EvenParity::value(uint8_t(x));
        }
        //        static_assert(isEvenParity(0_B));
    };
}


