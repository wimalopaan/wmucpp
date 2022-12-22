#include <etl/meta.h>
#include <etl/type_traits.h>
#include <etl/ranged.h>
#include <mcu/pgm/pgmarray.h>
#include <cmath>
#include <compare>

namespace Bcd {
    template<typename B, typename T> 
    struct WR {
        using type = B;
        const T v;  
    };
    
    template<typename B, typename T>
    WR<B, T> operator+(const WR<B, T>& lhs, const WR<B, T>& rhs) {
        return {B::add(lhs.v, rhs.v)};        
    }

    template<uint8_t Digits>
    struct BCD {
        inline static constexpr uint64_t maxValue = []{
            uint64_t v{1};
            for(uint8_t d{0}; d < Digits; ++d) {
                v *= 10;
            }
            return v - 1;
        }();
        using binary_t = etl::typeForValue_t<maxValue>;
        using safe_t = etl::uint_ranged<binary_t, 0, maxValue>;
        using bcd_t = etl::typeForBits_t<Digits * 4>;
        
        template<uint8_t N>
        struct Generator {
            static inline constexpr etl::enclosingType_t<bcd_t> base = etl::enclosingType_t<bcd_t>{1} << (N * 8);
            constexpr auto operator()() {
                std::array<bcd_t, 256> lut;
                for(binary_t i{0}; bcd_t& e : lut) {
                    e = convert(i * base);
                    ++i;
                }
                return lut;
            }
        private:
            static inline constexpr bcd_t convert(binary_t bv) {
                bcd_t result{0};
                for(binary_t f{1}; bv > 0; f *= 10) {
                    binary_t digit = bv % 10;
                    result += digit * f;
                    bv /= 10;
                }
                return result;
            }   
        };
        
        template<uint8_t N>
        struct RGenerator {
            static inline constexpr binary_t f = pow(10, 2 * N);
            constexpr auto operator()() {
                std::array<binary_t, 0x99 + 1> lut;
                for(uint8_t i{0}; binary_t& e : lut) {
                    if (isBcd(i)) {
                        e = (i >> 4) * 10 + (i & 0x0f);
                        e *= f;
                    }
                    ++i;
                }
                return lut;
            }
        private:
            constexpr bool isBcd(const uint8_t v) {
                return ((v & 0x0f) <= 9) && (((v >> 4) & 0x0f) <= 9);
            }
        };
        
        static inline constexpr uint8_t nTables = (Digits + 1) / 2;
        using nl = std::make_index_sequence<nTables>;
    
        template<typename IS> struct Tables;
        template<size_t... II>
        struct Tables<std::index_sequence<II...>> {
            using type = Meta::List<typename AVR::Pgm::Util::Converter<Generator<II>>::pgm_type...>;        
        };
        using tables = Tables<nl>::type;    
        
        template<typename IS> struct RTables;
        template<size_t... II>
        struct RTables<std::index_sequence<II...>> {
            using type = Meta::List<typename AVR::Pgm::Util::Converter<RGenerator<II>>::pgm_type...>;        
        };
        using rtables = RTables<nl>::type;    
    
        template<typename IS> struct BinRF;
        template<size_t... II>
        struct BinRF<std::index_sequence<II...>> {
            static inline constexpr binary_t value(const bcd_t v) {
                return (Meta::nth_element<II, rtables>::value((v >> (II * 8)) & 0xff) + ...);
            }        
        };
        
        using binrf = BinRF<std::make_index_sequence<nTables>>;
        
        constexpr binary_t toBinary() const {
            return binrf::value(mValue);
        }
    
        template<typename IS> struct BinF;
        template<size_t... II>
        struct BinF<std::index_sequence<II...>> {
            static inline constexpr bcd_t value(const binary_t v) {
                using wr = WR<BCD, bcd_t>;
                return (wr{Meta::nth_element<II, tables>::value((v >> (II * 8)) & 0xff)} + ...).v;
            }        
        };
        using binf = BinF<std::make_index_sequence<nTables>>;
        
        static constexpr BCD fromBinary(const safe_t bv) {
            return binf::value(bv);        
        }
    
        static constexpr BCD fromRaw(const bcd_t bcd) {
            return BCD{bcd};
        }   
        constexpr bcd_t value() const {
            return mValue;
        }
        template<typename T>
        static constexpr T median (const T& x, const T& y, const T& z) {
             return (x & (y | z)) | (y & z);
        }
        
        static constexpr bcd_t bcd_add_knuth (const bcd_t x, const bcd_t y) {
            static constexpr uint8_t R = Digits;
            constexpr auto c1 = []{
                bcd_t value{};
                for(uint8_t i{0}; i < (R - 1); ++i) {
                    value += (0x06 << (4 * i));
                }
                return value;
            }();
            constexpr auto c2 = []{
                bcd_t value{};
                for(uint8_t i{1}; i < R; ++i) {
                    value += (0x88 << (4 * i));
                }
                return value;
            }();
            const bcd_t z = y + c1;
            const bcd_t u = x + z;
            const bcd_t t = median(~x, ~z, u) & c2;
            return u - t + (t >> 2);
        }
        
        static constexpr bcd_t add(const bcd_t a, const bcd_t b) {
            return bcd_add_knuth(a, b);
            
    //        static constexpr uint8_t R = Digits;
    //        constexpr auto c1 = []{
    //            bcd_t value{};
    //            for(uint8_t i{0}; i < (R - 1); ++i) {
    //                value += (0x06 << (4 * i));
    //            }
    //            return value;
    //        }();
    //        constexpr auto c2 = []{
    //            bcd_t value{};
    //            for(uint8_t i{1}; i < R; ++i) {
    //                value += (0x01 << (4 * i));
    //            }
    //            return value;
    //        }();
            
    //        bcd_t sum = a + b;    
    //        const bcd_t carry = ((sum + c1) ^ a ^ b) & c2;
    //        sum += ((carry - (carry >> 4)) & c1);
            
    //        return sum;
        }
    private:
        constexpr BCD(const bcd_t bcd) : mValue{bcd} {}
        bcd_t mValue;
    };
    
    template<uint8_t N, uint8_t M>
    constexpr auto operator+(const BCD<N>& lhs, const BCD<M>& rhs) {
        constexpr uint8_t R = std::max(N, M) + 1;
        using result_t = BCD<R>;
        return result_t::fromRaw(result_t::add(lhs.value(), rhs.value()));    
    }     
}

int main() {
    using namespace Bcd;
    
    auto b1 = BCD<2>::fromBinary(BCD<2>::safe_t{99});
//    auto b1 = BCD<2>::fromRaw(0x11);
    
//    return Meta::nth_element<0, BCD<2>::tables>::value(0);
    
//    return b1.value();
    
//    auto b2 = BCD<2>::fromBinary(BCD<2>::safe_t{18});
//    auto b2 = BCD<2>::fromRaw(0x18);
    
//    auto b3 = b1 + b2;
//////    decltype(b3)::_;
    
//////    return BCD<8>::rlut67::value(1);
    
    return b1.toBinary();
//    return b3.value();
}
