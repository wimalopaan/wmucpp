#include <cstddef>
#include <array>
#include <algorithm>
#include <etl/types.h>
#include <etl/algorithm.h>

#if 0
volatile uint8_t firstbyte;
volatile uint8_t secondbyte;
volatile uint8_t result;

int main()
{
  unsigned char bar=0;
  
  if (firstbyte & 0x01) bar += 0x1;
  if (firstbyte & 0x02) bar += 0x4;
  if (firstbyte & 0x04) bar += 0x2;
  if (firstbyte & 0x08) bar += 0x8;
  result = bar*100;

  bar = 0;
  if (secondbyte>>4 & 0x01) bar += 0x1;
  if (secondbyte>>4 & 0x02) bar += 0x4;
  if (secondbyte>>4 & 0x04) bar += 0x2;
  if (secondbyte>>4 & 0x08) bar += 0x8;
  result += bar*10;

  bar = 0;
  if (secondbyte&0xf & 0x01) bar += 0x1;
  if (secondbyte&0xf & 0x02) bar += 0x4;
  if (secondbyte&0xf & 0x04) bar += 0x2;
  if (secondbyte&0xf & 0x08) bar += 0x8;
  result += bar;  
  
  while(1);
}
#endif

#if 1

template<uint8_t Bit1, uint8_t Bit2, typename T>
inline constexpr T swapBits(T v) {
    static_assert(Bit1 != Bit2);
    static_assert(Bit1 < sizeof(T) * 8);
    static_assert(Bit2 < sizeof(T) * 8);
    
    constexpr T mask1{(1 << Bit1)};
    constexpr T mask2{(1 << Bit2)};
    constexpr T mask{mask1 | mask2};
    
    T v1 = ((v & mask1) >> Bit1) << Bit2;
    T v2 = ((v & mask2) >> Bit2) << Bit1;
    T v3 = v & ~mask;
    return v3 | v2 | v1;
}

template<auto B1, auto B2>
struct StaticPair {
    inline static constexpr auto first = B1;
    inline static constexpr auto second = B2;
};

template<uint8_t Size, typename BTS>
auto gen = []{
    using lut_type = std::array<std::byte, Size>;
    using size_type = lut_type::size_type;
    lut_type l;
    for(size_type i = 0; i < l.size(); ++i) {
        l[i] = swapBits<BTS::first, BTS::second>(std::byte{i});
    }
    return l;
};

template<auto B1, auto B2>
using BitsToSwap = StaticPair<B1, B2>;

namespace  {
    constinit auto lut = gen<16, BitsToSwap<1, 2>>();
}

volatile std::byte firstByte;
volatile std::byte secondByte;

volatile uint16_t v;

int main() {
//        auto c1 = swapBits<1,2>(firstByte);
        
    auto c1 = lut[uint8_t(firstByte & 0x0f_B)];
    auto c2 = lut[uint8_t(secondByte >> 4)];
    auto c3 = lut[uint8_t(secondByte & 0x0f_B)];

    v = uint16_t(c3) * 100 + uint8_t(c2) * 10 + uint8_t(c1);
    
    while(true);
}
#endif

#if 0
template<uint8_t Bit1, uint8_t Bit2, typename T>
inline constexpr T swapBits(T v) {
    static_assert(Bit1 != Bit2);
    static_assert(Bit1 < sizeof(T) * 8);
    static_assert(Bit2 < sizeof(T) * 8);
    
    constexpr T mask1{(1 << Bit1)};
    constexpr T mask2{(1 << Bit2)};
    constexpr T mask{mask1 | mask2};
    
    T v1 = ((v & mask1) >> Bit1) << Bit2;
    T v2 = ((v & mask2) >> Bit2) << Bit1;
    T v3 = v & ~mask;
    return v3 | v2 | v1;
}

struct Lower;
struct Upper;

template<typename Nibble>
inline constexpr etl::uint_NaN<uint8_t> bcdNibbleToInt(std::byte b) {
    if constexpr(std::is_same_v<Nibble, Lower>) {
    }
    if constexpr(std::is_same_v<Nibble, Upper>) {
    }
    return {};
}

auto gen100 = []{
    using value_type = etl::uint_ranged_NaN<uint16_t, 0, 999>;
    using lut_type = std::array<value_type, 16>;
    using size_type = lut_type::size_type;
    lut_type l;
    for(size_type i = 0; i < l.size(); ++i) {
        auto v1 = bcdNibbleToInt<Lower>(swapBits<1, 2>(std::byte(i)));
        if (v1) {
            l[i] = v1.toInt() * 100 ;
        }
    }
    return l;
};
auto gen = []{
    using value_type = etl::uint_ranged_NaN<uint8_t, 0, 99>;
    using lut_type = std::array<value_type, 16 * 16>;
    using size_type = lut_type::size_type;
    lut_type l;
    for(size_type i = 0; i < l.size(); ++i) {
        auto v1 = bcdNibbleToInt<Upper>(swapBits<1, 2>(std::byte(i)));
        auto v2 = bcdNibbleToInt<Lower>(swapBits<1, 2>(std::byte(i)));
        if (v1 && v2) {
            l[i] = v1.toInt() * 10 + v2.toInt();
        }
    }
    return l;
};

namespace  {
    constinit auto lut100 = gen100();
    constinit auto lut = gen();
}

volatile std::byte firstByte;
volatile std::byte secondByte;

volatile uint16_t result;

int main() {
    result = lut100[uint8_t(firstByte) >> 4] + lut[uint8_t(secondByte)];
    
    while(true);
}
#endif

#if 0
template<uint8_t Bit1, uint8_t Bit2, typename T>
inline constexpr T swapBits(T v) {
    static_assert(Bit1 != Bit2);
    static_assert(Bit1 < sizeof(T) * 8);
    static_assert(Bit2 < sizeof(T) * 8);
    
    constexpr T mask1{(1 << Bit1)};
    constexpr T mask2{(1 << Bit2)};
    constexpr T mask{mask1 | mask2};
    
    T v1 = ((v & mask1) >> Bit1) << Bit2;
    T v2 = ((v & mask2) >> Bit2) << Bit1;
    T v3 = v & ~mask;
    return v3 | v2 | v1;
}

struct Lower;
struct Upper;

template<typename Nibble>
inline constexpr etl::uint_NaN<uint8_t> bcdNibbleToInt(std::byte b) {
    if constexpr(std::is_same_v<Nibble, Lower>) {
    }
    if constexpr(std::is_same_v<Nibble, Upper>) {
    }
    return {};
}

auto gen = []{
    using value_type = etl::uint_ranged_NaN<uint16_t, 0, 999>;
    using lut_type = std::array<value_type, 16 * 16 * 16>;
    using size_type = lut_type::size_type;
    lut_type l;
    for(size_type i = 0; i < l.size(); ++i) {
        auto v1 = bcdNibbleToInt<Lower>(swapBits<1, 2>(std::byte(i)));
        auto v2 = bcdNibbleToInt<Upper>(swapBits<1, 2>(std::byte(i)));
        auto v3 = bcdNibbleToInt<Lower>(swapBits<1, 2>(std::byte(i >> 8)));
        if (v1 && v2 && v3) {
            l[i] = v1.toInt() * 100 + v2.toInt() * 10 + v3.toInt();
        }
    }
    return l;
};

namespace  {
    constinit auto lut = gen();
}

volatile std::byte firstByte;
volatile std::byte secondByte;

volatile uint16_t result;

int main() {
    result = lut[uint8_t(firstByte) * 256 + uint8_t(secondByte)];
    
    while(true);
}
#endif

#if 0

template<uint8_t Bit, auto Value>
struct BitValue {
    static_assert(Bit < 8);
    inline static constexpr uint8_t bit = Bit;
    inline static constexpr auto value  = Value;
};

namespace {
    namespace detail {
        template<typename> struct BCD;
        
        template<typename... BV>
        struct BCD<Meta::List<BV...>> {
            inline static constexpr auto max = []{
                return etl::maximum(BV::value...);
            }();
            using value_type = etl::typeForValue_t<max>;
            
            template<uint8_t BitValue, value_type Value>
            constexpr inline static void bitToValue(uint8_t v, value_type& r) {
                if (v & BitValue) {
                    r += value_type{Value};
                }
            }
            constexpr inline static auto toValue(uint8_t v) {
                value_type result{};
                (bitToValue<(1 << BV::bit), BV::value>(v, result), ...);
                return result;
            }
        };
    }
    
    template<typename BitValuesList>
    constexpr inline auto toValue(uint8_t v) {
        return detail::BCD<BitValuesList>::toValue(v);
    }
}

volatile uint8_t firstbyte  = 0x01; // Hex Eingabe = 1, 2, 3
volatile uint8_t secondbyte = 0x45;
volatile uint16_t result;

int main(){
    using sw1List  = Meta::List<BitValue<0, 100>, BitValue<1, 400>, BitValue<2, 200>, BitValue<3, 800>>;
    using sw2hList = Meta::List<BitValue<4, 10>,  BitValue<5, 40>,  BitValue<6, 20>,  BitValue<7, 80>>;
    using sw2lList = Meta::List<BitValue<0, 1>,   BitValue<1, 4>,   BitValue<2, 2>,   BitValue<3, 8> >;

    using sw2List = Meta::concat<sw2hList, sw2lList>;
    
    result = toValue<sw1List>(firstbyte) + toValue<sw2List>(secondbyte);
    
    while(true);
}

#endif

#if 0

namespace  {
    
    template<uint8_t Bit, auto Value>
    struct BitValue {
        static_assert(Bit < 8);
        inline static constexpr uint8_t bit = Bit;
        inline static constexpr auto value  = Value;
    };
    
    namespace detail {
        template<typename> struct BCD;
        
        template<typename... BV>
        struct BCD<Meta::List<BV...>> {
            inline static constexpr auto max = []{
                return etl::maximum(BV::value...);
            }();
            using value_type = etl::typeForValue_t<max>;
            
            template<uint8_t BitValue, value_type Value, typename T>
            constexpr inline static void bitToValue(uint8_t v, T& r) {
                if (v & BitValue) {
                    r += value_type{Value};
                }
            }
            template<typename T>
            constexpr inline static void toValue(uint8_t v, T& r) {
                (bitToValue<(1 << BV::bit), BV::value>(v, r), ...);
            }
        };
    }
    
    template<typename BitValuesList, typename T>
    constexpr static inline auto toValue(uint8_t v, T& r) {
        return detail::BCD<BitValuesList>::toValue(v, r);
    }
}

volatile uint8_t firstbyte  = 0x01; // Hex Eingabe = 1, 2, 3
volatile uint8_t secondbyte = 0x45;
volatile uint16_t result;

int main(){
    using sw1List  = Meta::List<BitValue<0, 100>, BitValue<1, 400>, BitValue<2, 200>, BitValue<3, 800>>;
    using sw2hList = Meta::List<BitValue<4, 10>,  BitValue<5, 40>,  BitValue<6, 20>,  BitValue<7, 80>>;
    using sw2lList = Meta::List<BitValue<0, 1>,   BitValue<1, 4>,   BitValue<2, 2>,   BitValue<3, 8> >;
    using sw2List = Meta::concat<sw2hList, sw2lList>;

    uint16_t r = 0;    
    toValue<sw1List>(firstbyte, r);
    toValue<sw2List>(secondbyte, r);
    result = r;
    while(true);
}

#endif

#if 0
volatile uint8_t firstbyte  = 0x01; // Hex Eingabe = 1, 2, 3
volatile uint8_t secondbyte = 0x45;
volatile uint16_t result;

int main()
{
  uint16_t r = 0;
  
  uint8_t f = firstbyte;
  uint8_t s = secondbyte;
  if (f & 0x01)      r += 100;
  if (f & 0x02)      r += 400;
  if (f & 0x04)      r += 200;
  if (f & 0x08)      r += 800;
  if (s & 0x01) r += 1;
  if (s & 0x02) r += 4;
  if (s & 0x04) r += 2;
  if (s & 0x08) r += 8;
  if (s & 0x10)  r += 10;
  if (s & 0x20)  r += 40;
  if (s & 0x40)  r += 20;
  if (s & 0x80)  r += 80;
  result = r;
  
  while(1);
}
#endif


#if 0
volatile uint8_t firstbyte  = 0x01; // Hex Eingabe = 1, 2, 3
volatile uint8_t secondbyte = 0x45;
volatile uint16_t result;

namespace  {
    uint16_t bcd3(uint16_t in) {
      uint8_t outl = 0;
      if (in & 0x0001) outl += 100;
      if (in & 0x1000) outl += 10;
      if (in & 0x2000) outl += 40;
      if (in & 0x4000) outl += 20;
      if (in & 0x8000) outl += 80;
      if (in & 0x0100) outl += 1;
      if (in & 0x0200) outl += 4;
      if (in & 0x0400) outl += 2;
      if (in & 0x0800) outl += 8;
      uint16_t out = outl;
      if (in & 0x0002) out += 400;
      if (in & 0x0004) out += 200;
      if (in & 0x0008) out += 800;
      return out;
    }
}

int main() {
    result = bcd3(firstbyte << 8 | secondbyte);

    while(true);
}
#endif
