#include <cstdint>
#include <cstddef>
#include <type_traits>

constexpr uint32_t julianDay(const uint16_t year, const uint8_t month, const uint8_t day) {
    const size_t a = (14 - month)/12;
    const size_t y = year+4800-a;
    const size_t m = month + 12*a - 3;
    return day + (153*m+2)/5 + uint32_t{y}*365 + y/4 - y/100 + y/400 - 32045;
}

constexpr uint32_t secondsSinceEpoch(const uint16_t year, const uint8_t month, const uint8_t day) {
    return (julianDay(year, month, day) - julianDay(1970, 1, 1)) * 24u * 3600u;
}

uint32_t utc;

int main() {
//    if (utc > (secondsSinceEpoch(2098, 12, 31) - 1)) { // 2098-12-31 23:59:59
//        utc = (secondsSinceEpoch(2098, 12, 31) - 1);
//    }
    constexpr auto v2 = secondsSinceEpoch(2099, 1, 1) - 1;
    if (utc > v2) { // 2098-12-31 23:59:59
        utc = v2;
    }

//        std::integral_constant<uint32_t, v2>::_;
//    return v2;
    
}
