#include <iostream>
#include <utility>
#include <array>
#include <cstddef>
#include <cstdint>

static constexpr double RVCC =   10.0;    // 10k to VCC
static constexpr double RROW =   1.0;     // 1k between rows
static constexpr double RCOL =   3.9;     // 3k9 between colums
static constexpr std::size_t NROW  =  4;       // number of rows
static constexpr uint32_t ADCRES = 256;     // ADC resolution
static constexpr uint8_t NKEY  =  12;

constexpr double adcval(std::uint8_t n) {
    return ADCRES*(1-RVCC/((n)%NROW*RROW+(n)/NROW*RCOL+RVCC));
}

constexpr double treshold(std::size_t n) {
    return (uint16_t)((adcval(n)+adcval(n+1))/2);
}

constexpr auto THRESHOLDS = []<auto N = 12>(auto f){
        std::array<decltype(f(0)), N> data{};
        for(size_t i = 0; i < N; ++i) {
           data[i] = f(i);
        }
        return data;
}(treshold);

int main() {
    std::cout << __cplusplus << '\n';    
    for (const auto& val : THRESHOLDS) {
        std::cout << val << ", ";
    }
    std::cout << std::endl;
    return 0;
}
