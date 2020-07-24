#include <cstddef>
#include <array>
#include <algorithm>
#include <iostream>
    
using Frequency = uint64_t;

Frequency operator "" _Hz(unsigned long long int val)
{
    return val;
}

Frequency operator "" _kHz(unsigned long long int val)
{
    return val * 1000;
}

using Bla = uint64_t;

int main() {

    Frequency freq1 = 1_Hz;
    Frequency freq2 = 23_kHz;

    Bla b = 23_kHz;
    
    auto x = b * freq1;
    
//    decltype(x)::_;
}
