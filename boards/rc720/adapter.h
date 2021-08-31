#pragma once

template<typename PPM, uint8_t N>
struct PPMAdapter {
    using ranged_type = PPM::ranged_type;
    using index_t = PPM::index_type;
    inline static void set(const ranged_type& v) {
        PPM::ppmRaw(index_t{N}, v.toInt());
    }
};

template<typename PPM>
struct PPMAsyncAdapter {
    using ranged_type = PPM::ranged_type;
    using index_t = PPM::index_type;
    inline static void set(const ranged_type& v) {
        PPM::ppmRaw(v.toInt());
    }
};

template<typename ADC, uint8_t N>
struct ADCAdapter {
    using value_type = ADC::value_type; 
    using index_type = ADC::index_type; 
    inline static value_type value() {
        return ADC::value(index_type{N});
    }
};
