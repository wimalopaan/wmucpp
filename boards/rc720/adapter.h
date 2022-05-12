#pragma once

template<typename PPM, uint8_t N>
struct PPMAdapter {
    inline static constexpr auto ocMax =  PPM::ocMax;
    inline static constexpr auto ocMin =  PPM::ocMin;
    inline static constexpr auto ocMedium =  PPM::ocMedium;
    using ranged_type = PPM::ranged_type;
    using index_t = PPM::index_type;
    inline static void init() {
        PPM::init();
    }
    inline static void set(const ranged_type& v) {
        PPM::ppmRaw(index_t{N}, v.toInt());
    }
    inline static void ppmRaw(const auto v) {
        PPM::ppmRaw(index_t{N}, v);
    }
    inline static void ppm_async(const auto v) {
        PPM::ppm(index_t{N}, v);
    }
    inline static void onReload(auto f) {
        PPM::template onCompareMatch<AVR::PWM::WO<N>>([&]{
            f();
        });
    }
};

template<typename PPM>
struct PPMAsyncAdapter {
    inline static constexpr auto ocMax =  PPM::ocMax;
    inline static constexpr auto ocMin =  PPM::ocMin;
    inline static constexpr auto ocMedium =  PPM::ocMedium;
    using ranged_type = PPM::ranged_type;
    using index_t = PPM::index_type;
    inline static void init() {
        PPM::init();
    }
    inline static void set(const ranged_type& v) {
        PPM::ppmRaw(v.toInt());
    }
    inline static void ppmRaw(const auto v) {
        PPM::ppmRaw(v);
    }
    inline static void ppm_async(const auto v) {
        PPM::ppm_async(v);
    }
};

template<typename ADC, uint8_t N>
struct ADCAdapter {
    static inline constexpr uint8_t number = N;
    using value_type = ADC::value_type; 
    using index_type = ADC::index_type; 
    inline static value_type value() {
        return ADC::value(index_type{N});
    }
};
