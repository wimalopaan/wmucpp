#pragma once

#include <cstdint>

namespace External {
    
    template<typename ADCont, auto Channel, typename Uoff, typename K, typename Kd, typename value_type = uint16_t>
    struct AnalogSensor {
        inline static constexpr double VRef = ADCont::VRef;
        inline static constexpr double uoff = (double)Uoff::nom / (double)Uoff::denom;
        inline static constexpr double k    = (double)K::nom / (double)K::denom;
        inline static constexpr double kd   = (double)Kd::nom / (double)Kd::denom;
        
        inline static constexpr double admax = ADCont::value_type::Upper;
        inline static constexpr double scale = (kd * VRef) / (k * admax);        
        inline static constexpr double offset= (kd * uoff) / k;
        
//        std::integral_constant<uint16_t, (uint16_t)(k * 1000)>::_;
//        std::integral_constant<uint16_t, (uint16_t)(kd)>::_;
        //        std::integral_constant<uint16_t, (uint16_t)(VRef * 1000)>::_;
//        std::integral_constant<uint16_t, (uint16_t)admax>::_;
//        std::integral_constant<uint16_t, (uint16_t)(scale * 1000)>::_;
//        std::integral_constant<uint16_t, (uint16_t)offset>::_;
        
//        ADCont::_;
        
        inline static constexpr uint8_t freeBits = etl::numberOfBits<value_type>() - ADCont::mcu_adc_type::reso_type::bits;
//        std::integral_constant<uint8_t, freeBits>::_;
        
        inline static constexpr double qmax = (double)((1 << freeBits) - 1) / scale;
//        std::integral_constant<uint16_t, (uint16_t)qmax>::_;
        
        inline static constexpr uint16_t shift = []{
            for(uint8_t i = 8; i--> 0;) {
                auto x = (1U << i);
                if (x <= qmax) {
                    return i;
                }
            }
        }();
//        std::integral_constant<uint8_t, shift>::_;
        
        inline static constexpr uint16_t a = scale * (1 << shift) + 0.5;
        inline static constexpr uint16_t b = offset * (1 << shift) + 0.5;
//        std::integral_constant<uint16_t, a>::_;
//        std::integral_constant<uint16_t, b>::_;
        
        using index_type = ADCont::index_type;
        
        inline static constexpr auto channel_n = index_type{Channel};

        template<bool on = false>
        static inline void pullup() {
            ADCont::template pullup<Channel, on>();
        }
        
        static inline value_type value() {
            uint16_t sv = a * ADCont::value(channel_n) ;
            if (sv >= b) {
                return (sv - b) >> shift;
            }
            else {
                return 0;
            }
        }
        static inline ADCont::value_type raw() {
            return ADCont::value(channel_n);
        }
    };

    template<typename Provider, typename Uoff, typename K, typename Kd, typename value_type = uint16_t>
    struct AnalogConverter {
        inline static constexpr double VRef = Provider::VRef;
        inline static constexpr double uoff = (double)Uoff::nom / (double)Uoff::denom;
        inline static constexpr double k    = (double)K::nom / (double)K::denom;
        inline static constexpr double kd   = (double)Kd::nom / (double)Kd::denom;
        
        inline static constexpr double admax = Provider::value_type::Upper;
        inline static constexpr double scale = (kd * VRef) / (k * admax);        
        inline static constexpr double offset= (kd * uoff) / k;
        
//        std::integral_constant<uint16_t, (uint16_t)admax>::_;
//        std::integral_constant<uint16_t, (uint16_t)(scale * 1000)>::_;
//        std::integral_constant<uint16_t, (uint16_t)offset>::_;
        
        inline static constexpr uint8_t freeBits = etl::numberOfBits<value_type>() - Provider::reso_type::bits;
//        std::integral_constant<uint8_t, freeBits>::_;
        
        inline static constexpr double qmax = (double)((1 << freeBits) - 1) / scale;
//        std::integral_constant<uint16_t, (uint16_t)qmax>::_;
        
        inline static constexpr uint16_t shift = []{
            for(uint8_t i = 12; i--> 0;) {
                auto x = (1U << i);
                if (x <= qmax) {
                    return i;
                }
            }
        }();
//        std::integral_constant<uint8_t, shift>::_;
        
        inline static constexpr uint16_t a = scale * (1 << shift) + 0.5;
        inline static constexpr uint16_t b = offset * (1 << shift) + 0.5;
//        std::integral_constant<uint16_t, a>::_;
//        std::integral_constant<uint16_t, b>::_;
        
        static inline value_type value() {
            uint16_t sv = a * Provider::adcValue() ;
            if (sv >= b) {
                return (sv - b) >> shift;
            }
            else {
                return 0;
            }
        }
    };


}
