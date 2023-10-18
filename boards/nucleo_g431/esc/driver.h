#pragma once

#include <cmath>
#include <numbers>

#include "ranged.h"

namespace Sinus {
    template<typename PWM, typename StepsPerRotation = std::integral_constant<size_t, 64>>
    struct Driver {
        using pwm = PWM;
        static inline constexpr uint16_t steps = StepsPerRotation::value; 
        
        using index_type = etl::ranged<0, steps - 1>;
        using angle_type = etl::ranged_circular<0, steps - 1>;
        
        template<size_t Steps, size_t Shift>    
        struct Generator {
            static inline constexpr auto operator()() {
                std::array<float, Steps> data;
                for(uint16_t i = 0; i < Steps; ++i) {
                    data[i] = (1.0 + sin(((i - Shift) * 2.0 * std::numbers::pi) / Steps)) / 2.0;
                }
                return data;
            }
        };
        
        static inline constexpr auto sine0 = Generator<steps, 0>{}();
        static inline constexpr auto sine1 = Generator<steps, (steps / 3)>{}();
        static inline constexpr auto sine2 = Generator<steps, ((2 * steps) / 3)>{}();
        
        static inline void init() {
            pwm::init();
        }
        static inline void periodic() {
        }
        static inline void ratePeriodic() {
            ++mIndex;
            setDuty();
        }
        static inline void scale(const float s) {
            mScale = s;
        }
//        static inline void hall(const uint16_t s) {
//            const uint16_t s2 = (s + 0) % 6;
//            const uint16_t index = (steps * s2) / 6;
//            PWM::duty(mScale * sine0[index], mScale * sine1[index], mScale * sine2[index]);
//        }
    private:    
        inline static void setDuty() {
            PWM::duty(mScale * sine0[mIndex], mScale * sine1[mIndex], mScale * sine2[mIndex]);
        }
        static inline float mScale{0.3};
        inline static angle_type mIndex{};
    };
}
