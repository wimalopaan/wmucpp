#pragma once

#include <cstdint>
#include <cstddef>

#include <std/chrono>
#include <etl/rational.h>
#include <etl/concepts.h>
#include <etl/algorithm.h>

#include "mcu/internals/groups.h"

namespace External {
    namespace IFX007 {
        using namespace etl;
        using namespace std;
        using namespace External::Units;
          
        template<typename PWM, typename InputType>
        struct HBridge {
            using pwm = PWM;
            using value_type = typename PWM::value_type;
            using input_type = InputType;
            
            inline static constexpr hertz fPwmMin{400};
            inline static constexpr hertz fPwmMax{16000};
            
            inline static constexpr value_type bottomCount = Project::Config::fMcu / fPwmMax;
            inline static constexpr value_type topCount = Project::Config::fMcu / fPwmMin;

            inline static constexpr value_type medium = (input_type::Upper + input_type::Lower) / 2;
            inline static constexpr value_type span = (input_type::Upper - input_type::Lower) / 2;
            
            static_assert(bottomCount >= 512);
            static_assert(bottomCount <= topCount);
            
            static inline void init() {
            }

            static inline void duty(const input_type& d) {
                if (!d) return;
                
                value_type v = (d.toInt() >= medium) ? (d.toInt() - medium) : (medium - d.toInt());
                
                value_type sp = etl::scale(v, etl::Intervall<value_type>{startRamp, endRamp}, etl::Intervall<value_type>{bottomCount, topCount});
                value_type t = (etl::enclosing_t<value_type>(v) * sp) / span;
                
                if (d.toInt() >= medium) {
                    pwm::reverse(false);
                }
                else {
                    pwm::reverse(true);
                }
                pwm::duty(t, sp);
                
                x1 = sp;
                x2 = t;
            }
            
            static inline void periodic() {
            }
            
            inline static value_type x1;
            inline static value_type x2;
            
        private:
            inline static value_type startRamp = 2000;
            inline static value_type endRamp = span;

        };
    }
}
