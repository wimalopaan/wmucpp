#pragma once

#include <mcu/avr.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/port.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/adcomparator.h>
#include <mcu/internals/capture.h>
#include <mcu/internals/pwm.h>

#include <external/units/physical.h>
#include <external/units/percent.h>

#include <etl/fixedpoint.h>
#include <etl/output.h>

#include <std/chrono>

#include <math.h>

namespace BLDC {
    using namespace AVR;
    using namespace External::Units;
    using namespace External::Units::literals;
    using namespace std::literals::chrono;
    
    namespace Sine {

        template<typename RotTimer, typename ComTimer, typename PWM>
        struct Controller {

            inline static constexpr auto sine_table = [](){
                std::array<uint16_t, 512> t;
                for(uint16_t i = 0; i < t.size(); ++i){
                    t[i] = 500 * (cos((i * 2 * M_PI) / t.size()) + 1.0);
                }
                return t;
            }();
            
//            std::integral_constant<uint16_t, sine_table[128]>::_;
            
            
            inline static void init() {
                RotTimer::init();
                ComTimer::init();
                mActualPwm = 0;
                
            }

            
            inline static void ratePeriodic() {
                constexpr uint16_t shift = sine_table.size() / 3;
                static uint16_t index = 0;
                
                uint16_t v0 = sine_table[index];
                uint16_t v1 = sine_table[(index + shift) % sine_table.size()];
                uint16_t v2 = sine_table[(index + 2 * shift) % sine_table.size()];

                constexpr uint16_t scale = 20;
                
                v0 /= scale;
                v1 /= scale;
                v2 /= scale;
                
                PWM::template duty<AVR::PWM::WO<0>>(v0);
                PWM::template duty<AVR::PWM::WO<1>>(v1);
                PWM::template duty<AVR::PWM::WO<2>>(v2);
                
                index = (index + 1) % sine_table.size();
            }
            
            inline static void periodic() {
                
            }

            inline static void off() {
                
            }
            inline static void start() {
                
            }
            inline static void pwmInc() {
                
            }
            inline static void pwmDec() {
                
            }
            
            inline static uint16_t mActualPwm;
            
        };
    }
}
