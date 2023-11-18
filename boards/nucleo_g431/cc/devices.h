#pragma once

#include <cstring>
#include "../include/fixedvector.h"
#include "../include/stackstring.h"

#include "../include/mcu.h"
#include "../include/pwm.h"
#include "../include/rc.h"
#include "../include/usart.h"
#include "../include/si5351.h"
#include "../include/i2c.h"
#include "../include/timer.h"
#include "../include/opamp.h"
#include "../include/adc.h"
#include "../include/dac.h"
#include "../include/clock.h"
#include "../include/units.h"
#include "../include/output.h"
#include "../include/concepts.h"
#include "../include/mcu_traits.h"
#include "../include/arm.h"
#include "../include/gpio.h"
#include "../include/tick.h"
#include "../include/meta.h"
 
#include "crsf.h"
#include "sport.h"

struct CC01;

template<typename HW, typename Config, typename MCU = void>
struct Devices;

using namespace Mcu::Stm;
using namespace std::literals::chrono_literals;

template<typename Config>
struct Devices<CC01, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
//    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
    using trace = Arm::Trace<clock, 2_MHz, 128>; 

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;

    struct CB;
    
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CB, trace, MCU>;
    using crsf    = Mcu::Stm::Uart<1, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;

    template<typename PA>
    using sport_usart = Mcu::Stm::Uart<2, PA, 16, std::byte, MCU>;
    
    
    template<typename Out>
    struct Telemetry {
        static inline constexpr External::Tick<systemTimer> telemTicks{1ms};

        static inline void ratePeriodic() {
            (++mTelemTick).on(telemTicks, []{
                
            });
        }
        
        static inline External::Tick<systemTimer> mTelemTick;
        
//        static inline void crsfInitSensor() {
//            CRC8 crc;
//            mCrsfBuffer[0] = 0xc8;
//            mCrsfBuffer[1] = 8 + 2; // Payload len = N + 2
//            mCrsfBuffer[2] = (uint8_t)RC::Protokoll::Crsf::Type::Battery; 
//            mCrsfBuffer[3] = 0x03; // V
//            mCrsfBuffer[4] = 0x01; 
//            mCrsfBuffer[5] = 0x05; // I
//            mCrsfBuffer[6] = 0x01; 
//            mCrsfBuffer[7] = 0x07; // C
//            mCrsfBuffer[8] = 0x01; 
//            mCrsfBuffer[9] = 0x01; 
//            mCrsfBuffer[10] = 0x08; // remaining C
            
//            for(uint8_t i = 2; i <= 10; ++i) {
//                crc += mCrsfBuffer[i];
//            }
//            mCrsfBuffer[11] = crc; // CRC
//        }
//        static inline void crsfInit() {
//            CRC8 crc;
//            mCrsfBuffer[0] = 0xc8;
//            mCrsfBuffer[1] = 4 + 2; // Payload len = 4
//            mCrsfBuffer[2] = 0x32; // Command
//            mCrsfBuffer[3] = 0xea; // Dest: Handset
//            mCrsfBuffer[4] = 0xc8; // Src: FC
//            mCrsfBuffer[5] = 0x01; // payload
//            mCrsfBuffer[6] = 0x02; // payload
            
//            for(uint8_t i = 2; i <= 6; ++i) {
//                crc += mCrsfBuffer[i];
//            }
//            mCrsfBuffer[7] = crc; // CRC
//        }
    };
    
    using telemetry = Telemetry<crsf_out>;
    
    struct CB {
        using out = crsf_out;
        using Param_t = RC::Protokoll::Crsf::Parameter;

        static inline void setParameter(const uint8_t index, const uint8_t value) {
            if ((index >= 1) && (index <= params.size())) {
                params[index - 1].mValue = value;
            }
        }
        static inline RC::Protokoll::Crsf::Parameter parameter(const uint8_t index) {
            if ((index >= 1) && (index <= params.size())) {
                return params[index - 1];
            }
            return {};
        }
        static inline const char* name() {
            return mName;
        }
        static inline uint32_t serialNumber() {
            return mSerialNumber;
        }
        static inline uint32_t hwVersion() {
            return mHWVersion;
        }
        static inline uint32_t swVersion() {
            return mSWVersion;
        }
        static inline uint8_t numberOfParameters() {
            return params.size();
        }
        static inline uint8_t protocolVersion() {
            return 0;
        }
    private:
        static inline constexpr uint32_t mSerialNumber{1234};
        static inline constexpr uint32_t mHWVersion{1};
        static inline constexpr uint32_t mSWVersion{1};
        static inline constexpr auto mVersionString = [](){
            std::array<char, 16> s{};
            auto [ptr, e] = std::to_chars(std::begin(s), std::end(s), mHWVersion);
            *ptr++ = '/';
            std::to_chars(ptr, std::end(s), mSWVersion);
            return s;
        }();
        static inline constexpr const char* const mName = "CruiseControl";
        static inline etl::FixedVector<Param_t, 64> params {
            Param_t{0, 9, "P00", "Off;On", 0, 0, 2},
            Param_t{0, 9, "Bus", "Off;S.Port;SBus;IBus-Sens;IBus-Servo", 0, 0, 4},
            Param_t{0, 0x0c, "Version(HW/SW)", &mVersionString[0]},
            Param_t{0, 0x0b, "Channels"},
            Param_t{4, 0x00, "Output 1", nullptr, 1, 1, 16},
            Param_t{4, 0x00, "Output 2", nullptr, 2, 1, 16},
            Param_t{4, 0x00, "Output 3", nullptr, 3, 1, 16},
            Param_t{4, 0x00, "Output 4", nullptr, 4, 1, 16},
            Param_t{0, 0x0b, "Modes"},
            Param_t{9, 0x09, "Output 1", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
            Param_t{9, 0x09, "Output 2", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
            Param_t{9, 0x09, "Output 3", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
            Param_t{9, 0x09, "Output 4", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
        };
    };
    
    using pintx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using pinrx = Mcu::Stm::Pin<gpiob, 7, MCU>;
    
    using led = Mcu::Stm::Pin<gpiob, 8, MCU>;

    using pinb4 = Mcu::Stm::Pin<gpiob, 4, MCU>; 
    using pinb5 = Mcu::Stm::Pin<gpiob, 5, MCU>;
    using cppm_period = std::integral_constant<uint16_t, 5120>; // duty 512 -> 2ms, duty 256 -> 1ms Pulse, Period = 20ms
    using cppm_prescaler = std::integral_constant<uint16_t, 664>;
    using timerPwm1 = Mcu::Stm::Timer<3, cppm_period, cppm_prescaler, Mcu::Stm::Trigger<false>, MCU>;
    using pwm1 = Mcu::Stm::Pwm::Servo<timerPwm1, Meta::List<pinb4, void, pinb5, void>>; // bis zu 4 Ausgänge
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpioa::init();
        gpiob::init();

        crsf::init();
        crsf::baud(420'000);
        
        pintx::afunction(7);
        pinrx::afunction(7);
        pinrx::pullup<true>();
        
        led::template dir<Mcu::Output>();
       
        pwm1::init();
    }
};

