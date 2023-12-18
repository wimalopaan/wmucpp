#pragma once

#include <cstring>

#include "etl/fixedvector.h"
#include "etl/stackstring.h"

#include "mcu/mcu.h"
#include "pwm.h"
#include "rc/rc.h"
#include "usart.h"
#include "si5351.h"
#include "i2c.h"
#include "timer.h"
#include "opamp.h"
#include "adc.h"
#include "dac.h"
#include "clock.h"
#include "units.h"
#include "output.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "gpio.h"
#include "tick.h"
#include "meta.h"
 
#include "rc/crsf.h"
#include "rc/sport.h"

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

    struct CrsfCallback;
    
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfCallback, trace, MCU>;
    using crsf    = Mcu::Stm::Uart<1, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;
    using crsftx = Mcu::Stm::Pin<gpiob, 6, MCU>;
    using crsfrx = Mcu::Stm::Pin<gpiob, 7, MCU>;

    
    using sport_pa = RC::SPort::Master::ProtocolAdapter<0, systemTimer, MCU>;
    using sport_uart = Mcu::Stm::Uart<2, sport_pa, 16, std::byte, clock, MCU>;
    using sport = RC::SPort::Master::Fsm<sport_uart, systemTimer, MCU>;
    using sporttx = Mcu::Stm::Pin<gpioa, 2, MCU>;
    using sportrx = Mcu::Stm::Pin<gpioa, 3, MCU>;
    
    template<typename Out>
    struct Telemetry {
        static inline constexpr External::Tick<systemTimer> telemTicks{1ms};

        static inline void ratePeriodic() {
            (++mTelemTick).on(telemTicks, []{
                
            });
        }
        static inline External::Tick<systemTimer> mTelemTick;
    };
    using telemetry = Telemetry<crsf_out>;
    
    struct CrsfCallback {
        using out = crsf_out;
        using Param_t = RC::Protokoll::Crsf::Parameter;
        using PType = RC::Protokoll::Crsf::Parameter::Type;

        static inline void setParameter(const uint8_t index, const uint8_t value) {
            IO::outl<trace>("SetP adr: ", index, " v: ", value);
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
        static inline bool isCommand(const uint8_t index) {
            if ((index >= 1) && (index <= params.size())) {
                return params[index - 1].mType == 0x0d;
            }
            return false;
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
            Param_t{0, PType::Sel, "P00", "Off;On", 0, 0, 1},
            Param_t{0, PType::Folder, "Serial"}, // 2
            Param_t{2, PType::Sel, "Protocol", "Off;S.Port;SBus;IBus-Sens;IBus-Servo", 0, 0, 4},
            Param_t{2, PType::Sel, "Baudrate", "100k;115k;420k", 0, 0, 2},
            Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]},
            Param_t{0, PType::Folder, "Channels"}, // 6
            Param_t{6, PType::U8, "Output 1", nullptr, 1, 1, 16},
            Param_t{6, PType::U8, "Output 2", nullptr, 2, 1, 16},
            Param_t{6, PType::U8, "Output 3", nullptr, 3, 1, 16},
            Param_t{6, PType::U8, "Output 4", nullptr, 4, 1, 16},
            Param_t{0, PType::Folder, "Modes"},
            Param_t{11, PType::Sel, "Output 1", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
            Param_t{11, PType::Sel, "Output 2", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
            Param_t{11, PType::Sel, "Output 3", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
            Param_t{11, PType::Sel, "Output 4", "PWM;M-Graupner;M-Robbe;M-Cp", 0, 0, 3},
            Param_t{0, PType::Command, "Reset", "Resetting...", 0}, // Command, timeout: 200 = 2s (value 0xc8)
            Param_t{0, PType::Sel, "Blutooth", "Off;On", 0, 0, 1},
        };
    };
    
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
        crsftx::afunction(7);
        crsfrx::afunction(7);
        crsfrx::pullup<true>();

        sport_uart::init();
        sport_uart::baud(57600);
        sport_uart::invert(true);
        sporttx::afunction(7);
        sportrx::afunction(7);
        sportrx::pulldown<true>();

        led::template dir<Mcu::Output>();
       
        pwm1::init();
    }
};

