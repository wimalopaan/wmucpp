#pragma once

#include <cstring>

#include "etl/fixedvector.h"
#include "etl/stackstring.h"

#include "meta.h"

#include "mcu/mcu.h"
#include "mcu/mcu_traits.h"
#include "mcu/arm.h"
#include "pwm.h"
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
#include "gpio.h"
#include "tick.h"
#include "meta.h"
#include "rc/rc.h"
#include "rc/crsf.h"
#include "rc/sport.h"
#include "rc/roboremo.h"
#include "rc/escape.h"
#include "rc/sbus2.h"
#include "rc/vesc.h"
#include "bluetooth/hc05.h"

struct CC01;
struct CC02;
struct CC03;

using namespace Mcu::Stm;
using namespace std::literals::chrono_literals;

template<typename HW, typename Config, typename MCU = void>
struct Devices;

template<typename Config>
struct Devices<CC03, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
//    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
    using trace = Arm::Trace<clock, 2_MHz, 128>; 

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    struct CrsfCallback;
    
    // Usart 1
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfCallback, trace, MCU>;
    using crsf    = Mcu::Stm::Uart<1, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;
    using crsftx = Mcu::Stm::Pin<gpiob, 6, MCU>; // AF 7
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>; // AF 7
    
    // Usart 3
#ifdef USE_SPORT
    using sport_pa = RC::SPort::Master::ProtocolAdapter<0, systemTimer, MCU>;
    using sport_uart = Mcu::Stm::Uart<3, sport_pa, 16, std::byte, clock, MCU>;
    using sport = RC::SPort::Master::Fsm<sport_uart, systemTimer, MCU>;
    using sportrxtx = Mcu::Stm::Pin<gpiob, 11, MCU>; // rx, AF 7
    using sporttxinv = Mcu::Stm::Pin<gpiob, 9, MCU>; // tx, AF 7
#endif
#ifdef USE_ESC
    using esc_pa = RC::ESCape::Master::ProtocolAdapter<0, systemTimer, MCU>;
    using esc_uart = Mcu::Stm::Uart<3, esc_pa, 16, std::byte, clock, MCU>;
    using esc = RC::ESCape::Master::Fsm<esc_uart, systemTimer, MCU>;
    using escrxtx = Mcu::Stm::Pin<gpiob, 11, MCU>; // rx, AF 7
    using esctxinv = Mcu::Stm::Pin<gpiob, 9, MCU>; // tx, AF 7
#endif
#ifdef USE_VESC
    struct VescCallback;
    using vesc_pa = RC::VESC::Master::ProtocolAdapter<0, VescCallback, systemTimer, MCU>;
    using vesc_uart = Mcu::Stm::Uart<3, vesc_pa, 16, std::byte, clock, MCU>;
    using vesc = RC::VESC::Master::Fsm<vesc_uart, systemTimer, MCU>;
    using vescrxtx = Mcu::Stm::Pin<gpiob, 11, MCU>; // rx, AF 7
//    using vesctxinv = Mcu::Stm::Pin<gpiob, 9, MCU>; // tx, AF 7
#endif

    // DMA    
    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;

    // Usart 2

    // use SBUS2 proto
    struct SbusCallback;
    // using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    // using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    // using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    using sbusDmaChannel5 = Mcu::Stm::Dma::Channel<dma1, 5, MCU>;    
    using sbus = RC::Protokoll::SBus2::Master<2, sbusDmaChannel5, clock, MCU>;
    using sbustx = Mcu::Stm::Pin<gpiob, 4, MCU>; // rx, AF 7, need swap rxtx-pins
    
    // Uart 4? PC10, PC11 (nicht nutzbar)
    
    using led1 = Mcu::Stm::Pin<gpiob, 13, MCU>;
    using led2 = Mcu::Stm::Pin<gpioa, 0, MCU>;

    struct PassThru1 {
        static inline void start() {
            // sbus_uart::clear();
            // sbus_pa::reset();
            // sbus::stop();
            // sbus_uart::template halfDuplex<true>();
            // sbustx::template pullup<true>();
            // sbus_uart::baud(38'400);
            // sbus_uart::parityOff();
            // sbus_uart::invert(false);            
        }
        static inline void stop() {
            // sbus_uart::clear();
            // sbus_pa::reset();
            // sbus::start();
            // sbus_uart::template halfDuplex<true>();
            // sbustx::template pulldown<true>();
            // sbus_uart::baud(100'000);
            // sbus_uart::parity(true);
            // sbus_uart::invert(true);
        }
        static inline void put(const std::byte b) {
            //  sbus_uart::put(b);            
        }
    };
    using passthru = PassThru1;


    struct SendBack;

    // LPuart 1
    using btpwr = Mcu::Stm::Pin<gpioc, 13, MCU>;
    using bten = Mcu::Stm::Pin<gpioc, 14, MCU>;
    using robo_pa = External::QtRobo::ProtocollAdapter<0, systemTimer, passthru, SendBack>;
    using bt_uart = Mcu::Stm::LpUart<1, robo_pa, 16, std::byte, clock, MCU>;
    using btrx = Mcu::Stm::Pin<gpioa, 3, MCU>; // AF 12
    using bttx = Mcu::Stm::Pin<gpioa, 2, MCU>; // AF 12

    struct SendBack {
        static inline void put(auto b) {
            bt_uart::put(b);
        }
    };
 
    // PWM Timer: t2, t3, t4, t15
    using pwm1 = Mcu::Stm::Pin<gpioa, 1, MCU>; // tim2, ch2, af 1
    using pwm2 = Mcu::Stm::Pin<gpioa, 4, MCU>; // tim3, ch2, af 2
    using pwm3 = Mcu::Stm::Pin<gpioa, 5, MCU>; // tim2, ch1, af 1
    using pwm4 = Mcu::Stm::Pin<gpioa, 6, MCU>; // tim3, ch1, af 2
    using pwm5 = Mcu::Stm::Pin<gpiob, 0, MCU>; // tim3, ch3, af 2
    using pwm6 = Mcu::Stm::Pin<gpiob, 1, MCU>; // tim3, ch4, af 2
    using pwm7 = Mcu::Stm::Pin<gpioa, 12, MCU>; // tim4, ch2, af 10
    using pwm8 = Mcu::Stm::Pin<gpioa, 11, MCU>; // tim4, ch1, af 10
    using pwm9 = Mcu::Stm::Pin<gpioa, 9, MCU>; // tim2, ch3, af 10
    using pwm10 = Mcu::Stm::Pin<gpiob, 14, MCU>; // tim15, ch1, af 1

    using pwmDmaChannel1 = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;    
    using pwmDmaChannel2 = Mcu::Stm::Dma::Channel<dma1, 2, MCU>;    
    using pwmDmaChannel3 = Mcu::Stm::Dma::Channel<dma1, 3, MCU>;    
    // using pwmDmaChannel4 = Mcu::Stm::Dma::Channel<dma1, 4, MCU>;
    using pwmDmaChannels = Meta::List<pwmDmaChannel1, pwmDmaChannel2, pwmDmaChannel3>;    
    using pwmSequence1 = Mcu::Stm::Pwm::Sequence<2, 10, pwmDmaChannels, clock, MCU>; // timer 2, sequence-length 
    using pwmSequences1 = pwmSequence1::pwms;
    using pwmSequence1_0 = Meta::nth_element<0, pwmSequences1>; // pwm 3
    using pwmSequence1_1 = Meta::nth_element<1, pwmSequences1>; // pwm 1
    using pwmSequence1_2 = Meta::nth_element<2, pwmSequences1>; // pwm 9

    using pwmGroup1 = Mcu::Stm::Pwm::Servo<3, clock, MCU>; // pwm 2, 4, 5, 6
    
    // I2C 1
    using eeprom_sda = Mcu::Stm::Pin<gpiob, 7, MCU>; // af 4
    using eeprom_clk = Mcu::Stm::Pin<gpioa, 15, MCU>; // af 4
    
    // I2C 3
    using ext_sda = Mcu::Stm::Pin<gpiob, 5, MCU>; // af 8
    using ext_clk = Mcu::Stm::Pin<gpioa, 8, MCU>; // af 2
    
    // Testpoints
    
    using tp1 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using tp2 = Mcu::Stm::Pin<gpioc, 15, MCU>;
    
    template<typename Out>
    struct CrsfTelemetry {
        using out = Out;
        static inline constexpr External::Tick<systemTimer> telemTicks{5ms};
        
        enum class State : uint8_t {Gps, Batt, Temp, Rpm};
        
        static inline void ratePeriodic() {
            (++mTelemTick).on(telemTicks, []{
                switch(mState) {
                case State::Gps:
                    out::data(RC::Protokoll::Crsf::Type::Gps, mGps); // crsf gps data
                    mState = State::Batt;
                break;
                case State::Batt:
                    out::data(RC::Protokoll::Crsf::Type::Battery, mBatt); 
                    mState = State::Temp;
                break;
                case State::Temp:
                    out::data(RC::Protokoll::Crsf::Type::Temp1, mTemp);
                    mState = State::Rpm;
                break;
                case State::Rpm:
                    out::data(RC::Protokoll::Crsf::Type::Rpm1, mRpm);
                    mState = State::Batt;
                break;
                }
            });
        }
//    private:
        static inline State mState{State::Batt};
        static inline std::array<std::byte, 4> mGps;
        static inline std::array<std::byte, 8> mBatt; //volts amps mAh percent
        static inline std::array<std::byte, 2> mTemp;
        static inline std::array<std::byte, 2> mRpm;
        static inline External::Tick<systemTimer> mTelemTick;
    };
    using crsfTelemetry = CrsfTelemetry<crsf_out>;

    struct SbusCallback {
        static inline void update() {
            // uint16_t rpm = (uint16_t) sbus_pa::mSlots[0][1];
            // rpm += ((uint16_t) sbus_pa::mSlots[0][2]) << 8;
            // rpm *= 6;

            // crsfTelemetry::mRpm[1] = std::byte(rpm & 0xff);
            // crsfTelemetry::mRpm[0] = std::byte((rpm >> 8));

            // uint16_t volt = (uint16_t) sbus_pa::mSlots[3][2];
            // volt += ((uint16_t) sbus_pa::mSlots[3][1]) << 8;

            // volt /= 10;

            // crsfTelemetry::mBatt[1] = std::byte(volt & 0xff);
            // crsfTelemetry::mBatt[0] = std::byte(volt >> 8);


            // uint16_t curr = (uint16_t) sbus_pa::mSlots[2][2];
            // curr += ((uint16_t) (sbus_pa::mSlots[2][1] & std::byte(0x3f))) << 8;

            // curr /= 10;

            // crsfTelemetry::mBatt[3] = std::byte(curr & 0xff);
            // crsfTelemetry::mBatt[2] = std::byte(curr >> 8);
        }
    };

    struct VescCallback {
        static inline void update() {
            uint16_t temp = vesc_pa::mTemperature;
            temp /= 10;
            crsfTelemetry::mTemp[1] = std::byte(temp & 0xff);
            crsfTelemetry::mTemp[0] = std::byte((temp >> 8));

            uint16_t rpm = vesc_pa::mRPM;
            rpm /= 7;
            crsfTelemetry::mRpm[1] = std::byte(rpm & 0xff);
            crsfTelemetry::mRpm[0] = std::byte((rpm >> 8));

            uint16_t volt = vesc_pa::mVoltage;

            crsfTelemetry::mBatt[1] = std::byte(volt & 0xff);
            crsfTelemetry::mBatt[0] = std::byte(volt >> 8);

            uint16_t curr = vesc_pa::mCurrent;
            curr /= 10;
            crsfTelemetry::mBatt[3] = std::byte(curr & 0xff);
            crsfTelemetry::mBatt[2] = std::byte(curr >> 8);
        }
    };

    
    struct CrsfCallback {
        using out = crsf_out;
        using Param_t = RC::Protokoll::Crsf::Parameter;
        using PType = RC::Protokoll::Crsf::Parameter::Type;

        static inline void setParameter(const uint8_t index, const uint8_t value) {
            IO::outl<trace>("SetP adr: ", index, " v: ", value);
            if ((index >= 1) && (index <= params.size())) {
                params[index - 1].mValue = value;
                switch(index) {
                case 17: // BT
                    IO::outl<trace>("BT: ", value);
                    if (value) {
                        btpwr::set(false); // low on pin
                        btpwr::template dir<Mcu::Output>(); // floating is off
                    }
                    else {
                        btpwr::set(true); // high on pin
                        btpwr::template dir<Mcu::Input>(); // floating
                    }
                break;
                default:
                break;
                }
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
                return params[index - 1].mType == PType::Command;
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
            *ptr++ = ':';
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
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpioa::init();
        gpiob::init();
        gpioc::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        crsf::init();
        crsf::baud(420'000);
        crsftx::afunction(7);
        crsfrx::afunction(7);
        crsfrx::pullup<true>();

#ifdef USE_SPORT
        sport_uart::init();
        sport_uart::baud(57'600);
        sport_uart::invert(true);
        sportrxtx::afunction(7);
        sportrxtx::pulldown<true>();
        sporttxinv::afunction(7);
        sporttxinv::pulldown<true>();
#endif
#ifdef USE_ESC
        esc_uart::init();
        esc_uart::baud(115'200);
        escrxtx::afunction(7);
        escrxtx::pullup<true>();
#endif
#ifdef USE_VESC
        vesc_uart::init();
        vesc_uart::baud(115'200);
        vesc_uart::template rxEnable<false>();
        vescrxtx::afunction(7);
        vescrxtx::pullup<true>();
        vesc_uart::template halfDuplex<true>();
        vesc_uart::rxtxswap(true);
#endif

        sbus::init();
        // sbus_uart::init();
        // sbus_uart::baud(100'000);
        // sbus_uart::parity(true);
        // sbus_uart::invert(true);
        // sbus_uart::template halfDuplex<true>();
        // sbus_uart::rxtxswap(true);
        sbustx::afunction(7);
        sbustx::template pulldown<true>();

        bt_uart::init();
        // bt_uart::baud(9600);
        bt_uart::baud(115200); // BLE
        bttx::afunction(12);
        bttx::pullup<true>();
        btrx::afunction(12);
        btrx::pullup<true>();
      
        // bten::template dir<Mcu::Output>();
        // bten::set();
        btpwr::template dir<Mcu::Output>(); // floating is off (RTC config???)
        // btpwr::set();
        
        led1::template dir<Mcu::Output>();
        led2::template dir<Mcu::Output>();

        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();
        
        pwmSequence1::init();
        pwmGroup1::init();

        pwm1::afunction(1);
        pwm2::afunction(2);
        pwm3::afunction(1);
        pwm4::afunction(2);
        pwm5::afunction(2);
        pwm6::afunction(2);
    }
};

template<typename Config>
struct Devices<CC02, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
//    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
    using trace = Arm::Trace<clock, 2_MHz, 128>; 

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    struct CrsfCallback;
    
    // Usart 1
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfCallback, trace, MCU>;
    using crsf    = Mcu::Stm::Uart<1, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;
    using crsftx = Mcu::Stm::Pin<gpiob, 6, MCU>; // AF 7
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>; // AF 7
    
    // Usart 3
#ifdef USE_SPORT
    using sport_pa = RC::SPort::Master::ProtocolAdapter<0, systemTimer, MCU>;
    using sport_uart = Mcu::Stm::Uart<3, sport_pa, 16, std::byte, clock, MCU>;
    using sport = RC::SPort::Master::Fsm<sport_uart, systemTimer, MCU>;
    using sportrxtx = Mcu::Stm::Pin<gpiob, 11, MCU>; // rx, AF 7
    using sporttxinv = Mcu::Stm::Pin<gpiob, 9, MCU>; // tx, AF 7
#endif
#ifdef USE_ESC
    using esc_pa = RC::ESCape::Master::ProtocolAdapter<0, systemTimer, MCU>;
    using esc_uart = Mcu::Stm::Uart<3, esc_pa, 16, std::byte, clock, MCU>;
    using esc = RC::ESCape::Master::Fsm<esc_uart, systemTimer, MCU>;
    using escrxtx = Mcu::Stm::Pin<gpiob, 11, MCU>; // rx, AF 7
    using esctxinv = Mcu::Stm::Pin<gpiob, 9, MCU>; // tx, AF 7
#endif
#ifdef USE_VESC
    struct VescCallback;
    using vesc_pa = RC::VESC::Master::ProtocolAdapter<0, VescCallback, systemTimer, MCU>;
    using vesc_uart = Mcu::Stm::Uart<3, vesc_pa, 16, std::byte, clock, MCU>;
    using vesc = RC::VESC::Master::Fsm<vesc_uart, systemTimer, MCU>;
    using vescrxtx = Mcu::Stm::Pin<gpiob, 11, MCU>; // rx, AF 7
//    using vesctxinv = Mcu::Stm::Pin<gpiob, 9, MCU>; // tx, AF 7
#endif
    
    // Usart 2

    // use SBUS2 proto
    struct SbusCallback;
    using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    using sbustx = Mcu::Stm::Pin<gpiob, 4, MCU>; // rx, AF 7, need swap rxtx-pins
    
    // Uart 4? PC10, PC11 (nicht nutzbar)
    
    using led1 = Mcu::Stm::Pin<gpiob, 13, MCU>;
    using led2 = Mcu::Stm::Pin<gpioa, 0, MCU>;

    struct PassThru1 {
        static inline void start() {
            sbus_uart::clear();
            sbus_pa::reset();
            sbus::stop();
            sbus_uart::template halfDuplex<true>();
            sbustx::template pullup<true>();
            sbus_uart::baud(38'400);
            sbus_uart::parityOff();
            sbus_uart::invert(false);            
        }
        static inline void stop() {
            sbus_uart::clear();
            sbus_pa::reset();
            sbus::start();
            sbus_uart::template halfDuplex<true>();
            sbustx::template pulldown<true>();
            sbus_uart::baud(100'000);
            sbus_uart::parity(true);
            sbus_uart::invert(true);
        }
        static inline void put(const std::byte b) {
             sbus_uart::put(b);            
        }
    };
    using passthru = PassThru1;


    struct SendBack;

    // LPuart 1
    using btpwr = Mcu::Stm::Pin<gpioc, 13, MCU>;
    using bten = Mcu::Stm::Pin<gpioc, 14, MCU>;
    using robo_pa = External::QtRobo::ProtocollAdapter<0, systemTimer, passthru, SendBack>;
    using bt_uart = Mcu::Stm::LpUart<1, robo_pa, 16, std::byte, clock, MCU>;
    using btrx = Mcu::Stm::Pin<gpioa, 3, MCU>; // AF 12
    using bttx = Mcu::Stm::Pin<gpioa, 2, MCU>; // AF 12

    struct SendBack {
        static inline void put(auto b) {
            bt_uart::put(b);
        }
    };
 
    // PWM Timer: t2, t3, t4, t15
    using pwm1 = Mcu::Stm::Pin<gpioa, 1, MCU>; // tim2, ch2, af 1
    using pwm2 = Mcu::Stm::Pin<gpioa, 4, MCU>; // tim3, ch2, af 2
    using pwm3 = Mcu::Stm::Pin<gpioa, 5, MCU>; // tim2, ch1, af 1
    using pwm4 = Mcu::Stm::Pin<gpioa, 6, MCU>; // tim3, ch1, af 2
    using pwm5 = Mcu::Stm::Pin<gpiob, 0, MCU>; // tim3, ch3, af 2
    using pwm6 = Mcu::Stm::Pin<gpiob, 1, MCU>; // tim3, ch4, af 2
    using pwm7 = Mcu::Stm::Pin<gpioa, 12, MCU>; // tim4, ch2, af 10
    using pwm8 = Mcu::Stm::Pin<gpioa, 11, MCU>; // tim4, ch1, af 10
    using pwm9 = Mcu::Stm::Pin<gpioa, 9, MCU>; // tim2, ch3, af 10
    using pwm10 = Mcu::Stm::Pin<gpiob, 14, MCU>; // tim15, ch1, af 1

    using dma1 = Mcu::Stm::Dma::Controller<1, MCU>;
    using pwmDmaChannel1 = Mcu::Stm::Dma::Channel<dma1, 1, MCU>;    
    using pwmDmaChannel2 = Mcu::Stm::Dma::Channel<dma1, 2, MCU>;    
    using pwmDmaChannel3 = Mcu::Stm::Dma::Channel<dma1, 3, MCU>;    
    // using pwmDmaChannel4 = Mcu::Stm::Dma::Channel<dma1, 4, MCU>;
    using pwmDmaChannels = Meta::List<pwmDmaChannel1, pwmDmaChannel2, pwmDmaChannel3>;    
    using pwmSequence1 = Mcu::Stm::Pwm::Sequence<2, 10, pwmDmaChannels, clock, MCU>; // timer 2, sequence-length 
    using pwmSequences1 = pwmSequence1::pwms;
    using pwmSequence1_0 = Meta::nth_element<0, pwmSequences1>; // pwm 3
    using pwmSequence1_1 = Meta::nth_element<1, pwmSequences1>; // pwm 1
    using pwmSequence1_2 = Meta::nth_element<2, pwmSequences1>; // pwm 9

    using pwmGroup1 = Mcu::Stm::Pwm::Servo<3, clock, MCU>; // pwm 2, 4, 5, 6
    
    // I2C 1
    using eeprom_sda = Mcu::Stm::Pin<gpiob, 7, MCU>; // af 4
    using eeprom_clk = Mcu::Stm::Pin<gpioa, 15, MCU>; // af 4
    
    // I2C 3
    using ext_sda = Mcu::Stm::Pin<gpiob, 5, MCU>; // af 8
    using ext_clk = Mcu::Stm::Pin<gpioa, 8, MCU>; // af 2
    
    // Testpoints
    
    using tp1 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using tp2 = Mcu::Stm::Pin<gpioc, 15, MCU>;
    
    template<typename Out>
    struct CrsfTelemetry {
        using out = Out;
        static inline constexpr External::Tick<systemTimer> telemTicks{5ms};
        
        enum class State : uint8_t {Gps, Batt, Temp, Rpm};
        
        static inline void ratePeriodic() {
            (++mTelemTick).on(telemTicks, []{
                switch(mState) {
                case State::Gps:
                    out::data(RC::Protokoll::Crsf::Type::Gps, mGps); // crsf gps data
                    mState = State::Batt;
                break;
                case State::Batt:
                    out::data(RC::Protokoll::Crsf::Type::Battery, mBatt); 
                    mState = State::Temp;
                break;
                case State::Temp:
                    out::data(RC::Protokoll::Crsf::Type::Temp1, mTemp);
                    mState = State::Rpm;
                break;
                case State::Rpm:
                    out::data(RC::Protokoll::Crsf::Type::Rpm1, mRpm);
                    mState = State::Batt;
                break;
                }
            });
        }
//    private:
        static inline State mState{State::Batt};
        static inline std::array<std::byte, 4> mGps;
        static inline std::array<std::byte, 8> mBatt; //volts amps mAh percent
        static inline std::array<std::byte, 2> mTemp;
        static inline std::array<std::byte, 2> mRpm;
        static inline External::Tick<systemTimer> mTelemTick;
    };
    using crsfTelemetry = CrsfTelemetry<crsf_out>;

    struct SbusCallback {
        static inline void update() {
            uint16_t rpm = (uint16_t) sbus_pa::mSlots[0][1];
            rpm += ((uint16_t) sbus_pa::mSlots[0][2]) << 8;
            rpm *= 6;

            crsfTelemetry::mRpm[1] = std::byte(rpm & 0xff);
            crsfTelemetry::mRpm[0] = std::byte((rpm >> 8));

            uint16_t volt = (uint16_t) sbus_pa::mSlots[3][2];
            volt += ((uint16_t) sbus_pa::mSlots[3][1]) << 8;

            volt /= 10;

            crsfTelemetry::mBatt[1] = std::byte(volt & 0xff);
            crsfTelemetry::mBatt[0] = std::byte(volt >> 8);


            uint16_t curr = (uint16_t) sbus_pa::mSlots[2][2];
            curr += ((uint16_t) (sbus_pa::mSlots[2][1] & std::byte(0x3f))) << 8;

            curr /= 10;

            crsfTelemetry::mBatt[3] = std::byte(curr & 0xff);
            crsfTelemetry::mBatt[2] = std::byte(curr >> 8);
        }
    };

    struct VescCallback {
        static inline void update() {
            uint16_t temp = vesc_pa::mTemperature;
            temp /= 10;
            crsfTelemetry::mTemp[1] = std::byte(temp & 0xff);
            crsfTelemetry::mTemp[0] = std::byte((temp >> 8));

            uint16_t rpm = vesc_pa::mRPM;
            rpm /= 7;
            crsfTelemetry::mRpm[1] = std::byte(rpm & 0xff);
            crsfTelemetry::mRpm[0] = std::byte((rpm >> 8));

            uint16_t volt = vesc_pa::mVoltage;

            crsfTelemetry::mBatt[1] = std::byte(volt & 0xff);
            crsfTelemetry::mBatt[0] = std::byte(volt >> 8);

            uint16_t curr = vesc_pa::mCurrent;
            curr /= 10;
            crsfTelemetry::mBatt[3] = std::byte(curr & 0xff);
            crsfTelemetry::mBatt[2] = std::byte(curr >> 8);
        }
    };

    
    struct CrsfCallback {
        using out = crsf_out;
        using Param_t = RC::Protokoll::Crsf::Parameter;
        using PType = RC::Protokoll::Crsf::Parameter::Type;

        static inline void setParameter(const uint8_t index, const uint8_t value) {
            IO::outl<trace>("SetP adr: ", index, " v: ", value);
            if ((index >= 1) && (index <= params.size())) {
                params[index - 1].mValue = value;
                switch(index) {
                case 17: // BT
                    IO::outl<trace>("BT: ", value);
                    if (value) {
                        btpwr::set(false); // low on pin
                        btpwr::template dir<Mcu::Output>(); // floating is off
                    }
                    else {
                        btpwr::set(true); // high on pin
                        btpwr::template dir<Mcu::Input>(); // floating
                    }
                break;
                default:
                break;
                }
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
                return params[index - 1].mType == PType::Command;
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
            *ptr++ = ':';
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
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpioa::init();
        gpiob::init();
        gpioc::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        crsf::init();
        crsf::baud(420'000);
        crsftx::afunction(7);
        crsfrx::afunction(7);
        crsfrx::pullup<true>();

#ifdef USE_SPORT
        sport_uart::init();
        sport_uart::baud(57'600);
        sport_uart::invert(true);
        sportrxtx::afunction(7);
        sportrxtx::pulldown<true>();
        sporttxinv::afunction(7);
        sporttxinv::pulldown<true>();
#endif
#ifdef USE_ESC
        esc_uart::init();
        esc_uart::baud(115'200);
        escrxtx::afunction(7);
        escrxtx::pullup<true>();
#endif
#ifdef USE_VESC
        vesc_uart::init();
        vesc_uart::baud(115'200);
        vesc_uart::template rxEnable<false>();
        vescrxtx::afunction(7);
        vescrxtx::pullup<true>();
        vesc_uart::template halfDuplex<true>();
        vesc_uart::rxtxswap(true);
#endif

        sbus::init();
        sbus_uart::init();
        sbus_uart::baud(100'000);
        sbus_uart::parity(true);
        sbus_uart::invert(true);
        sbus_uart::template halfDuplex<true>();
        sbus_uart::rxtxswap(true);
        sbustx::afunction(7);
        sbustx::template pulldown<true>();

        bt_uart::init();
        // bt_uart::baud(9600);
        bt_uart::baud(115200); // BLE
        bttx::afunction(12);
        bttx::pullup<true>();
        btrx::afunction(12);
        btrx::pullup<true>();
      
        // bten::template dir<Mcu::Output>();
        // bten::set();
        btpwr::template dir<Mcu::Output>(); // floating is off (RTC config???)
        // btpwr::set();
        
        led1::template dir<Mcu::Output>();
        led2::template dir<Mcu::Output>();

        tp1::template dir<Mcu::Output>();
        tp2::template dir<Mcu::Output>();
        
        pwmSequence1::init();
        pwmGroup1::init();

        pwm1::afunction(1);
        pwm1::speed<Mcu::High>();
        pwm2::afunction(2);
        pwm2::speed<Mcu::High>();
        pwm3::afunction(1);
        pwm3::speed<Mcu::High>();
        pwm4::afunction(2);
        pwm4::speed<Mcu::High>();
        pwm5::afunction(2);
        pwm5::speed<Mcu::High>();
        pwm6::afunction(2);
        pwm6::speed<Mcu::High>();

    }
};

template<typename Config>
struct Devices<CC01, Config, Mcu::Stm::Stm32G431> {
    using MCU = Mcu::Stm::Stm32G431;
//    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 1'000_Hz, Mcu::Stm::HSI>, MCU>;
    using clock = Mcu::Stm::Clock<Mcu::Stm::ClockConfig<170_MHz, 2'000_Hz, Mcu::Stm::HSI>, MCU>;
    using systemTimer = Mcu::Stm::SystemTimer<clock, Mcu::UseInterrupts<false>, MCU>; 
    using trace = Arm::Trace<clock, 2_MHz, 128>; 

    using gpioa = Mcu::Stm::GPIO<Mcu::Stm::A, MCU>;
    using gpiob = Mcu::Stm::GPIO<Mcu::Stm::B, MCU>;
    using gpioc = Mcu::Stm::GPIO<Mcu::Stm::C, MCU>;

    struct CrsfCallback;
    
    // Usart 1
    using crsf_pa = RC::Protokoll::Crsf::Adapter<0, CrsfCallback, trace, MCU>;
    using crsf    = Mcu::Stm::Uart<1, crsf_pa, RC::Protokoll::Crsf::maxMessageSize, std::byte, clock, MCU>;
    using crsf_out= RC::Protokoll::Crsf::Generator<crsf, systemTimer, MCU>;
    using crsftx = Mcu::Stm::Pin<gpiob, 6, MCU>; // AF 7
    using crsfrx = Mcu::Stm::Pin<gpioa, 10, MCU>; // AF 7
    
    // Usart 3
#ifdef USE_SPORT
    using sport_pa = RC::SPort::Master::ProtocolAdapter<0, systemTimer, MCU>;
    using sport_uart = Mcu::Stm::Uart<3, sport_pa, 16, std::byte, clock, MCU>;
    using sport = RC::SPort::Master::Fsm<sport_uart, systemTimer, MCU>;
    using sportrxtx = Mcu::Stm::Pin<gpiob, 11, MCU>; // rx, AF 7
    using sporttxinv = Mcu::Stm::Pin<gpiob, 9, MCU>; // tx, AF 7
#endif
#ifdef USE_ESC
    using esc_pa = RC::ESCape::Master::ProtocolAdapter<0, systemTimer, MCU>;
    using esc_uart = Mcu::Stm::Uart<3, esc_pa, 16, std::byte, clock, MCU>;
    using esc = RC::ESCape::Master::Fsm<esc_uart, systemTimer, MCU>;
    using escrxtx = Mcu::Stm::Pin<gpiob, 11, MCU>; // rx, AF 7
    using esctxinv = Mcu::Stm::Pin<gpiob, 9, MCU>; // tx, AF 7
#endif
#ifdef USE_VESC
    struct VescCallback;
    using vesc_pa = RC::VESC::Master::ProtocolAdapter<0, VescCallback, systemTimer, MCU>;
    using vesc_uart = Mcu::Stm::Uart<3, vesc_pa, 16, std::byte, clock, MCU>;
    using vesc = RC::VESC::Master::Fsm<vesc_uart, systemTimer, MCU>;
    using vescrxtx = Mcu::Stm::Pin<gpiob, 11, MCU>; // rx, AF 7
//    using vesctxinv = Mcu::Stm::Pin<gpiob, 9, MCU>; // tx, AF 7
#endif
    
    // Usart 2

    // use SBUS2 proto
    struct SbusCallback;
    using sbus_pa = RC::Protokoll::SBus2::Adapter<0, SbusCallback>;
    using sbus_uart = Mcu::Stm::Uart<2, sbus_pa, 32, std::byte, clock, MCU>;
    using sbus = RC::Protokoll::SBus2::Fsm<sbus_uart, systemTimer>;
    using sbustx = Mcu::Stm::Pin<gpiob, 4, MCU>; // rx, AF 7, need swap rxtx-pins
    
    // Uart 4? PC10, PC11 (nicht nutzbar)
    
    using led1 = Mcu::Stm::Pin<gpiob, 13, MCU>;
    using led2 = Mcu::Stm::Pin<gpioa, 0, MCU>;

    struct PassThru1 {
        static inline void start() {
            sbus_uart::clear();
            sbus_pa::reset();
            sbus::stop();
            sbus_uart::template halfDuplex<true>();
            sbustx::template pullup<true>();
            sbus_uart::baud(38'400);
            sbus_uart::parityOff();
            sbus_uart::invert(false);            
        }
        static inline void stop() {
            sbus_uart::clear();
            sbus_pa::reset();
            sbus::start();
            sbus_uart::template halfDuplex<true>();
            sbustx::template pulldown<true>();
            sbus_uart::baud(100'000);
            sbus_uart::parity(true);
            sbus_uart::invert(true);
        }
        static inline void put(const std::byte b) {
             sbus_uart::put(b);            
        }
    };
    using passthru = PassThru1;


    struct SendBack;

    // LPuart 1
    using btpwr = Mcu::Stm::Pin<gpioc, 13, MCU>;
    using bten = Mcu::Stm::Pin<gpioc, 14, MCU>;
    using robo_pa = External::QtRobo::ProtocollAdapter<0, systemTimer, passthru, SendBack>;
    using bt_uart = Mcu::Stm::LpUart<1, robo_pa, 16, std::byte, clock, MCU>;
    using btrx = Mcu::Stm::Pin<gpioa, 3, MCU>; // AF 12
    using bttx = Mcu::Stm::Pin<gpioa, 2, MCU>; // AF 12

    struct SendBack {
        static inline void put(auto b) {
            bt_uart::put(b);
        }
    };
 
    // PWM Timer: t2, t3, t4, t15
    using pwm1 = Mcu::Stm::Pin<gpioa, 1, MCU>; // tim2, ch2, af 1
    using pwm2 = Mcu::Stm::Pin<gpioa, 4, MCU>; // tim3, ch2, af 2
    using pwm3 = Mcu::Stm::Pin<gpioa, 5, MCU>; // tim2, ch1, af 1
    using pwm4 = Mcu::Stm::Pin<gpioa, 6, MCU>; // tim3, ch1, af 2
    using pwm5 = Mcu::Stm::Pin<gpiob, 0, MCU>; // tim3, ch3, af 2
    using pwm6 = Mcu::Stm::Pin<gpiob, 1, MCU>; // tim3, ch4, af 2
    using pwm7 = Mcu::Stm::Pin<gpioa, 12, MCU>; // tim4, ch2, af 10
    using pwm8 = Mcu::Stm::Pin<gpioa, 11, MCU>; // tim4, ch1, af 10
    using pwm9 = Mcu::Stm::Pin<gpioa, 9, MCU>; // tim2, ch3, af 10
    using pwm10 = Mcu::Stm::Pin<gpiob, 14, MCU>; // tim15, ch1, af 1
    
    using cppm_period = std::integral_constant<uint16_t, 32800>; // 1ms -> 1640 (SBus delta: 1640 = 1812 - 172)
    using cppm_prescaler = std::integral_constant<uint16_t, 104>; // timer-freq = 1,634 (ca. 1,640) MHz
    // using timer2 = Mcu::Stm::Timer<2, cppm_period, cppm_prescaler, Mcu::Stm::Trigger<false>, MCU>;
    // using pwmGen1 = Mcu::Stm::Pwm::Sequence<2, 1, clock, MCU>; // timer 2, sequence-length 1
    
    
    // I2C 1
    using eeprom_sda = Mcu::Stm::Pin<gpiob, 7, MCU>; // af 4
    using eeprom_clk = Mcu::Stm::Pin<gpioa, 15, MCU>; // af 4
    
    // I2C 3
    using ext_sda = Mcu::Stm::Pin<gpiob, 5, MCU>; // af 8
    using ext_clk = Mcu::Stm::Pin<gpioa, 8, MCU>; // af 2
    
    // Testpoints
    
    using tp1 = Mcu::Stm::Pin<gpioa, 7, MCU>;
    using tp2 = Mcu::Stm::Pin<gpioc, 15, MCU>;
    
    template<typename Out>
    struct CrsfTelemetry {
        using out = Out;
        static inline constexpr External::Tick<systemTimer> telemTicks{5ms};
        
        enum class State : uint8_t {Gps, Batt, Temp, Rpm};
        
        static inline void ratePeriodic() {
            (++mTelemTick).on(telemTicks, []{
                switch(mState) {
                case State::Gps:
                    out::data(RC::Protokoll::Crsf::Type::Gps, mGps); // crsf gps data
                    mState = State::Batt;
                break;
                case State::Batt:
                    out::data(RC::Protokoll::Crsf::Type::Battery, mBatt); 
                    mState = State::Temp;
                break;
                case State::Temp:
                    out::data(RC::Protokoll::Crsf::Type::Temp1, mTemp);
                    mState = State::Rpm;
                break;
                case State::Rpm:
                    out::data(RC::Protokoll::Crsf::Type::Rpm1, mRpm);
                    mState = State::Batt;
                break;
                }
            });
        }
//    private:
        static inline State mState{State::Batt};
        static inline std::array<std::byte, 4> mGps;
        static inline std::array<std::byte, 8> mBatt; //volts amps mAh percent
        static inline std::array<std::byte, 2> mTemp;
        static inline std::array<std::byte, 2> mRpm;
        static inline External::Tick<systemTimer> mTelemTick;
    };
    using crsfTelemetry = CrsfTelemetry<crsf_out>;

    struct SbusCallback {
        static inline void update() {
            uint16_t rpm = (uint16_t) sbus_pa::mSlots[0][1];
            rpm += ((uint16_t) sbus_pa::mSlots[0][2]) << 8;
            rpm *= 6;

            crsfTelemetry::mRpm[1] = std::byte(rpm & 0xff);
            crsfTelemetry::mRpm[0] = std::byte((rpm >> 8));

            uint16_t volt = (uint16_t) sbus_pa::mSlots[3][2];
            volt += ((uint16_t) sbus_pa::mSlots[3][1]) << 8;

            volt /= 10;

            crsfTelemetry::mBatt[1] = std::byte(volt & 0xff);
            crsfTelemetry::mBatt[0] = std::byte(volt >> 8);


            uint16_t curr = (uint16_t) sbus_pa::mSlots[2][2];
            curr += ((uint16_t) (sbus_pa::mSlots[2][1] & std::byte(0x3f))) << 8;

            curr /= 10;

            crsfTelemetry::mBatt[3] = std::byte(curr & 0xff);
            crsfTelemetry::mBatt[2] = std::byte(curr >> 8);
        }
    };

    struct VescCallback {
        static inline void update() {
            uint16_t temp = vesc_pa::mTemperature;
            temp /= 10;
            crsfTelemetry::mTemp[1] = std::byte(temp & 0xff);
            crsfTelemetry::mTemp[0] = std::byte((temp >> 8));

            uint16_t rpm = vesc_pa::mRPM;
            rpm /= 2;
            crsfTelemetry::mRpm[1] = std::byte(rpm & 0xff);
            crsfTelemetry::mRpm[0] = std::byte((rpm >> 8));

            uint16_t volt = vesc_pa::mVoltage;

            crsfTelemetry::mBatt[1] = std::byte(volt & 0xff);
            crsfTelemetry::mBatt[0] = std::byte(volt >> 8);

            uint16_t curr = vesc_pa::mCurrent;
            curr /= 10;
            crsfTelemetry::mBatt[3] = std::byte(curr & 0xff);
            crsfTelemetry::mBatt[2] = std::byte(curr >> 8);
        }
    };

    
    struct CrsfCallback {
        using out = crsf_out;
        using Param_t = RC::Protokoll::Crsf::Parameter;
        using PType = RC::Protokoll::Crsf::Parameter::Type;

        static inline void setParameter(const uint8_t index, const uint8_t value) {
            IO::outl<trace>("SetP adr: ", index, " v: ", value);
            if ((index >= 1) && (index <= params.size())) {
                params[index - 1].mValue = value;
                switch(index) {
                case 17: // BT
                    IO::outl<trace>("BT: ", value);
                    if (value) {
                        btpwr::set(false); // low on pin
                        btpwr::template dir<Mcu::Output>(); // floating is off
                    }
                    else {
                        btpwr::set(true); // high on pin
                        btpwr::template dir<Mcu::Input>(); // floating
                    }
                break;
                default:
                break;
                }
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
                return params[index - 1].mType == PType::Command;
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
            *ptr++ = ':';
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
    
    static inline void init() {
        clock::init();
        trace::init();
        systemTimer::init();
        
        gpioa::init();
        gpiob::init();
        gpioc::init();

        SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);

        crsf::init();
        crsf::baud(420'000);
        crsftx::afunction(7);
        crsfrx::afunction(7);
        crsfrx::pullup<true>();

#ifdef USE_SPORT
        sport_uart::init();
        sport_uart::baud(57'600);
        sport_uart::invert(true);
        sportrxtx::afunction(7);
        sportrxtx::pulldown<true>();
        sporttxinv::afunction(7);
        sporttxinv::pulldown<true>();
#endif
#ifdef USE_ESC
        esc_uart::init();
        esc_uart::baud(115'200);
        escrxtx::afunction(7);
        escrxtx::pullup<true>();
#endif
#ifdef USE_VESC
        vesc_uart::init();
        vesc_uart::baud(115'200);
        vesc_uart::template rxEnable<false>();
        vescrxtx::afunction(7);
        vescrxtx::pullup<true>();
        vesc_uart::template halfDuplex<true>();
        vesc_uart::rxtxswap(true);
#endif

        sbus::init();
        sbus_uart::init();
        sbus_uart::baud(100'000);
        sbus_uart::parity(true);
        sbus_uart::invert(true);
        sbus_uart::template halfDuplex<true>();
        sbus_uart::rxtxswap(true);
        sbustx::afunction(7);
        sbustx::template pulldown<true>();

        bt_uart::init();
        // bt_uart::baud(9600);
        bt_uart::baud(115200); // BLE
        bttx::afunction(12);
        bttx::pullup<true>();
        btrx::afunction(12);
        btrx::pullup<true>();
      
        // bten::template dir<Mcu::Output>();
        // bten::set();
        btpwr::template dir<Mcu::Output>(); // floating is off (RTC config???)
        // btpwr::set();
        
        led1::template dir<Mcu::Output>();
        led2::template dir<Mcu::Output>();

        // tp1::template dir<Mcu::Output>();
        // tp2::template dir<Mcu::Output>();
        
        // pwmGen1::init();
    }
};

