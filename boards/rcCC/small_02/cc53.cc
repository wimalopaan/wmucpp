#define USE_MCU_STM_V3
#define USE_CRSF_V2
#define USE_DEVICES4

#define NDEBUG

// Usart 3
// #define USE_ESC
// #define USE_SPORT
#define USE_VESC
//#define USE_GPS

// USART 2
// #define USE_DMA_SBUS

#include <cstdint>
#include <array>
#include <chrono>
#include <cassert>

#include "eeprom.h"
#include "devices.h"
#include "telemetry.h"
#include "vesc.h"
#include "crsf.h"

using namespace std::literals::chrono_literals;

struct Storage {
    static inline void init() {
        std::memcpy(&eeprom, &eeprom_flash, sizeof(EEProm));
    }
    __attribute__((__section__(".eeprom")))
    static inline const EEProm eeprom_flash{};

    static inline EEProm eeprom;
};

struct Data {
    static inline std::array<uint16_t, 64> mChannels{}; // sbus [172, 1812], center = 992
    static inline std::array<uint8_t, 256> mAltData{};

    static inline std::array<uint16_t, 4> rpm;
    static inline std::array<uint16_t, 4> temp;
    static inline uint16_t speed;
    static inline std::array<uint16_t, 4> current;
    static inline std::array<uint16_t, 4> voltage;
};

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;

    using pwm1 = devs::pwm1;
    using pwmGroup1 = devs::pwmGroup1;
    using crsf = devs::crsf;
    using crsf_pa = devs::crsf_pa;
    using crsf_out = devs::crsf_out;
    using crsfCallback = devs::crsfCallback;
    using crsfTelemetry = crsfCallback::crsfTelemetry;

    using sbusfilter = crsfCallback::sbusfilter;

#ifdef USE_SPORT
    using sport = devs::sport;
    using sport_pa = devs::sport_pa;
    using sport_uart = devs::sport_uart;
#endif
#ifdef USE_ESC
    using esc = devs::esc;
    using esc_pa = devs::esc_pa;
    using esc_uart = devs::esc_uart;
#endif
#ifdef USE_VESC
    using vesc = devs::vesc;
    using vesc_pa = devs::vesc_pa;
    using vesc_uart = devs::vesc_uart;
#endif
#ifdef USE_GPS
    using vtg = devs::vtg;
    using gsv = devs::gsv;
    using rmc = devs::rmc;
    using gps_pa = devs::gps_pa;
    using gps_uart = devs::gps_uart;
#endif

    // using sbus = devs::sbus;

#ifndef USE_DMA_SBUS
    // using sbus_pa = devs::sbus_pa;
#endif

    using bt_uart = devs::bt_uart;
    using robo_pa = devs::robo_pa;

    using aux1_uart = devs::aux1_uart;
    using aux2_uart = devs::aux2_uart;

    using sbus_aux1 = devs::sbus_aux1;
    using sbus_aux2 = devs::sbus_aux2;

    using led1 = devs::led1;
    using led2 = devs::led2;

    using btpwr = devs::btpwr;
    using bten = devs::bten;

    using tp1 = devs::tp1;
    using tp2 = devs::tp2;

    using i2c_2 = devs::i2c_2;
    using switches = devs::switches;

    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> initTicks{500ms};

    enum class State : uint8_t {Undefined, Init, Info, DevsInit, Run};

    static inline bool crsfOn = false;

    static inline void update() {
        crsfCallback::update();
        crsfCallback::callbacks();
    }

    static inline void boot(const bool b) {
        crsfCallback::bootMode = b;
    }

    static inline void set(const uint8_t) {
    }

    static inline void init() {
        devs::init();
        devs::led1::set();

        for(auto& v : Data::mChannels) {
            v = 992;
        }

        sbusfilter::init();
    }

    static inline void periodic() {
        tp1::set();
        trace::periodic();
        i2c_2::periodic();
        crsf::periodic();

        switches::periodic();

#ifdef USE_SPORT
        sport::periodic();
        sport_uart::periodic();
#endif
#ifdef USE_ESC
        esc::periodic();
        esc_uart::periodic();
#endif
#ifdef USE_VESC
        vesc::periodic();
        vesc_uart::periodic();
#endif
#ifndef USE_DMA_SBUS
        // sbus::periodic();
#endif
#ifdef USE_GPS
        gps_uart::periodic();
#endif
        aux1_uart::periodic();
        aux2_uart::periodic();

        bt_uart::periodic();
        tp1::reset();
    }
    static inline void ratePeriodic() {
        tp2::set();

        i2c_2::ratePeriodic();
        robo_pa::ratePeriodic();

        crsf_pa::copyChangedChannels(Data::mChannels);

        robo_pa::whenTargetChanged([&](const robo_pa::Target t, const robo_pa::index_type::nan_type i){
            IO::outl<trace>("Robo");
            switch(t) {
            case robo_pa::Target::Prop:
                if (i < Data::mChannels.size()) {
                    Data::mChannels[i] = robo_pa::propValues[i];
                }
                break;
            case robo_pa::Target::Switch:
                if (i < Data::mAltData.size()) {
                    Data::mAltData[i] = robo_pa::switchValues[i];
                }
                break;
            case robo_pa::Target::Toggle:
                if (i < Data::mAltData.size()) {
                    Data::mAltData[i] = robo_pa::toggleValues[i];
                }
                break;
            default:
                break;
            }
        });

        // sbus::set(std::span{&Data::mChannels[0], 16});
        sbus_aux1::set(std::span{&Data::mChannels[0], 16});

        sbusfilter::ratePeriodic();

#ifdef USE_SPORT
        sport::ratePeriodic();
#endif
#ifdef USE_ESC
        esc::throttle(Data::mChannels[0]);
        esc::ratePeriodic();
#endif
#ifdef USE_VESC
        vesc::throttle(Data::mChannels[0]);
        vesc::ratePeriodic();
#endif

        if (crsf_pa::packages() > 100) {
            crsfOn = true;
            led2::set();
        }
        if (crsfOn) {
            crsf_out::ratePeriodic();
            crsfTelemetry::ratePeriodic();
        }

#ifndef USE_DMA_SBUS
        // sbus::ratePeriodic();
#endif

        sbus_aux1::ratePeriodic();
        sbus_aux2::ratePeriodic();

        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Init;
                btpwr::reset(); // on
            });
            break;
        case State::Init:
            mStateTick.on(initTicks, []{
                if (i2c_2::isIdle()) {
                    mState = State::Info;
                }
                else {
                    mStateTick.reset();
                }
            });
            break;
        case State::Info:
            for(uint8_t a{0}; a < 0x7f; ++a) {
                if (i2c_2::isPresent(I2C::Address{a})) {
                    IO::outl<trace>("I2C2: ", a);
                }
            }
            mState = State::DevsInit;
            break;
        case State::DevsInit:
            mStateTick.on(initTicks, []{
                if (i2c_2::isIdle()) {
                    mState = State::Run;
                }
                else {
                    mStateTick.reset();
                }
            });
            break;
        case State::Run:
            pwmGroup1::set(0, Data::mChannels[0]);
            pwmGroup1::set(1, Data::mChannels[1]);
            pwmGroup1::set(2, Data::mChannels[2]);
            pwmGroup1::set(3, Data::mChannels[3]);

            (++mDebugTick).on(debugTicks, []{
                led1::toggle();
#ifdef USE_VESC
                IO::outl<trace>(
                            "ch0: ", Data::mChannels[0],
                            " maj: ", vesc_pa::mVersionMajor,
                            " min: ", vesc_pa::mVersionMinor,
                            " name: ", vesc_pa::mName
                //     " ch1: ", Data::mChannels[1],
                //     " ch2: ", Data::mChannels[2],
                //     " ch3: ", Data::mChannels[3],
                //     " v b: ", vesc_pa::mBytes,
                //     " v p: ", vesc_pa::mPackages,
                //     " v t: ", (uint8_t)vesc_pa::mType,
                //     " v l: ", vesc_pa::mLength
                    );
#endif
#ifdef USE_GPS
                std::array<char, External::GPS::Sentence::DecimalMaxWidth> raw{};

                gsv::numberRaw(raw);
                uint16_t sats = 0;
                std::from_chars(std::begin(raw), std::end(raw), sats);

                crsfTelemetry::sats(sats);

                vtg::speedRaw(raw);
                const float speed = etl::from_chars(raw);
                crsfTelemetry::speed(speed);
#endif
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                i2c_2::scan([](Mcu::Stm::I2C::Address){});
                break;
            case State::Info:
                break;
            case State::DevsInit:
                switches::init();
                break;
            case State::Run:
                // led2::set();
                break;
            }
        }
        tp2::reset();
    }
    private:
    static inline void i2c_scan_callback(Mcu::Stm::I2C::Address) {
    }
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline State mState{State::Undefined};
};

struct Setter;

template<typename L, typename T>
using CrsfCallback_WithSetter = CrsfCallback<L, Setter, T>;

using devs = Devices4<CC51, Storage, CrsfCallback_WithSetter, VescCallback>;
using gfsm = GFSM<devs>;

struct Setter {
    static inline void set(const uint8_t sw) {
        gfsm::set(sw);
    }
};

int main() {
    Storage::init();
    gfsm::init();
    gfsm::update();

    gfsm::boot(false);

    // NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    // __enable_irq();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}
void __assert_func (const char *, int, const char *, const char *){
    while(true) {
        devs::tp1::set();
        devs::tp1::reset();
    }
}

extern "C" {

void DMA1_Channel1_IRQHandler() {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
}

}
