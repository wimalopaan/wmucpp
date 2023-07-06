#include "devices.h"

#include "fsk.h"

#include <chrono>

using namespace std::literals::chrono_literals;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using trace = devs::trace;
    using systemTimer = devs::systemTimer;
    
    using pwm1 = devs::pwm1;
    
    struct Callback {
        template<auto N>
        static inline void process(const std::array<uint8_t, N>& data) {
            const uint16_t ch1 = data[1] + 256;
            pwm1::duty(ch1);
        }
    };
    
//    using demod = Demodulation<trace>;
    using demod = Demodulation<void, Callback, typename devs::pinb6>;
    
//    using adc1 = devs::adc1;

    using adc_i = devs::adc2;
    using dac_ref = devs::dac3;
    
    using i2c2 = devs::i2c2;
    using si = devs::si;
    
    enum class State : uint8_t {Undefined, Init, Setup, Conv, Wait};
    
//    static inline constexpr External::Tick<systemTimer> mInitTicks{500ms};
    
    static inline void init() {
        devs::init();
        devs::led::set();
    }
    static inline void periodic() {
        si::periodic();
        i2c2::periodic();
        
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
        break;
        case State::Init:
        break;
        case State::Setup:
        break;
        case State::Conv:
        break;
        case State::Wait:
            if (!adc_i::busy()) {
                aValue = adc_i::value();
                devs::pina11::reset();
                devs::dac1::set(aValue);
                devs::pinb5::set();
                iq_bit = demod::process({aValue, aValue});
                if (iq_bit.i) {
                    devs::pinb0::set();
                }
                else {
                    devs::pinb0::reset();
                }
                devs::pinb5::reset();
                ++ac;
                mState = State::Conv;
            }
        break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
            break;
            case State::Init:
            break;
            case State::Setup:
            break;
            case State::Conv:
            break;
            case State::Wait:
            break;
            }
        }
    }
    static inline void ratePeriodic() {
        
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
        break;
        case State::Init:
            if (adc_i::ready()) {
                dac_ref::set(0x0fff / 2);
                si::setOutput(1);
//                si::off();    
                mState = State::Setup;
            }
        break;
        case State::Setup:
            if (si::setFrequency(Units::hertz{6'995'000 * 4})) { // zf = 5 KHz
                mState = State::Conv;
            }
        break;
        case State::Conv:
            devs::pina11::set();
            adc_i::start();
            mState = State::Wait;
        break;
        case State::Wait:
        break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
            break;
            case State::Init:
            break;
            case State::Setup:
            break;
            case State::Conv:
            break;
            case State::Wait:
            break;
            }
        }

        if (++c == 48000) {
            c = 0;
////            IO::outl<trace>("systick: ", systemTimer::value, " r: ", r, " a: ", a);
//            IO::outl<trace>("State: ", (uint8_t)mState, " adc: ", aValue, " ac: ", ac);
            IO::outl<trace>("max i: ", Stats::max(), " e: ", demod::proto::ByteStuff::errors(), " p: ", demod::proto::ByteStuff::packages());
        }
    }
private:
    static inline IQ_Bit iq_bit;
    
    static inline uint32_t g{0};
    
    static inline uint32_t c;
    static inline uint16_t a;
    
    static inline State mState{State::Undefined};
    static inline uint16_t aValue{42};
    static inline uint16_t ac;
};

//extern "C" void SysTick_Handler()  {                               
////    devs::systemTimer::isr();
//    devs::pinb3::set();
//    devs::pinb3::reset();
//}

int main() {
    using devs = Devices<SDR01, Mcu::Stm::Stm32G431>;
    using gfsm = GFSM<devs>;
    gfsm::init();

    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
        });
    }
}

