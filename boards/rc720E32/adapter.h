#pragma once

#include <cstdint>
#include "ressources.h"

template<typename Pwm, typename Pin, uint8_t Channel, typename Debug = void>
struct PwmAdapter {
    static_assert(Channel >= 1);
    using pwm = Pwm;

    static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<Pin, pwm, Mcu::Stm::AlternateFunctions::CC<Channel>>;

    // Pin::_;
    // std::integral_constant<uint8_t, af>::_;

    static inline void set(const uint16_t v) {
        Pwm::set(Channel - 1, v);
    }
    static inline void init() {
        IO::outl<Debug>("PWMA", Channel, " init");
        RessourceCount<pwm>::acquire([]{
            IO::outl<Debug>("PWMA", Channel, " acquire");
            pwm::init(); // only if first channel
        });
        Pin::afunction(af);
    }
    static inline void reset() {
        IO::outl<Debug>("PWMA", Channel, " reset");
        RessourceCount<pwm>::release([]{
            IO::outl<Debug>("PWMA", Channel, " release");
            pwm::reset(); // only if last channel
        });
        Pin::analog();
    }
    static inline uint16_t current() {
        return 0;
    }
    static inline uint16_t rpm() {
        return 0;
    }
    static inline void update() {
    }
    static inline void periodic() {
    }
    static inline void ratePeriodic() {
    }
};

template<uint8_t Channel, typename Adc, typename Pin, typename Debug = void>
struct FeedbackAdapter {
    using adc = Adc;
    using pin = Pin;
    static inline void init() {
        RessourceCount<adc>::acquire([]{
            IO::outl<Debug>("FBA", Channel, " init");
            adc::init(); // only if first channel
            adc::oversample(8); // 256
            while(!adc::ready()) ;
            IO::outl<Debug>("FBA", Channel, " init ready");
        });
        pin::analog();
    }
    static inline void reset() {
        // todo: stoppping and restart ????

        // RessourceCount<adc>::release([]{
            // adc::reset(); // only if last channel
            // adc::dmaChannel::enable(false);
        // });
    }
    static inline uint16_t read() {
        return adc::values()[Channel];
    }
};

