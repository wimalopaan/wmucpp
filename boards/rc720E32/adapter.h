/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>
#include "ressources.h"

template<typename Mpx, typename Pin, typename Debug = void>
struct MpxAdapter {
    static inline constexpr uint8_t channel = Mpx::channel;
    static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<Pin, Mpx, Mcu::Stm::AlternateFunctions::CC<channel>>;
    static inline void init() {
        IO::outl<Debug>("Mpx init");
        RessourceCount<Mpx>::acquire([]{
            IO::outl<Debug>("Mpx acquire");
            Mpx::init(); // only if first channel
        });
        Pin::afunction(af);
    }
    static inline void reset() {
        IO::outl<Debug>("Mpx reset");
        RessourceCount<Mpx>::release([]{
            IO::outl<Debug>("Mpx release");
            // Mpx::reset(); // only if last channel
        });
        Pin::analog();
    }
    static inline std::pair<uint8_t, uint8_t> hwVersion() {
        return {255, 255};
    }
    static inline std::pair<uint8_t, uint8_t> fwVersion() {
        return {255, 255};
    }
    static inline uint16_t turns() {
        return 0;
    }
    static inline uint16_t actualPos() {
        return 0;
    }
    static inline void speed(uint16_t) {
    }
    static inline void offset(uint16_t) {
    }
    static inline void zero() {
    }
    static inline void update() {
    }
    static inline void periodic() {
    }
    static inline void ratePeriodic() {
    }
};

template<typename Pwm, typename Pin, uint8_t Channel, typename Storage, typename EEpromOffsetIndex, typename Debug = void>
struct PwmAdapter {
    static_assert(Channel >= 1);
    using pwm = Pwm;

    static inline constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<Pin, pwm, Mcu::Stm::AlternateFunctions::CC<Channel>>;

    template<typename T>
    struct getValue {
        static inline constexpr size_t value = 0;
    };
    template<typename T, auto V>
    struct getValue<std::integral_constant<T, V>> {
        static inline constexpr size_t value = V;
    };

    static inline void set(const uint16_t v) {
        if (!std::is_same_v<EEpromOffsetIndex, void>) {
            const int vv = std::clamp(v + (Storage::eeprom.esc_mid[getValue<EEpromOffsetIndex>::value] - 100), 172, 1812);
            Pwm::set(Channel - 1, vv);
        }
        else {
            Pwm::set(Channel - 1, v);
        }
    }
    static inline void init() {
        IO::outl<Debug>("# PWMA ", Channel, " init");
        RessourceCount<pwm>::acquire([]{
            IO::outl<Debug>("# PWMA ", Channel, " acquire");
            pwm::init(); // only if first channel
        });
        Pin::afunction(af);
    }
    static inline void reset() {
        IO::outl<Debug>("# PWMA ", Channel, " reset");
        RessourceCount<pwm>::release([]{
            IO::outl<Debug>("# PWMA ", Channel, " release");
            pwm::reset(); // only if last channel
        });
        Pin::analog();
    }
    static inline std::pair<uint8_t, uint8_t> hwVersion() {
        return {255, 255};
    }
    static inline std::pair<uint8_t, uint8_t> fwVersion() {
        return {255, 255};
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

