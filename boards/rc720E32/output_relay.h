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

#include "output.h"
#include "iservo.h"

template<typename Devs>
struct Relays {
    using devs = Devs;
    using debug = devs::debug;
    using relay = devs::relay1;
    using sbus = devs::sbus1;

    using pulse_in = devs::pulse_in;
    using ibus_in = devs::ibus_in;
    using sbus_in = devs::sbus_in;
    using sumdv3_in = devs::sumdv3_in;
    using sumdv3_out = devs::sumdv3_out;

    // std::integral_constant<uint8_t, sizeof(Relay<sbus>)>::_;
    static_assert(sizeof(Relay<sbus>) == sizeof(Relay<relay>));
    static_assert(sizeof(Relay<sbus>) == sizeof(Relay<pulse_in>));
    static_assert(sizeof(Relay<sbus>) == sizeof(Relay<ibus_in>));
    static_assert(sizeof(Relay<sbus>) == sizeof(Relay<sbus_in>));
    static_assert(sizeof(Relay<sbus>) == sizeof(Relay<sumdv3_in>));

    static inline void set(const uint8_t r) {
        IO::outl<debug>("# relay ", r);
        switch(r) {
        case 0: //sbus
        {
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<sbus>>();
            Relay<sbus>* const rptr = static_cast<Relay<sbus>*>(mRelay.get());
            rptr->activateSBus2(false);
        }
            break;
        case 1: // crsf
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<relay>>();
            break;
        case 2: // sbus2
        {
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<sbus>>();
            Relay<sbus>* const rptr = static_cast<Relay<sbus>*>(mRelay.get());
            rptr->activateSBus2(true);
        }
            break;
        case 3: // cppm/N
        {
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<pulse_in>>();
            Relay<pulse_in>* const rptr = static_cast<Relay<pulse_in>*>(mRelay.get());
            rptr->positive(false);
        }
            break;
        case 4: // cppm/P
            mRelay = nullptr;
        {
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<pulse_in>>();
            Relay<pulse_in>* const rptr = static_cast<Relay<pulse_in>*>(mRelay.get());
            rptr->positive(true);
        }
            break;
        case 5: // combined pwm/P
            mRelay = nullptr;
        {
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<pulse_in>>();
            Relay<pulse_in>* const rptr = static_cast<Relay<pulse_in>*>(mRelay.get());
            rptr->positive(true, false);
        }
            break;
        case 6: // ibus
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<ibus_in>>();
            break;
        case 7: // sbus
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<sbus_in>>();
            break;
        case 8: // sumdv3-in
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<sumdv3_in>>();
            break;
        case 9: // sumdv3-out
            mRelay = nullptr;
            mRelay = std::make_unique<Relay<sumdv3_out>>();
            break;
        default:
            mRelay = nullptr;
            break;
        }
    }
    static inline void setChannel(const uint8_t ch, const uint16_t v) {
        if (mRelay) {
            mRelay->setChannel(ch, v);
        }
    }
    static inline uint16_t value(const uint8_t ch) {
        if (mRelay) {
            return mRelay->value(ch);
        }
        else {
            return 992;
        }
    }
#ifdef USE_EXTRA_FORWARDS
    static inline constexpr void forwardPacket(const std::byte type, const std::array<uint8_t, 64>& data, const uint16_t length) {
        if (mRelay) {
            mRelay->forwardPacket(type, data, length);
        }
    }
    static inline void command(const std::array<uint8_t, 64>& data, const uint16_t length){
        if (mRelay) {
            mRelay->command(data, length);
        }
    }
    static inline void ping() {
        if (mRelay) {
            mRelay->ping();
        }
    }
#else
    static inline constexpr void forwardPacket(const auto data, const uint16_t length) {
        if (mRelay) {
            mRelay->forwardPacket(data, length);
        }
    }
#endif
    static inline void update() {
        if (mRelay) {
            mRelay->update();
        }
    }
    static inline void periodic() {
        if (mRelay) {
            mRelay->periodic();
        }
    }
    static inline void ratePeriodic() {
        if (mRelay) {
            mRelay->ratePeriodic();
        }
    }
    private:
    static inline std::unique_ptr<IRelay> mRelay{};
};
