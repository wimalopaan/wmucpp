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

#include "rc/rc_2.h"

template<typename Polars, typename Servos, typename ESCS, typename Relays, typename Auxes,
         typename Telemetry, typename Storage>
struct ChannelCallback {
    using polars = Polars;
    using servos = Servos;
    using escs = ESCS;
    using relays = Relays;
    using auxes = Auxes;

    using telemetry = Telemetry;
    using storage = Storage;
    static inline auto& eeprom = storage::eeprom;

    using p1 = Meta::nth_element<0, polars>;
    using p2 = Meta::nth_element<1, polars>;

    using pa1 = p1::pa;
    using pa2 = p2::pa;
    static_assert(std::is_same_v<pa1, pa2>);
    using pa = pa1;

    static inline void update() {
        if (eeprom.mode == 1) { // Passthru-Mode
            escs::set(0, pa::value(eeprom.channels[0].first));
            escs::set(1, pa::value(eeprom.channels[1].first));

            servos::set(0, pa::value(eeprom.channels[0].second));
            servos::set(1, pa::value(eeprom.channels[1].second));

            telemetry::current(0, escs::current(0));
            telemetry::rpm(0, escs::rpm(0));
            telemetry::voltage(0, escs::voltage(0));
            telemetry::temp(0, escs::temp(0));
            telemetry::current(1, escs::current(1));
            telemetry::rpm(1, escs::rpm(1));
            telemetry::voltage(1, escs::voltage(1));
            telemetry::temp(1, escs::temp(1));

            relays::update();

            for(uint8_t i = 0; i < 16; ++i) {
                relays::setChannel(i, pa::value(i));
                escs::setChannel(0, i, pa::value(i));
                escs::setChannel(1, i, pa::value(i));
            }

            auxes::update();
        }
        else if (eeprom.mode == 0) { // Schottel-Mode
            p1::update();
            p2::update();

            escs::set(0, ampToSbusValue(p1::amp()));
            escs::set(1, ampToSbusValue(p2::amp()));

            servos::update();

            {
                telemetry::actual(0, servos::actualPos(0));
                const int8_t turns = servos::turns(0);
                telemetry::turns(0, turns);
                telemetry::alarm(0, (turns >= 5) || (turns <= -5));
            }
            {
                telemetry::actual(1, servos::actualPos(1));
                const int8_t turns = servos::turns(1);
                telemetry::turns(1, turns);
                telemetry::alarm(1, (turns >= 5) || (turns <= -5));
            }
            telemetry::template phi<0>(p1::phi());
            telemetry::template amp<0>(p1::amp());

            telemetry::template phi<1>(p2::phi());
            telemetry::template amp<1>(p2::amp());

            telemetry::current(0, escs::current(0));
            telemetry::rpm(0, escs::rpm(0));
			telemetry::voltage(0, escs::voltage(0));

            telemetry::current(1, escs::current(1));
            telemetry::rpm(1, escs::rpm(1));
			telemetry::voltage(1, escs::voltage(1));

            relays::update();
            if (eeprom.inject) {
                relays::setChannel(eeprom.amp1_ch, ampToSbusValue(p1::amp()));
                relays::setChannel(eeprom.phi1_ch, phiToSbusValue(p1::phi()));
                relays::setChannel(eeprom.amp2_ch, ampToSbusValue(p2::amp()));
                relays::setChannel(eeprom.phi2_ch, phiToSbusValue(p2::phi()));
            }
            auxes::update();
        }

    }

    private:

    static inline uint16_t phiToSbusValue(const uint16_t phi) {
        if (phi < 4096) {
            return (phi * RC::Protokoll::SBus::V2::amp) / 4095 + RC::Protokoll::SBus::V2::min;
        }
        return RC::Protokoll::SBus::V2::mid;
    }
    static inline uint16_t ampToSbusValue(const uint16_t amp) {
        if (amp <= RC::Protokoll::SBus::V2::span) {
            return amp + RC::Protokoll::SBus::V2::mid;
        }
        return RC::Protokoll::SBus::V2::mid;
    }
};

