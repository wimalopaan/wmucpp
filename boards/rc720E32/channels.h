#pragma once

template<typename Polars, typename Servos, typename ESCS, typename Relays, typename Auxes, typename SBusXX, typename Telemetry, typename Storage>
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

    static inline void update() {
        p1::update();
        p2::update();

        escs::set(0, ampToSbusValue(p1::amp()));
        escs::set(1, ampToSbusValue(p2::amp()));

        servos::update(0);
        servos::update(1);

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

        telemetry::current(0, escs::current(0));
        telemetry::rpm(0, escs::rpm(0));

        telemetry::current(1, escs::current(1));
        telemetry::rpm(1, escs::rpm(1));

        relays::update();
        if (eeprom.inject) {
            relays::setChannel(eeprom.amp1_ch, ampToSbusValue(p1::amp()));
            relays::setChannel(eeprom.phi1_ch, phiToSbusValue(p1::phi()));
            relays::setChannel(eeprom.amp2_ch, ampToSbusValue(p2::amp()));
            relays::setChannel(eeprom.phi2_ch, phiToSbusValue(p2::phi()));
        }
        auxes::update();

        telemetry::template phi<0>(p1::phi());
        telemetry::template amp<0>(p1::amp());

        telemetry::template phi<1>(p2::phi());
        telemetry::template amp<1>(p2::amp());
    }

    private:

    static inline uint16_t phiToSbusValue(const uint16_t phi) {
        if (phi < 4096) {
            return (phi * 1640) / 4096 + 172;
        }
        return 0;
    }
    static inline uint16_t ampToSbusValue(const uint16_t amp) {
        if (amp <= 820) {
            return amp + 992;
        }
        return 1812;
    }
    // static inline uint8_t mUpdateCounter = 0;
};

