#pragma once

#include <etl/converter.h>
#include <etl/fixedpoint.h>

#include <external/solutions/gps.h>
#include <external/solutions/series01/swuart.h>

using rxPin = so3Pin; // watchout for ISR definitions
using txPin = void;

using vtg = External::GPS::VTG;
using rmc = External::GPS::RMC;
using gpsPA = External::GPS::GpsProtocollAdapter<0, vtg, rmc>;

using gpsUsart = External::SoftSerial::Usart<Meta::List<rxPin, void>, Component::Tcd<0>, gpsPA,
                                            AVR::BaudRate<9600>, AVR::ReceiveQueueLength<0>>;

template<typename VTG>
struct SpeedProvider {
#ifdef FS_I6S
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE
#else
    inline static constexpr auto ibus_type = IBus::Type::type::SPEED; // km/h
#endif
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        VTG::speedRaw(data);
        auto ss = etl::StringConverter<etl::FixedPoint<uint16_t, 4>>::parse<1>(data);
        auto s = ss * 100;
        return s.integer();
    }
private:
    static inline etl::StringBuffer<External::GPS::Sentence::DecimalMaxWidth> data;
};

using speedP = SpeedProvider<vtg>;

template<typename Dekoder>
struct PackagesProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::GPS_STATUS; 
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return Dekoder::receivedPackages();
    }
};
template<typename Dekoder>
struct BytesProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::ARMED; 
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return Dekoder::receivedBytes();
    }
};

using packagesR = PackagesProvider<rmc>;
using packagesV = PackagesProvider<vtg>;

using bytesP = BytesProvider<gpsPA>;
