#pragma once

// used for rpm
// tca clock divisor 1024
// tcb input capture
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using rpm0Position = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using rpm1Position = Portmux::Position<Component::Tcb<1>, Portmux::Default>;


using rpm0 = External::Rpm::RpmFreq<tcaPosition::component_type, rpm0Position::component_type>;
using rpm1 = External::Rpm::RpmFreq<tcaPosition::component_type, rpm1Position::component_type>;

template<typename R>
struct RpmProvider {
    inline static constexpr auto ibus_type = IBus::Type::type::RPM;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        auto rpm = R::value();
        if (rpm) {
            return rpm.value();
        }
        return 0;
    }
};
using rpm0P = RpmProvider<rpm0>;
using rpm1P = RpmProvider<rpm1>;

using evch0 = Event::Channel<0, Event::Generators::Pin<so4Pin>>;
using evch1 = Event::Channel<1, Event::Generators::Pin<q0Pin>>;
using evuser0  = Event::Route<evch0, Event::Users::Tcb<0>>;
using evuser1  = Event::Route<evch1, Event::Users::Tcb<1>>;

//using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1>>;

