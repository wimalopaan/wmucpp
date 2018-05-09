/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "config.h"
#include "mcu/ports.h"
#include "hal/event.h"

template<typename... Buttons>
class ButtonController final {
    ButtonController() = delete;
public:
    typedef Meta::List<Buttons...> button_list;
    typedef Meta::front<button_list> first_button;
    inline static constexpr bool useEvents = first_button::useEvents;
    static_assert(((useEvents == Buttons::useEvents) && ... && true));
    
    static inline void init() {
        (Buttons::init(), ...); 
    }
    
    template<bool Q = !useEvents>
    static inline 
    typename std::enable_if<Q, void>::type
    periodic(const Util::Callable<uint8_t>& f) {
        (Buttons::sample(f), ...);
    }
    template<bool Q = useEvents>
    static inline
    typename std::enable_if<Q, void>::type
    periodic() {
        (Buttons::sample(), ...);
    }
    static inline void start() {}
};

template<typename BC, typename Int>
struct ButtonControllerIsr : IsrBaseHandler<Int> {
    inline static void isr() {
        BC::periodic([](uint8_t){});
    }
    template<typename B>
    inline static bool isPressed() {
        return B::mData.state;
    }
};

template<uint8_t N, typename Pin, ::Util::NamedFlag UseEvent = UseEvents<true>, uint8_t Thresh = Config::Button::buttonTicksForPressed>
class Button final {
    static_assert(N < 8, "wrong number of buttons");
    static_assert(Thresh <= 127);
    struct Data {
        uint8_t state : 1, count : 7;
    };
    inline static Data mData{0, 0};
public:
    Button() = delete;
    inline static void init() {
        Pin::template dir<AVR::Input>();
        Pin::pullup();
    }
    typedef Pin pin_type;
    typedef typename Pin::port port_type;
    static inline constexpr bool useEvents = UseEvent::value;
private:
    template<bool Q = useEvents>
    static 
    inline typename std::enable_if<Q, void>::type
    sample() {
        if (!Pin::isHigh()) { // pressed (active low)
            if (!mData.state) { // not pressed
                if (++mData.count >= Thresh) {
                    mData.state = 1u;
                    mData.count = 0u;
                    EventManager::enqueue({EventType::ButtonPress, std::byte{N}});
                }
            }
        }
        else { // not pressed
            if (mData.state) {
                mData.state = 0u;
                mData.count = 0u;
            }
        }
    }
    template<bool Q = useEvents>
    static 
    inline typename std::enable_if<!Q, void>::type
    sample(const Util::Callable<uint8_t>& f) {
        if (!Pin::isHigh()) { // pressed (active low)
            if (!mData.state) { // not pressed
                if (++mData.count >= Thresh) {
                    mData.state = 1u;
                    mData.count = 0u;
                    f(N);
                }
            }
        }
        else { // not pressed
            if (mData.state) {
                mData.state = 0u;
                mData.count = 0u;
            }
        }
    }
};

