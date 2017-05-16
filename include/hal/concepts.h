/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

namespace HAL {
    
template<typename P>
concept bool StaticPeriodic() {
    return requires(P p) {
        P::periodic();
    };
}

template<typename P>
concept bool CallableObject() {
    return requires(P p) {
        p();
    };
}

template<typename EH, typename EV>
concept bool EventHandlerGroup() {
    return requires(EH eh, EV ev) {
        EH::process(ev);
    };
}

// note: produces internal compiler error (s.a. EventHandler)
template<typename EH>
concept bool EventHandler() {
    return requires(EH eh) {
        EH::eventType;
        EH::process(uint8_t(0));
    };
}

}