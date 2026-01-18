/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

struct PortDevice {
    virtual void periodic() = 0;
    virtual void ratePeriodic() = 0;
    virtual ~PortDevice() {}
};
struct IAux : PortDevice {
    // virtual void update() = 0;
};
template<typename A>
struct Aux: IAux {
    Aux() {
        A::init();
    }
    ~Aux() {
        A::reset();
    }
    // virtual void update() {
    //     A::update();
    // }
    virtual void periodic() {
        if constexpr(requires(){A::periodic();}) {
            A::periodic();
        }
    }
    virtual void ratePeriodic() {
        if constexpr(requires(){A::ratePeriodic();}) {
            A::ratePeriodic();
        }
    }
};

struct ISm : PortDevice {
    // virtual void update() = 0;
};
template<typename S>
struct Sm: ISm{
    Sm() {
        S::init();
    }
    ~Sm() {
        S::reset();
    }
    // virtual void update() {
    //     S::update();
    // }
    virtual void periodic() {
        if constexpr(requires(){S::periodic();}) {
            S::periodic();
        }
    }
    virtual void ratePeriodic() {
        if constexpr(requires(){S::ratePeriodic();}) {
            S::ratePeriodic();
        }
    }
};

struct IEnc : PortDevice {
    // virtual void update() = 0;
};
template<typename E>
struct Enc: IEnc {
    Enc() {
        E::init();
    }
    ~Enc() {
        E::reset();
    }
    virtual void periodic() {
        if constexpr(requires(){E::periodic();}) {
            E::periodic();
        }
    }
    virtual void ratePeriodic() {
        if constexpr(requires(){E::ratePeriodic();}) {
            E::ratePeriodic();
        }
    }
};

struct IBus : PortDevice {
    virtual void forwardPacket(volatile uint8_t* data, const uint16_t length) = 0;
};
template<typename B>
struct Bus: IBus {
    Bus() {
        B::init();
    }
    ~Bus() {
        B::reset();
    }
    virtual void periodic() {
        if constexpr(requires(){B::periodic();}) {
            B::periodic();
        }
    }
    virtual void ratePeriodic() {
        if constexpr(requires(){B::ratePeriodic();}) {
            B::ratePeriodic();
        }
    }
    virtual void forwardPacket(volatile uint8_t* data, const uint16_t length) {
        if constexpr(requires(){B::forwardPacket(data, length);}) {
            B::forwardPacket(data, length);
        }
    }
};
