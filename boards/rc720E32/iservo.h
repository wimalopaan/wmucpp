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

struct IDevice {
    virtual void periodic() = 0;
    virtual void ratePeriodic() = 0;
    virtual ~IDevice() {}
};
struct IServo : IDevice {
    virtual std::pair<uint8_t, uint8_t> fwVersion() = 0;
    virtual std::pair<uint8_t, uint8_t> hwVersion() = 0;
    virtual int8_t turns() = 0;
    virtual uint16_t actualPos() = 0;
    virtual void speed(uint16_t) = 0;
    virtual void offset(uint16_t) = 0;
    virtual void zero() = 0;
    virtual void update() = 0;
    virtual void set(uint16_t) = 0;
    virtual ~IServo(){}
};
struct IEsc : IDevice {
    virtual std::pair<uint8_t, uint8_t> fwVersion() = 0;
    virtual std::pair<uint8_t, uint8_t> hwVersion() = 0;
    virtual uint16_t current() = 0;
    virtual uint16_t rpm() = 0;
    virtual uint16_t voltage() = 0;
    virtual uint16_t temp() = 0;
    virtual void set(uint16_t) = 0;
    virtual void update() = 0;
    virtual ~IEsc(){}
};
struct IRelay : IDevice {
#ifdef USE_EXTRA_FORWARDS
    virtual void ping() = 0;
    virtual void forwardPacket(const std::byte type, const std::array<uint8_t, 64>& data, const uint16_t length) = 0;
    virtual void command(const std::array<uint8_t, 64>& data, const uint16_t length) = 0;
#else
    virtual void forwardPacket(const volatile uint8_t* data, const uint16_t length) = 0;
#endif
    virtual void setChannel(const uint8_t ch, const uint16_t v) = 0;
    virtual uint16_t value(const uint8_t ch) = 0;
    virtual void update() = 0;
    virtual ~IRelay(){}
};

template<typename T>
concept hasActivateSBus2 = requires(T) {
    T::activateSBus2(true);
};
template<typename T>
concept hasForward = requires(T) {
    T::forwardPacket(nullptr, 0);
};
template<typename T>
concept hasPing = requires(T) {
    T::ping();
};
template<typename T>
concept hasSet = requires(T) {
    T::set(0, 0);
};
template<typename T>
concept hasValue = requires(T) {
    T::value(0);
};
template<typename T>
concept hasPositive = requires(T) {
    T::positive(true);
};
// template<typename T>
// concept hasForwardPacket = requires(T) {
//     T::ping();
// };

template<typename R>
struct Relay : IRelay {
    Relay() {
        R::init();
    }
    ~Relay() {
        R::reset();
    }
    virtual void positive(const bool pos, const bool per = true) {
        if constexpr(hasPositive<R>) {
            R::positive(pos, per);
        }
    }
    virtual void activateSBus2(const bool b) {
        if constexpr(hasActivateSBus2<R>) {
            R::activateSBus2(b);
        }
    }
#ifdef USE_EXTRA_FORWARDS
    virtual void forwardPacket(const std::byte type, const std::array<uint8_t, 64>& data, const uint16_t length) {
        if constexpr(hasPing<R>) {
            R::forwardPacket(type, data, length);
        }
    }
    virtual void ping() {
        if constexpr(hasPing<R>) {
            R::ping();
        }
    }
    virtual void command(const std::array<uint8_t, 64>& data, const uint16_t length) {
        if constexpr(hasPing<R>) {
            R::command(data, length);
        }
    }
#else
    virtual void forwardPacket(const volatile uint8_t* data, const uint16_t length) {
        if constexpr(hasForward<R>) {
            R::forwardPacket(data, length);
        }
    }
#endif
    virtual void setChannel(const uint8_t ch, const uint16_t v) {
        if constexpr(hasSet<R>) {
            R::set(ch, v);
        }
    }
    virtual uint16_t value(const uint8_t ch) {
        if constexpr(hasValue<R>) {
            return R::value(ch);
        }
        else {
            return 992;
        }
    }
    virtual void update() {
        R::update();
    }
    virtual void periodic() {
        R::periodic();
    }
    virtual void ratePeriodic() {
        R::ratePeriodic();
    }
};

template<typename S>
struct Servo : IServo {
    Servo() {
        S::init();
    }
    ~Servo() {
        S::reset();
    }
    virtual std::pair<uint8_t, uint8_t> fwVersion() {
        if constexpr(requires(){S::fwVersion();}) {
            return S::fwVersion();
        }
        else {
            return {};
        }
    }
    virtual std::pair<uint8_t, uint8_t> hwVersion() {
        if constexpr(requires(){S::hwVersion();}) {
            return S::hwVersion();
        }
        else {
            return {};
        }
    }
    virtual int8_t turns() {
        if constexpr(requires(){S::turns();}) {
            return S::turns();
        }
        return 0;
    }
    virtual uint16_t actualPos() {
        if constexpr(requires(){S::actualPos();}) {
            return S::actualPos();
        }
        return 0;
    }
    virtual void speed( const uint16_t s) {
        if constexpr(requires(){S::speed(s);}) {
            S::speed(s);
        }
    }
    virtual void offset( const uint16_t o) {
        if constexpr(requires(){S::offset(o);}) {
            S::offset(o);
        }
    }
    virtual void zero() {
        if constexpr(requires(){S::zero();}) {
            S::zero();
        }
    }
    virtual void set(const uint16_t s) {
        if constexpr(requires(){S::set(s);}) {
            S::set(s);
        }
    }
    virtual void update() {
        if constexpr(requires(){S::update();}) {
            S::update();
        }
    }
    virtual void periodic() {
        S::periodic();
    }
    virtual void ratePeriodic() {
        S::ratePeriodic();
    }
};

template<typename E>
struct Esc: IEsc {
    Esc() {
        E::init();
    }
    ~Esc() {
        E::reset();
    }
    virtual std::pair<uint8_t, uint8_t> fwVersion() {
        if constexpr(requires(){E::fwVersion();}) {
            return E::fwVersion();
        }
        else {
            return {};
        }
    }
    virtual std::pair<uint8_t, uint8_t> hwVersion() {
        if constexpr(requires(){E::hwVersion();}) {
            return E::hwVersion();
        }
        else {
            return {};
        }
    }
    virtual uint16_t current() {
        if constexpr(requires(){E::current();}) {
            return E::current();
        }
        else {
            return 0;
        }
    }
    virtual uint16_t rpm() {
        if constexpr(requires(){E::rpm();}) {
            return E::rpm();
        }
        else {
            return 0;
        }
    }
    virtual uint16_t voltage() {
        if constexpr(requires(){E::voltage();}) {
            return E::voltage();
        }
        else {
            return 0;
        }
    }
    virtual uint16_t temp() {
        if constexpr(requires(){E::temp();}) {
            return E::temp();
        }
        else {
            return 0;
        }
    }
    virtual void set(const uint16_t s) {
        if constexpr(requires(){E::set(s);}) {
            return E::set(s);
        }
    }
    virtual void update() {
        E::update();
    }
    virtual void periodic() {
        E::periodic();
    }
    virtual void ratePeriodic() {
        E::ratePeriodic();
    }
};

template<typename D>
struct Device : IDevice {
    Device() {
        D::init();
    }
    ~Device() {
        D::reset();
    }
    virtual void periodic() {
        D::periodic();
    }
    virtual void ratePeriodic() {
        D::ratePeriodic();
    }
};

