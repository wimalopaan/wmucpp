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

#include "atomic.h"

namespace etl {
    template<typename T, typename F = uint32_t>
    struct SlotEvent {
        using type = T;
        using flag_type = F;
        struct OrEvent {
            explicit OrEvent(SlotEvent<T>& e) : mEvent{e}{}
            OrEvent thenOn(const T which, const auto f) {
                if (mEvent.is(which)) {
                    f();
                }
                return OrEvent{mEvent};
            }
            void clear() {
                mEvent.clear();
            }
        private:
            SlotEvent<T>& mEvent;
        };
        struct OrEventVol {
            explicit OrEventVol(volatile SlotEvent<T>& e) : mEvent{e}{}
            OrEventVol thenOn(const T which, const auto f) {
                if (mEvent.is(which)) {
                    f();
                }
                return OrEventVol{mEvent};
            }
            void clear() {
                mEvent.clear();
            }
        private:
            volatile SlotEvent<T>& mEvent;
        };
        // void operator=(const T e) volatile {
        //     const F mask = (1 << uint8_t(e));
        //     mEvent |= mask;
        // }
        void operator=(const T e) {
            const F mask = (1 << uint8_t(e));
            mEvent |= mask;
        }
        OrEvent on(const T which, const auto f) {
            if (is(which)) {
                f();
            }
            return OrEvent{*this};
        }
        // OrEventVol on(const T which, const auto f) volatile {
        //     if (is(which)) {
        //         f();
        //     }
        //     return OrEventVol{*this};
        // }
        bool is(const T which) {
            const F mask = (1 << uint8_t(which));
            if (mEvent & mask) {
                mEvent &= ~mask;
                return true;
            }
            return false;
        }
        // bool is(const T which) volatile {
        //     const F mask = (1 << uint8_t(which));
        //     return Mcu::Arm::Atomic::access([&]{
        //         if (mEvent & mask) {
        //             mEvent &= ~mask;
        //             return true;
        //         }
        //         return false;
        //     });
        // }
        void clear() {
            mEvent = 0;
        }
    private:
        F mEvent{};
    };
}

namespace etl {
    template<typename T>
    struct Event {
        struct OrEvent {
            explicit OrEvent(Event<T>& e) : mEvent{e}{}
            OrEvent thenOn(const T which, const auto f) {
                if (mEvent.is(which)) {
                    f();
                }
                return OrEvent{mEvent};
            }
        private:
            Event<T>& mEvent;
        };
        struct OrEventVol {
            explicit OrEventVol(volatile Event<T>& e) : mEvent{e}{}
            OrEvent thenOn(const T which, const auto f) {
                if (mEvent.is(which)) {
                    f();
                }
                return OrEvent{mEvent};
            }
        private:
            volatile Event<T>& mEvent;
        };

        void operator=(const T e) volatile {
            mEvent = e;
        }
        void operator=(const T e) {
            mEvent = e;
        }
        OrEvent on(const T which, const auto f) {
            if (is(which)) {
                f();
            }
            return OrEvent{*this};
        }
        OrEventVol on(const T which, const auto f) volatile {
            if (is(which)) {
                f();
            }
            return OrEventVol{*this};
        }
        bool is(const T which) {
            if (mEvent == which) {
                mEvent = T::None;
                return true;
            }
            return false;
        }
        bool is(const T which) volatile {
            if (mEvent == which) {
                mEvent = T::None;
                return true;
            }
            return false;
        }
        explicit operator bool() const {
            return mEvent != T::None;
        }
        private:
        T mEvent = T::None;
    };
}



