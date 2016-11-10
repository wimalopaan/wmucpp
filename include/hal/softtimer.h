/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "config.h"
#include "mcu/avr/isr.h"
#include "util/bits.h"
#include "container/fixedvector.h"
#include "std/optional.h"
#include "util/disable.h"
#include "hal/event.h"
#include "std/types.h"
#include "std/limits.h"

enum class TimerFlags : uint8_t {
    NoTimer =    0,
    Periodic =   1 << 0,
    OneShot  =   1 << 1,
    AutoDelete = 1 << 2,
    Disabled   = 1 << 3
};

namespace std {
template<>
struct enable_bitmask_operators<TimerFlags>{
    static const bool enable = true;
};
template<>
struct underlying_type<TimerFlags> {
    typedef uint8_t type;
};
}

namespace AVR {
namespace Util {

template<typename T>
struct TimerSetupData final {
    const uint16_t prescaler;
    const T ocr;
};

template<typename MCUTimer, typename T>
constexpr TimerSetupData<T> calculate(const std::hertz& ftimer) {
    using pRow = typename MCUTimer::mcu_timer_type::template PrescalerRow<MCUTimer::number>;
    for(const auto& p : pRow::values) {
        const auto tv = (Config::fMcu / ftimer) / p;
        if (tv < std::numerical_limits<T>::max()) {
            return {p, static_cast<T>(tv)};
        }
    }
    return {0, 0};
}
}
}
template<typename MCUTimer>
class Timer final {
public:
    Timer() = delete;

    static void init() {
        // todo: no structured bindings in c++1z
        // constexpr auto [xx,yy]  = AVR::Util::calculate<systemClock, uint8_t>(1000_Hz);

        constexpr auto t = AVR::Util::calculate<MCUTimer, uint8_t>(Config::Timer::frequency);
        static_assert(t.prescaler != 0, "falscher wert für p");

        MCUTimer::template prescale<t.prescaler>();
        MCUTimer::ocra(t.ocr);
        MCUTimer::mode(AVR::TimerMode::CTC);
        MCUTimer::start();
    }

    static std::optional<uint7_t> create(std::milliseconds millis, TimerFlags flags){
        Scoped<DisbaleInterrupt> di;
        if (auto index = timers().insert({millis / Config::Timer::resolution, millis  / Config::Timer::resolution, flags})) {
            assert(*index <= std::numerical_limits<uint7_t>::max());
            return uint7_t{*index};
        }
        return {};
    }
    static std::optional<uint7_t> create(std::seconds secs, TimerFlags flags){
        return create(static_cast<std::milliseconds>(secs), flags);
    }

    static void remove(uint8_t id) {
        Scoped<DisbaleInterrupt> di;
        timers().removeAt(id);
    }

    static void periodic() {
        using namespace std::literals::chrono;
        for(uint8_t i = 0; i < timers().capacity; ++i) {
            if (auto& t = timers()[i]) {
                if ((--t.ticksLeft == 0) && !isset(t.flags & TimerFlags::Disabled)) {
                    EventManager::enqueue({EventType::Timer, i});
                    if (isset(t.flags & TimerFlags::Periodic)) {
                        t.ticksLeft = t.ticks;
                    }
                    if (isset(t.flags & TimerFlags::OneShot)) {
                        t.flags |= TimerFlags::Disabled;
                    }
                    if (isset(t.flags & TimerFlags::AutoDelete)) {
                        t.flags = TimerFlags::NoTimer;
                    }
                }
            }
        }
    }
private:
    struct TimerImpl final {
        inline explicit operator bool() {
            return flags != TimerFlags::NoTimer;
        }
        uint16_t ticks = 0;
        uint16_t ticksLeft = 0;
        TimerFlags flags = TimerFlags::NoTimer;
    };

    // header only: to avoid static data member
    static std::FixedVector<TimerImpl, Config::Timer::NumberOfTimers>& timers() {
        static std::FixedVector<TimerImpl, Config::Timer::NumberOfTimers> timers;
        return timers;
    }
};
