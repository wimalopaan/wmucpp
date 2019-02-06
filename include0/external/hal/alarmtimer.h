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

#include <cstddef>
#include <optional>
#include <chrono>

#include <etl/fixedvector.h>

namespace External {
    namespace Hal {
        enum class AlarmFlags : uint8_t {
            NoTimer =    0,
            Periodic =   1 << 0,
            OneShot  =   1 << 1,
            AutoDelete = 1 << 2,
            Disabled   = 1 << 3
        };
    }
}

namespace std {
    template<> struct enable_bitmask_operators<External::Hal::AlarmFlags> : true_type {};
}

namespace External {
    namespace Hal {
        using namespace std::chrono;
        using namespace etl;
        
        template<typename SystemClock, auto MaxTimers = 8>
        class AlarmTimer final {
            struct TimerImpl final {
                constexpr inline explicit operator bool() const {
                    return flags != AlarmFlags::NoTimer;
                }
                uint16_t ticks = 0;
                uint16_t ticksLeft = 0;
                AlarmFlags flags = AlarmFlags::NoTimer;
            };
            using container_type = etl::FixedVector<TimerImpl, MaxTimers>;
            using size_type = typename container_type::size_type;

            inline static container_type mTimers;
        public:
            using index_type = etl::uint_NaN<uint8_t>;
            
            static_assert(MaxTimers < std::numeric_limits<index_type>::max());
            
            AlarmTimer() = delete;
            
            static constexpr auto intervall = SystemClock::intervall;
            
            inline static index_type create(milliseconds millis, AlarmFlags flags){
                if (auto index = mTimers.insert({millis / intervall, millis  / intervall, flags})) {
                    assert(*index <= std::numeric_limits<index_type>::max());
                    return index_type{*index};
                }
                return {};
            }
            inline static index_type create(seconds secs, AlarmFlags flags){
                return create(static_cast<milliseconds>(secs), flags);
            }
            inline static void remove(index_type id) {
                if (id) {
                    mTimers.removeAt(*id);
                }
            }
            inline static void start() {}
            
            inline static void start(index_type id) {
                if (id) {
                    mTimers[*id].flags &= ~AlarmFlags::Disabled;
                    mTimers[*id].ticksLeft = mTimers[*id].ticks;
                }
            }
            
            inline static void stop(index_type id) {
                if (id) {
                    mTimers[*id].flags |= AlarmFlags::Disabled;
                }
            }
            
            inline static bool isActive(index_type id) {
                if (id) {
                    return !isset((mTimers[*id].flags & AlarmFlags::Disabled));
                }
                return false;
            }
            
            template<typename Callable>
            inline static void periodic(const Callable& f) {
                using namespace std::literals::chrono;
                for(size_type i = 0; i < mTimers.capacity; ++i) {
                    if (auto& t = mTimers[i]) {
                        if ((--t.ticksLeft == 0) && !isset(t.flags & AlarmFlags::Disabled)) {
                            f(index_type{i});
                            if (isset(t.flags & AlarmFlags::Periodic)) {
                                t.ticksLeft = t.ticks;
                            }
                            if (isset(t.flags & AlarmFlags::OneShot)) {
                                t.flags |= AlarmFlags::Disabled;
                            }
                            if (isset(t.flags & AlarmFlags::AutoDelete)) {
                                t.flags = AlarmFlags::NoTimer;
                            }
                        }
                    }
                }
            }
        };
    }
}

