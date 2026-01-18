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

#include "meta.h"

namespace detail {
    template<typename T>
    struct hasInit : public std::false_type {};
    template<typename T>
    requires(requires(){T::init();})
    struct hasInit<T> : public std::true_type {};

    template<typename T>
    struct hasPeriodic : public std::false_type {};
    template<typename T>
    requires(requires(){T::periodic();})
    struct hasPeriodic<T> : public std::true_type {};

    template<typename T>
    struct hasRatePeriodic : public std::false_type {};
    template<typename T>
    requires(requires(){T::ratePeriodic();})
    struct hasRatePeriodic<T> : public std::true_type {};
}

template<typename... Comp>
struct StandardComponents {
    using clist = Meta::unique<Meta::filter<Meta::nonVoid, Meta::List<Comp...>>>;
    using initList = Meta::filter<detail::hasInit, clist>;
    using periodicList = Meta::filter<detail::hasPeriodic, clist>;
    using ratePeriodicList = Meta::filter<detail::hasRatePeriodic, clist>;

    static inline void init() {
        []<auto... II>(std::index_sequence<II...>){
            (Meta::nth_element<II, initList>::init(), ...);
        }(std::make_index_sequence<Meta::size_v<initList>>{});
    }
    static inline void periodic() {
        []<auto... II>(std::index_sequence<II...>){
            (Meta::nth_element<II, periodicList>::periodic(), ...);
        }(std::make_index_sequence<Meta::size_v<periodicList>>{});
    }
    static inline void ratePeriodic() {
        []<auto... II>(std::index_sequence<II...>){
            (Meta::nth_element<II, ratePeriodicList>::ratePeriodic(), ...);
        }(std::make_index_sequence<Meta::size_v<ratePeriodicList>>{});
    }
};

template<typename... Comp>
struct StandardComponents<Meta::List<Comp... >> {
    using clist = Meta::unique<Meta::filter<Meta::nonVoid, Meta::List<Comp...>>>;
    using initList = Meta::filter<detail::hasInit, clist>;
    using periodicList = Meta::filter<detail::hasPeriodic, clist>;
    using ratePeriodicList = Meta::filter<detail::hasRatePeriodic, clist>;

    static inline void init() {
        []<auto... II>(std::index_sequence<II...>){
            (Meta::nth_element<II, initList>::init(), ...);
        }(std::make_index_sequence<Meta::size_v<initList>>{});
    }
    static inline void periodic() {
        []<auto... II>(std::index_sequence<II...>){
            (Meta::nth_element<II, periodicList>::periodic(), ...);
        }(std::make_index_sequence<Meta::size_v<periodicList>>{});
    }
    static inline void ratePeriodic() {
        []<auto... II>(std::index_sequence<II...>){
            (Meta::nth_element<II, ratePeriodicList>::ratePeriodic(), ...);
        }(std::make_index_sequence<Meta::size_v<ratePeriodicList>>{});
    }
};
