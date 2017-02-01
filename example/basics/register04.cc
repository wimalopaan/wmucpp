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

#define NDEBUG

#include <stdint.h>
#include "std/traits.h"
#include "util/util.h"

template<typename Component, typename BitType, typename ValueType = uint8_t>
struct __attribute__((packed)) ControlRegister final {
    typedef Component component_type;
    typedef ValueType value_type;    
    typedef BitType bit_type;
    
    template<typename... Flags>
    void inline set(Flags... f) {
        static_assert((std::is_same<bit_type, Flags>::value && ... && true), "wrong flag type");
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        value_type v = (static_cast<value_type>(f) | ... | 0);
        assert(Util::numberOfOnes(v) == sizeof... (Flags));
        hwRegister = v;
    }
    template<BitType... Flags>
    void inline add() {
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        constexpr auto v = (static_cast<value_type>(Flags) | ... | 0);
        constexpr auto n = Util::numberOfOnes(v);        
        static_assert(n == sizeof... (Flags), "use different flags");
        hwRegister |= v;
    }
private:    
    volatile value_type hwRegister;
};

struct MCU final {
    struct CompA {
        enum class Flags1 {
            f1 = (1 << 0),
            f2 = (1 << 1)
        };
        enum class Flags2 {
            f1 = (1 << 1),
            f2 = (1 << 2)
        };
        
        // unidefined Behaviour
        
        union {
            ControlRegister<CompA, Flags1> part1;
            ControlRegister<CompA, Flags2> part2;
        } __attribute__ ((packed));
    };    
};

int main() {
    const auto c = reinterpret_cast<MCU::CompA*>(0x28);

    using F1 = MCU::CompA::Flags1;
    c->part1.set(F1::f1, F1::f1);

    using F2 = MCU::CompA::Flags2;
    c->part2.set(F2::f1);
    
    c->part2.add<F2::f2>();
    
    while(true) {}
}

