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

#include <stdint.h>
#include "std/limits"
#include "std/array"
#include "util/algorithm.h"

#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

// todo: nicht notwendig

template<typename U, U... Values>
class EnumIterator {
public:
    static_assert(sizeof...(Values) < std::numeric_limits<uint8_t>::max(), "Too much values");
    
    typedef typename std::underlying_type<U>::type UT;
    
    inline static constexpr std::array<UT, sizeof...(Values)> values = {static_cast<UT>(Values)...};
    
    inline static constexpr bool unique = [](){
        auto vv = values;
        const auto sortedValues = Util::sort(vv);
        for(size_t i = 1; i < sortedValues.size; ++i) {
            if (sortedValues[i - 1] == sortedValues[i]) {
                return false;
            }
        }
        return true;
    }();
    static_assert(unique, "Values are not unique");
public:
    constexpr typename std::underlying_type<U>::type size() const {
        return sizeof...(Values);
    }
private:
};

int main() {
    enum class X : uint8_t {x1, x2, x3};
    
    EnumIterator<X, X::x1, X::x2, X::x3> oe;

    std::outl<terminal>(oe.size());
    
    for(auto v : oe.values) {
        std::outl<terminal>(v);
    }
    
    while(true) {}
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif
