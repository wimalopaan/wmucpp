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

#include "std/array.h"
#include "std/types.h"
#include "util/bits.h"

template<uint8_t K, typename ValueType = uint16_t>
class ExponentialFilter {
public:
    typedef typename Util::enclosingType<ValueType>::type enc_type;
    typedef ValueType value_type;
    
    value_type operator()(value_type v) {
        enc_type m = v;
        enc_type r = (K * m + (256 - K) * lastValue) / 256; 
        lastValue = r;
        return r;
    }
    
    void clear() {
        lastValue = 0;
    }

private:
    enc_type lastValue = 0;
};
