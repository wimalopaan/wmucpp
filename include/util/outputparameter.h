/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

namespace Util
{
    template <typename T>
    class output_parameter
    {
    public:
        using parameter_type = T;

        explicit output_parameter(T& obj) : parameter(obj) {}

        output_parameter& operator=(const output_parameter&) = delete;
        output_parameter& operator=(output_parameter&&) = delete;

        operator T&() {
            return parameter;
        }
        
    private:
        T& parameter;
    };

    template <typename T>
    output_parameter<T> out(T& obj) {
        return output_parameter<T>(obj);
    }
}
