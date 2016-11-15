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

#include <array>
#include <string>
#include <iostream>

struct Digital {
    static constexpr const char name[] = "D";
};
struct Analog {
    static constexpr const char name[] = "A";
};

template<typename Type>
class Actor
{
public:
    Actor(const std::string& baseName = Type::name) : name(baseName + std::to_string(numberOfActors)), 
        number(numberOfActors++) {}
    static size_t numberOfActors;
    std::string name;
    const size_t number;
};
template<typename Type>
size_t Actor<Type>::numberOfActors = 0;

template<typename Type>
std::ostream& operator<<(std::ostream& o, const Actor<Type>& a) {
    return o << a.name;
}

template<typename Type>
class Actor2
{
public:
    Actor2(int id) : name(std::string{Type::name} + std::to_string(id)), 
        mId(id) {}
    std::string name;
    const size_t mId;
};

template<typename Type>
std::ostream& operator<<(std::ostream& o, const Actor2<Type>& a) {
    return o << a.name;
}

Actor2<Digital> operator"" _da(unsigned long long v) {
    return Actor2<Digital>(v);
}

Actor2<Analog> operator"" _aa(unsigned long long v) {
    return Actor2<Analog>(v);
}

int main()
{
    std::array<Actor2<Digital>, 3> actors3 = {{1_da, 3_da, 2_da}};
    std::array<Actor2<Analog>, 3> actors4 = {{1_aa, 3_aa, 2_aa}};
    
    std::array<Actor<Digital>, 3> actors1;
    std::array<Actor<Analog>, 3> actors2;
    
    for(const auto& a: actors1) {
        std::cout << a << std::endl;
    }
    for(const auto& a: actors2) {
        std::cout << a << std::endl;
    }


}
