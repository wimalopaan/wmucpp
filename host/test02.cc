/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <algorithm>
#include <vector>

struct Digital {
    static constexpr const char name[] = "Digital";
    static constexpr const uint16_t baseAddress = 0x100;
};
struct Analog {
    static constexpr const char name[] = "Aanalog";
    static constexpr const uint16_t baseAddress = 0x200;
};

template<typename Type> class Actor;
template<typename Type> std::ostream& operator<<(std::ostream& o, const Actor<Type>& a);

template<typename Type>
class Actor
{
    friend std::ostream& operator<< <>(std::ostream& o, const Actor<Type>& a);
public:
    Actor(int id) : name(std::string{Type::name} + std::to_string(id)), mId(id), address(Type::baseAddress + mId) {}
private:
    std::string name;
    size_t mId;
    uint16_t address;
};

template<typename Type>
std::ostream& operator<<(std::ostream& o, const Actor<Type>& a) {
    return o << a.name << " : " << a.address;
}

Actor<Digital> operator"" _da(unsigned long long v) {
    return Actor<Digital>(v);
}

Actor<Analog> operator"" _aa(unsigned long long v) {
    return Actor<Analog>(v);
}

int main()
{
    std::array<Actor<Digital>, 3> actors1 = {{1_da, 3_da, 2_da}};
    std::array<Actor<Analog>, 3>  actors2 = {{1_aa, 3_aa, 2_aa}};
 
    std::vector<Actor<Digital>> v;
    int startId = 0;
    std::generate_n(std::back_inserter(v), 10, [&](){return Actor<Digital>(startId++);});
    
    for(const auto& a: v) {
        std::cout << a << std::endl;
    }
    
    for(const auto& a: actors1) {
        std::cout << a << std::endl;
    }
    for(const auto& a: actors2) {
        std::cout << a << std::endl;
    }
}
