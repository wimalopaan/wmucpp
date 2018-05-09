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

#include <iostream>
#include <vector>
#include <list>
#include <memory>

struct elem_t {
    char a[10000]; // stellvertretend für einen etwas größere Datensatz,
    // hier vereinfscht als Array dargestellt
};

template<size_t Size = 10000>
class A {
public:
    A() = default;
    A(const A&) = delete;
    A(A&&) = default;
    void swap(A& o) {
        using std::swap;
        swap(mData, o.mData);
    }
private:
    std::unique_ptr<uint8_t[]> mData = std::make_unique<uint8_t[]>(Size);
};

#ifdef USE_LIST
std::list<A<>> data;
#else
std::vector<std::string> data;
#endif

int main() {
    for(int i=0;; i++) {
        try {
            std::string elem;
            elem.reserve(10000);
            data.push_back(std::move(elem));
        }
        catch (std::bad_alloc)
        {
            std::cout << i << " sucessful push_backs\n";
            break;
        }
    }
}