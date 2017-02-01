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

enum class A {
    a = 1 << 0, 
    b = 1 << 1, 
    c = 1 << 2
};

template<typename T, T... Values>
struct static_container final {};

template<typename T, T... Ts>
constexpr auto make_static_container(T...) {
    return static_container<T, Ts...>{};
}

template<A... Flags>
void inline set() {
}

template<typename F, F... FF>
void inline set(static_container<F, FF...>) {
    set<FF...>();
}

int main() {
    constexpr static_container<A, A::a, A::b> sc1{};
    constexpr auto sc2 = make_static_container(A::a, A::c);
 
    set(sc1);    
    set(sc2);    
}
