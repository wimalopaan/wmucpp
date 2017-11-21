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

void f1(const char* p) {
    volatile char first = p[0];
    (void) first;
}

void f2(const char (*p)[10]) {
    volatile char first = (*p)[0];
    (void) first;
}

int main() {
    const char s1[] = "abc";
    const char s2[10] = "abc";
    
    f1(s1);
    f2(s1); // decay

    f1(s2);
    f2(s2); // decay

    f1(&s2); // incompatible
    f2(&s2); // ok!!!
    
    const char* cs1 = s1;
    const char* cs2 = s2; // decay

    f1(cs1); 
    f2(cs2); // decay
}
