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

constexpr int size = 31; // wesentlich anders als bei 32

typedef struct {
  int a[size];
  int b;
} X;

int test1 (volatile X* x) {
  return x->b + x->a[0];
}

int test2 (volatile int* array, volatile int* b) {
  return *b + array[0];
}

volatile X x;

volatile int a[size];
volatile int b;

int main() {
    test1(&x);
    test2(a, &b);    
}
