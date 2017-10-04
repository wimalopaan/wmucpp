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

#include <iostream>

template<typename D>
class B {
    uint8_t x;
};

class A : public B<A> {
    char a[10];
};

template<typename DataType, uint16_t Offset = 0>
class EEProm;

template<typename T>
class EEPromBase {
    EEPromBase() = delete;
    struct Flags {
        uint8_t changed : 1, saving : 1, timeoutExpired : 1;
    } __attribute__((packed));
    friend class EEProm<T>;
    static void startTimeout() {
        mFlags.timeoutExpired = false;
    }
public:
    inline static bool changed() {
        return mFlags.changed || mFlags.saving;
    }
    inline static void change() {
        mFlags.changed = true;
    }
    inline static bool timeout() {
        return mFlags.timeoutExpired;
    }
    inline static void saveStart() {
        mFlags.saving  = true;
        mFlags.changed = false;
    }
    inline static void saveEnd() {
        mFlags.saving = false;
    }
private:
    inline static Flags mFlags = {false, false, false};
};
class EEPromData : public EEPromBase<EEPromData>{
public:
private:
  char  mText[10];
};

int main() {
    std::cout << "Size: " << sizeof(EEPromBase<A>) << '\n';
    std::cout << "Size: " << sizeof(EEPromData) << '\n';
}
