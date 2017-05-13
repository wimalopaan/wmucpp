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

#include "mcu/avr8.h"
#include "mcu/ports.h"

inline void* operator new(unsigned int, void* ptr) {
  return ptr;
}

inline void operator delete(void*, unsigned int) noexcept {
  return;
}
void inline operator delete(void*) noexcept {
  return;
}

int main() {
    AVR::getBaseAddr<DefaultMcuType::PortRegister, AVR::B>()->out = 0_B;  
    
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

    PortB::set<0_B>();
    PortB::set(0_B);

    DefaultMcuType::PortRegister* p = new(reinterpret_cast<void*>(0x23)) DefaultMcuType::PortRegister;
    p->out = 0_B;
    
    delete p;
    
    while(true) {}
}
