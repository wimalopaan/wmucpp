/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

.global ascale
ascale:
  sub  r20,r22
  sub  r24,r22
  ldi  r25,100
  mul  r24,r25
  movw r30,r0
  ldi  r25,0x40
  ldi  r24,0
loop:
  mov  r21,r24
  or   r21,r25
  mul  r21,r20
  movw r22,r0
  cp   r30,r22
  cpc  r31,r23
  brlo skip
  mov  r24,r21
  breq leave
skip:
  lsr  r25
  brne loop
leave:
  clr  r1
  ret
  