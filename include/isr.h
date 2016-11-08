/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#if __has_include(<avr/interrupt.h>)
# include <avr/interrupt.h>

extern "C" {
    void TIMER0_OVF_vect(void);
    void TIMER0_COMPA_vect(void);

    void USART_RX_vect(void);
    void USART_UDRE_vect(void);

    void USART0_RX_vect(void);
    void USART0_UDRE_vect(void);
    void USART1_RX_vect(void);
    void USART1_UDRE_vect(void);

    void SPI_STC_vect(void);

    void TIMER1_COMPA_vect(void);
    void TIMER1_COMPB_vect(void);
    void TIMER1_CAPT_vect(void);

    void TIMER3_COMPA_vect(void);
    void TIMER3_COMPB_vect(void);
    void TIMER3_CAPT_vect(void);

    void PCINT0_vect(void);
    void PCINT1_vect(void);
    void PCINT2_vect(void);
    void PCINT3_vect(void);

}

#endif
