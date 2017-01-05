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

#pragma once

#include <stdint.h>
#include "mcu/ports.h"
#include "mcu/avr/delay.h"

template<typename Data, typename RS, typename RW, typename E, uint8_t Lines = 2, uint8_t Columns = 16>
class LcdHD44780 final {
    LcdHD44780() = delete;
public:
    static void init() {
        Data::template dir<AVR::Output>();
        RS::template dir<AVR::Output>();
        RW::template dir<AVR::Output>();
        E::template dir<AVR::Output>();
        Data::allOff();
        RS::off();
        RW::off();
        E::off();
        Util::delay(16_ms);
        
        Data::set(0x03);
        toggleE();
        Util::delay(5_ms);
        Data::set(0x03);
        toggleE();
        Util::delay(1_ms);
        Data::set(0x03);
        toggleE();
        Util::delay(1_ms);
        
        Data::set(0x02);
        toggleE();
        Util::delay(1_ms);
    }
private:
    static void write(uint8_t) {
        
    }
    
    static void toggleE() {
        E::on();
        Util::delay(20_us);
        E::off();
    }
};

//#define LCD_LINES           2     /**< number of visible lines of the display */
//#define LCD_DISP_LENGTH    16     /**< visibles characters per line of the display */
//#define LCD_LINE_LENGTH  0x40     /**< internal line length of the display    */
//#define LCD_START_LINE1  0x00     /**< DDRAM address of first char of line 1 */
//#define LCD_START_LINE2  0x40     /**< DDRAM address of first char of line 2 */
//#define LCD_START_LINE3  0x14     /**< DDRAM address of first char of line 3 */
//#define LCD_START_LINE4  0x54     /**< DDRAM address of first char of line 4 */
//#define LCD_WRAP_LINES      0     /**< 0: no wrap, 1: wrap at end of visibile line */
//#define LCD_PORT         PORTA        /**< port for the LCD lines   */
//#define LCD_DATA0_PORT   LCD_PORT     /**< port for 4bit data bit 0 */
//#define LCD_DATA1_PORT   LCD_PORT     /**< port for 4bit data bit 1 */
//#define LCD_DATA2_PORT   LCD_PORT     /**< port for 4bit data bit 2 */
//#define LCD_DATA3_PORT   LCD_PORT     /**< port for 4bit data bit 3 */
//#define LCD_DATA0_PIN    0            /**< pin for 4bit data bit 0  */
//#define LCD_DATA1_PIN    1            /**< pin for 4bit data bit 1  */
//#define LCD_DATA2_PIN    2            /**< pin for 4bit data bit 2  */
//#define LCD_DATA3_PIN    3            /**< pin for 4bit data bit 3  */
//#define LCD_RS_PORT      LCD_PORT     /**< port for RS line         */
//#define LCD_RS_PIN       4            /**< pin  for RS line         */
//#define LCD_RW_PORT      LCD_PORT     /**< port for RW line         */
//#define LCD_RW_PIN       5            /**< pin  for RW line         */
//#define LCD_E_PORT       LCD_PORT     /**< port for Enable line     */
//#define LCD_E_PIN        6            /**< pin  for Enable line     */


///* instruction register bit positions, see HD44780U data sheet */
//#define LCD_CLR               0      /* DB0: clear display                  */
//#define LCD_HOME              1      /* DB1: return to home position        */
//#define LCD_ENTRY_MODE        2      /* DB2: set entry mode                 */
//#define LCD_ENTRY_INC         1      /*   DB1: 1=increment, 0=decrement     */
//#define LCD_ENTRY_SHIFT       0      /*   DB2: 1=display shift on           */
//#define LCD_ON                3      /* DB3: turn lcd/cursor on             */
//#define LCD_ON_DISPLAY        2      /*   DB2: turn display on              */
//#define LCD_ON_CURSOR         1      /*   DB1: turn cursor on               */
//#define LCD_ON_BLINK          0      /*     DB0: blinking cursor ?          */
//#define LCD_MOVE              4      /* DB4: move cursor/display            */
//#define LCD_MOVE_DISP         3      /*   DB3: move display (0-> cursor) ?  */
//#define LCD_MOVE_RIGHT        2      /*   DB2: move right (0-> left) ?      */
//#define LCD_FUNCTION          5      /* DB5: function set                   */
//#define LCD_FUNCTION_8BIT     4      /*   DB4: set 8BIT mode (0->4BIT mode) */
//#define LCD_FUNCTION_2LINES   3      /*   DB3: two lines (0->one line)      */
//#define LCD_FUNCTION_10DOTS   2      /*   DB2: 5x10 font (0->5x7 font)      */
//#define LCD_CGRAM             6      /* DB6: set CG RAM address             */
//#define LCD_DDRAM             7      /* DB7: set DD RAM address             */
//#define LCD_BUSY              7      /* DB7: LCD is busy                    */

///* set entry mode: display shift on/off, dec/inc cursor move direction */
//#define LCD_ENTRY_DEC            0x04   /* display shift off, dec cursor move dir */
//#define LCD_ENTRY_DEC_SHIFT      0x05   /* display shift on,  dec cursor move dir */
//#define LCD_ENTRY_INC_           0x06   /* display shift off, inc cursor move dir */
//#define LCD_ENTRY_INC_SHIFT      0x07   /* display shift on,  inc cursor move dir */

///* display on/off, cursor on/off, blinking char at cursor position */
//#define LCD_DISP_OFF             0x08   /* display off                            */
//#define LCD_DISP_ON              0x0C   /* display on, cursor off                 */
//#define LCD_DISP_ON_BLINK        0x0D   /* display on, cursor off, blink char     */
//#define LCD_DISP_ON_CURSOR       0x0E   /* display on, cursor on                  */
//#define LCD_DISP_ON_CURSOR_BLINK 0x0F   /* display on, cursor on, blink char      */

///* move cursor/shift display */
//#define LCD_MOVE_CURSOR_LEFT     0x10   /* move cursor left  (decrement)          */
//#define LCD_MOVE_CURSOR_RIGHT    0x14   /* move cursor right (increment)          */
//#define LCD_MOVE_DISP_LEFT       0x18   /* shift display left                     */
//#define LCD_MOVE_DISP_RIGHT      0x1C   /* shift display right                    */

///* function set: set interface data length and number of display lines */
//#define LCD_FUNCTION_4BIT_1LINE  0x20   /* 4-bit interface, single line, 5x7 dots */
//#define LCD_FUNCTION_4BIT_2LINES 0x28   /* 4-bit interface, dual line,   5x7 dots */
//#define LCD_FUNCTION_8BIT_1LINE  0x30   /* 8-bit interface, single line, 5x7 dots */
//#define LCD_FUNCTION_8BIT_2LINES 0x38   /* 8-bit interface, dual line,   5x7 dots */


//#define LCD_MODE_DEFAULT     ((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC) )

//#define lcd_e_delay()   __asm__ __volatile__( "rjmp 1f\n 1:" );   //#define lcd_e_delay() __asm__ __volatile__( "rjmp 1f\n 1: rjmp 2f\n 2:" );
//#define lcd_e_high()    LCD_E_PORT  |=  _BV(LCD_E_PIN);
//#define lcd_e_low()     LCD_E_PORT  &= ~_BV(LCD_E_PIN);
//#define lcd_e_toggle()  toggle_e()
//#define lcd_rw_high()   LCD_RW_PORT |=  _BV(LCD_RW_PIN)
//#define lcd_rw_low()    LCD_RW_PORT &= ~_BV(LCD_RW_PIN)
//#define lcd_rs_high()   LCD_RS_PORT |=  _BV(LCD_RS_PIN)
//#define lcd_rs_low()    LCD_RS_PORT &= ~_BV(LCD_RS_PIN)

//#if LCD_LINES==1
//#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_1LINE 
//#endif

//#if LCD_CONTROLLER_KS0073
//#if LCD_LINES==4

//#define KS0073_EXTENDED_FUNCTION_REGISTER_ON  0x2C   /* |0|010|1100 4-bit mode, extension-bit RE = 1 */
//#define KS0073_EXTENDED_FUNCTION_REGISTER_OFF 0x28   /* |0|010|1000 4-bit mode, extension-bit RE = 0 */
//#define KS0073_4LINES_MODE                    0x09   /* |0|000|1001 4 lines mode */

//#endif
//#endif

//static void toggle_e(void);

//static inline void _delayFourCycles(unsigned int __count)
//{
//    if ( __count == 0 )    
//        __asm__ __volatile__( "rjmp 1f\n 1:" );    // 2 cycles
//    else
//        __asm__ __volatile__ (
//    	    "1: sbiw %0,1" "\n\t"                  
//    	    "brne 1b"                              // 4 cycles/loop
//    	    : "=w" (__count)
//    	    : "0" (__count)
//    	   );
//}

//#define delay(us)  _delayFourCycles( ( ( 1*(XTAL/4000) )*us)/1000 )

///* toggle Enable Pin to initiate write */
//static void toggle_e(void)
//{
//    lcd_e_high();
//    lcd_e_delay();
//    lcd_e_low();
//}
//static void lcd_write(uint8_t data,uint8_t rs) 
//{
//    unsigned char dataBits ;

//    if (rs) {   /* write data        (RS=1, RW=0) */
//       lcd_rs_high();
//    } else {    /* write instruction (RS=0, RW=0) */
//       lcd_rs_low();
//    }
//    lcd_rw_low();

//    if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT )
//      && (LCD_DATA0_PIN == 0) && (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3) )
//    {
//        /* configure data pins as output */
//        DDR(LCD_DATA0_PORT) |= 0x0F;

//        /* output high nibble first */
//        dataBits = LCD_DATA0_PORT & 0xF0;
//        LCD_DATA0_PORT = dataBits |((data>>4)&0x0F);
//        lcd_e_toggle();

//        /* output low nibble */
//        LCD_DATA0_PORT = dataBits | (data&0x0F);
//        lcd_e_toggle();

//        /* all data pins high (inactive) */
//        LCD_DATA0_PORT = dataBits | 0x0F;
//    }
//    else
//    {
//        /* configure data pins as output */
//        DDR(LCD_DATA0_PORT) |= _BV(LCD_DATA0_PIN);
//        DDR(LCD_DATA1_PORT) |= _BV(LCD_DATA1_PIN);
//        DDR(LCD_DATA2_PORT) |= _BV(LCD_DATA2_PIN);
//        DDR(LCD_DATA3_PORT) |= _BV(LCD_DATA3_PIN);
        
//        /* output high nibble first */
//        LCD_DATA3_PORT &= ~_BV(LCD_DATA3_PIN);
//        LCD_DATA2_PORT &= ~_BV(LCD_DATA2_PIN);
//        LCD_DATA1_PORT &= ~_BV(LCD_DATA1_PIN);
//        LCD_DATA0_PORT &= ~_BV(LCD_DATA0_PIN);
//    	if(data & 0x80) LCD_DATA3_PORT |= _BV(LCD_DATA3_PIN);
//    	if(data & 0x40) LCD_DATA2_PORT |= _BV(LCD_DATA2_PIN);
//    	if(data & 0x20) LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
//    	if(data & 0x10) LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);   
//        lcd_e_toggle();
        
//        /* output low nibble */
//        LCD_DATA3_PORT &= ~_BV(LCD_DATA3_PIN);
//        LCD_DATA2_PORT &= ~_BV(LCD_DATA2_PIN);
//        LCD_DATA1_PORT &= ~_BV(LCD_DATA1_PIN);
//        LCD_DATA0_PORT &= ~_BV(LCD_DATA0_PIN);
//    	if(data & 0x08) LCD_DATA3_PORT |= _BV(LCD_DATA3_PIN);
//    	if(data & 0x04) LCD_DATA2_PORT |= _BV(LCD_DATA2_PIN);
//    	if(data & 0x02) LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
//    	if(data & 0x01) LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);
//        lcd_e_toggle();        
        
//        /* all data pins high (inactive) */
//        LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);
//        LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
//        LCD_DATA2_PORT |= _BV(LCD_DATA2_PIN);
//        LCD_DATA3_PORT |= _BV(LCD_DATA3_PIN);
//    }
//}

//static uint8_t lcd_read(uint8_t rs) 
//{
//    uint8_t data;

//    if (rs)
//        lcd_rs_high();                       /* RS=1: read data      */
//    else
//        lcd_rs_low();                        /* RS=0: read busy flag */
//    lcd_rw_high();                           /* RW=1  read mode      */
    
//    if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT )
//      && ( LCD_DATA0_PIN == 0 )&& (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3) )
//    {
//        DDR(LCD_DATA0_PORT) &= 0xF0;         /* configure data pins as input */
        
//        lcd_e_high();
//        lcd_e_delay();        
//        data = PIN(LCD_DATA0_PORT) << 4;     /* read high nibble first */
//        lcd_e_low();
        
//        lcd_e_delay();                       /* Enable 500ns low       */
        
//        lcd_e_high();
//        lcd_e_delay();
//        data |= PIN(LCD_DATA0_PORT)&0x0F;    /* read low nibble        */
//        lcd_e_low();
//    }
//    else
//    {
//        /* configure data pins as input */
//        DDR(LCD_DATA0_PORT) &= ~_BV(LCD_DATA0_PIN);
//        DDR(LCD_DATA1_PORT) &= ~_BV(LCD_DATA1_PIN);
//        DDR(LCD_DATA2_PORT) &= ~_BV(LCD_DATA2_PIN);
//        DDR(LCD_DATA3_PORT) &= ~_BV(LCD_DATA3_PIN);
                
//        /* read high nibble first */
//        lcd_e_high();
//        lcd_e_delay();        
//        data = 0;
//        if ( PIN(LCD_DATA0_PORT) & _BV(LCD_DATA0_PIN) ) data |= 0x10;
//        if ( PIN(LCD_DATA1_PORT) & _BV(LCD_DATA1_PIN) ) data |= 0x20;
//        if ( PIN(LCD_DATA2_PORT) & _BV(LCD_DATA2_PIN) ) data |= 0x40;
//        if ( PIN(LCD_DATA3_PORT) & _BV(LCD_DATA3_PIN) ) data |= 0x80;
//        lcd_e_low();

//        lcd_e_delay();                       /* Enable 500ns low       */
    
//        /* read low nibble */    
//        lcd_e_high();
//        lcd_e_delay();
//        if ( PIN(LCD_DATA0_PORT) & _BV(LCD_DATA0_PIN) ) data |= 0x01;
//        if ( PIN(LCD_DATA1_PORT) & _BV(LCD_DATA1_PIN) ) data |= 0x02;
//        if ( PIN(LCD_DATA2_PORT) & _BV(LCD_DATA2_PIN) ) data |= 0x04;
//        if ( PIN(LCD_DATA3_PORT) & _BV(LCD_DATA3_PIN) ) data |= 0x08;        
//        lcd_e_low();
//    }
//    return data;
//}

//static uint8_t lcd_waitbusy(void)
//{
//    register uint8_t c;
    
//    /* wait until busy flag is cleared */
//    while ( (c=lcd_read(0)) & (1<<LCD_BUSY)) {}
    
//    /* the address counter is updated 4us after the busy flag is cleared */
//    delay(2);

//    /* now read the address counter */
//    return (lcd_read(0));  // return address counter
    
//}

//static inline void lcd_newline(uint8_t pos)
//{
//    register uint8_t addressCounter;


//#if LCD_LINES==1
//    addressCounter = 0;
//#endif
//#if LCD_LINES==2
//    if ( pos < (LCD_START_LINE2) )
//        addressCounter = LCD_START_LINE2;
//    else
//        addressCounter = LCD_START_LINE1;
//#endif
//#if LCD_LINES==4
//#if KS0073_4LINES_MODE
//    if ( pos < LCD_START_LINE2 )
//        addressCounter = LCD_START_LINE2;
//    else if ( (pos >= LCD_START_LINE2) && (pos < LCD_START_LINE3) )
//        addressCounter = LCD_START_LINE3;
//    else if ( (pos >= LCD_START_LINE3) && (pos < LCD_START_LINE4) )
//        addressCounter = LCD_START_LINE4;
//    else 
//        addressCounter = LCD_START_LINE1;
//#else
//    if ( pos < LCD_START_LINE3 )
//        addressCounter = LCD_START_LINE2;
//    else if ( (pos >= LCD_START_LINE2) && (pos < LCD_START_LINE4) )
//        addressCounter = LCD_START_LINE3;
//    else if ( (pos >= LCD_START_LINE3) && (pos < LCD_START_LINE2) )
//        addressCounter = LCD_START_LINE4;
//    else 
//        addressCounter = LCD_START_LINE1;
//#endif
//#endif
//    lcd_command((1<<LCD_DDRAM)+addressCounter);

//}

//void lcd_command(uint8_t cmd)
//{
//    lcd_waitbusy();
//    lcd_write(cmd,0);
//}

//void lcd_data(uint8_t data)
//{
//    lcd_waitbusy();
//    lcd_write(data,1);
//}

//void lcd_gotoxy(uint8_t x, uint8_t y)
//{
//#if LCD_LINES==1
//    lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
//#endif
//#if LCD_LINES==2
//    if ( y==0 ) 
//        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
//    else
//        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
//#endif
//#if LCD_LINES==4
//    if ( y==0 )
//        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
//    else if ( y==1)
//        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
//    else if ( y==2)
//        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE3+x);
//    else /* y==3 */
//        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE4+x);
//#endif

//}
//int lcd_getxy(void)
//{
//    return lcd_waitbusy();
//}
//void lcd_clrscr(void)
//{
//    lcd_command(1<<LCD_CLR);
//}
//void lcd_home(void)
//{
//    lcd_command(1<<LCD_HOME);
//}
//void lcd_putc(char c)
//{
//    uint8_t pos;


//    pos = lcd_waitbusy();   // read busy-flag and address counter
//    if (c=='\n')
//    {
//        lcd_newline(pos);
//    }
//    else
//    {
//#if LCD_WRAP_LINES==1
//#if LCD_LINES==1
//        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
//            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
//        }
//#elif LCD_LINES==2
//        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
//            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE2,0);    
//        }else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH ){
//            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
//        }
//#elif LCD_LINES==4
//        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
//            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE2,0);    
//        }else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH ) {
//            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE3,0);
//        }else if ( pos == LCD_START_LINE3+LCD_DISP_LENGTH ) {
//            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE4,0);
//        }else if ( pos == LCD_START_LINE4+LCD_DISP_LENGTH ) {
//            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
//        }
//#endif
//        lcd_waitbusy();
//#endif
//        lcd_write(c, 1);
//    }

//}
//void lcd_puts(const char *s)
//{
//    register char c;

//    while ( (c = *s++) ) {
//        lcd_putc(c);
//    }

//}
//void lcd_init(uint8_t dispAttr)
//{
//    /*
//     *  Initialize LCD to 4 bit I/O mode
//     */
     
//    if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT )
//      && ( &LCD_RS_PORT == &LCD_DATA0_PORT) && ( &LCD_RW_PORT == &LCD_DATA0_PORT) && (&LCD_E_PORT == &LCD_DATA0_PORT)
//      && (LCD_DATA0_PIN == 0 ) && (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3) 
//      && (LCD_RS_PIN == 4 ) && (LCD_RW_PIN == 5) && (LCD_E_PIN == 6 ) )
//    {
//        /* configure all port bits as output (all LCD lines on same port) */
//        DDR(LCD_DATA0_PORT) |= 0x7F;
//    }
//    else if ( ( &LCD_DATA0_PORT == &LCD_DATA1_PORT) && ( &LCD_DATA1_PORT == &LCD_DATA2_PORT ) && ( &LCD_DATA2_PORT == &LCD_DATA3_PORT )
//           && (LCD_DATA0_PIN == 0 ) && (LCD_DATA1_PIN == 1) && (LCD_DATA2_PIN == 2) && (LCD_DATA3_PIN == 3) )
//    {
//        /* configure all port bits as output (all LCD data lines on same port, but control lines on different ports) */
//        DDR(LCD_DATA0_PORT) |= 0x0F;
//        DDR(LCD_RS_PORT)    |= _BV(LCD_RS_PIN);
//        DDR(LCD_RW_PORT)    |= _BV(LCD_RW_PIN);
//        DDR(LCD_E_PORT)     |= _BV(LCD_E_PIN);
//    }
//    else
//    {
//        /* configure all port bits as output (LCD data and control lines on different ports */
//        DDR(LCD_RS_PORT)    |= _BV(LCD_RS_PIN);
//        DDR(LCD_RW_PORT)    |= _BV(LCD_RW_PIN);
//        DDR(LCD_E_PORT)     |= _BV(LCD_E_PIN);
//        DDR(LCD_DATA0_PORT) |= _BV(LCD_DATA0_PIN);
//        DDR(LCD_DATA1_PORT) |= _BV(LCD_DATA1_PIN);
//        DDR(LCD_DATA2_PORT) |= _BV(LCD_DATA2_PIN);
//        DDR(LCD_DATA3_PORT) |= _BV(LCD_DATA3_PIN);
//    }
//    delay(16000);        /* wait 16ms or more after power-on       */
    
//    /* initial write to lcd is 8bit */
//    LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);  // _BV(LCD_FUNCTION)>>4;
//    LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);  // _BV(LCD_FUNCTION_8BIT)>>4;
//    lcd_e_toggle();
//    delay(4992);         /* delay, busy flag can't be checked here */
   
//    /* repeat last command */ 
//    lcd_e_toggle();      
//    delay(64);           /* delay, busy flag can't be checked here */
    
//    /* repeat last command a third time */
//    lcd_e_toggle();      
//    delay(64);           /* delay, busy flag can't be checked here */

//    /* now configure for 4bit mode */
//    LCD_DATA0_PORT &= ~_BV(LCD_DATA0_PIN);   // LCD_FUNCTION_4BIT_1LINE>>4
//    lcd_e_toggle();
//    delay(64);           /* some displays need this additional delay */
    
//    /* from now the LCD only accepts 4 bit I/O, we can use lcd_command() */    

//#if KS0073_4LINES_MODE
//    /* Display with KS0073 controller requires special commands for enabling 4 line mode */
//	lcd_command(KS0073_EXTENDED_FUNCTION_REGISTER_ON);
//	lcd_command(KS0073_4LINES_MODE);
//	lcd_command(KS0073_EXTENDED_FUNCTION_REGISTER_OFF);
//#else
//    lcd_command(LCD_FUNCTION_DEFAULT);      /* function set: display lines  */
//#endif
//    lcd_command(LCD_DISP_OFF);              /* display off                  */
//    lcd_clrscr();                           /* display clear                */ 
//    lcd_command(LCD_MODE_DEFAULT);          /* set entry mode               */
//    lcd_command(dispAttr);                  /* display/cursor control       */

//}
