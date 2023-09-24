/*
  Devil-Elec
  Programmed and tested with: Arduino IDE 1.8.19 & avr-gcc 11.2.0
  23.01.2022
  AVRxDB32
  Numbering is not compatible with SpenceKonde labeling. Pin12!
*/

#pragma once

#include <stdint.h>

namespace Pins
{
  namespace Addr
  {
    struct Offset // Offsets der Registeradressen
    {
      // VPORTs
      const uint8_t vDir  = 0x00;
      const uint8_t vOut  = 0x01;
      const uint8_t vIn   = 0x02;
      const uint8_t vFlag = 0x03;
      // PORTs
      const uint8_t dir           = 0x00;
      const uint8_t dirSet        = 0x01;
      const uint8_t dirClear      = 0x02;
      const uint8_t dirToggle     = 0x03;
      const uint8_t out           = 0x04;
      const uint8_t outSet        = 0x05;
      const uint8_t outClear      = 0x06;
      const uint8_t outToggle     = 0x07;
      const uint8_t in            = 0x08;
      const uint8_t flag          = 0x09;
      const uint8_t portCtrl      = 0x0A;
      const uint8_t pinConfig     = 0x0B;
      const uint8_t pinCtrlUpdate = 0x0C;
      const uint8_t pinCtrlSet    = 0x0D;
      const uint8_t pinCtrlClear  = 0x0E;
    };  
    constexpr Offset addrOffset;

    struct Address // base addresses of registers
    {
      uint16_t vport;  
      uint16_t port;
      uint8_t  pinCtrl;
      uint8_t  mask;
    };

    constexpr Address baseAddr[]
    {
    // | VPORT | PORT | PINn | BIT  | //     | Arduino | Package |
    // | Base  | Base | CTRL | MASK | // BIT |   Pin   |   Pin   | PORT
       {0x0000, 0x0400, 0x10, 0x01},  //  0  |     0   |    30   | PA0
       {0x0000, 0x0400, 0x11, 0x02},  //  1  |     1   |    31   | PA1
       {0x0000, 0x0400, 0x12, 0x04},  //  2  |     2   |    32   | PA2       
       {0x0000, 0x0400, 0x13, 0x08},  //  3  |     3   |     1   | PA3
       {0x0000, 0x0400, 0x14, 0x10},  //  4  |     4   |     2   | PA4
       {0x0000, 0x0400, 0x15, 0x20},  //  5  |     5   |     3   | PA5
       {0x0000, 0x0400, 0x16, 0x40},  //  6  |     6   |     4   | PA6
       {0x0000, 0x0400, 0x17, 0x80},  //  7  |     7   |     5   | PA7

       {0x0008, 0x0440, 0x10, 0x01},  //  0  |     8   |     6   | PC0
       {0x0008, 0x0440, 0x11, 0x02},  //  1  |     9   |     7   | PC1
       {0x0008, 0x0440, 0x12, 0x04},  //  2  |    10   |     8   | PC2
       {0x0008, 0x0440, 0x13, 0x08},  //  3  |    11   |     9   | PC3
       // VDDIO2                                       |    10   |     
       {0x000C, 0x0460, 0x11, 0x02},  //  1  |    12   |    11   | PD1
       {0x000C, 0x0460, 0x12, 0x04},  //  2  |    13   |    12   | PD2
       {0x000C, 0x0460, 0x13, 0x08},  //  3  |    14   |    13   | PD3
       {0x000C, 0x0460, 0x14, 0x10},  //  4  |    15   |    14   | PD4
       {0x000C, 0x0460, 0x15, 0x20},  //  5  |    16   |    15   | PD5
       {0x000C, 0x0460, 0x16, 0x40},  //  6  |    17   |    16   | PD6
       {0x000C, 0x0460, 0x17, 0x80},  //  7  |    18   |    17   | PD7
       // AVDD                                         |    18   |
       // GND                                          |    19   |    
       {0x0014, 0x04A0, 0x10, 0x01},  //  0  |    19   |    20   | PF0
       {0x0014, 0x04A0, 0x11, 0x02},  //  1  |    20   |    21   | PF1
       {0x0014, 0x04A0, 0x12, 0x04},  //  2  |    21   |    22   | PF2
       {0x0014, 0x04A0, 0x13, 0x08},  //  3  |    22   |    23   | PF3
       {0x0014, 0x04A0, 0x14, 0x10},  //  4  |    23   |    24   | PF4
       {0x0014, 0x04A0, 0x15, 0x20},  //  5  |    24   |    25   | PF5
       {0x0014, 0x04A0, 0x16, 0x40},  //  6  |    25   |    26   | PF6
       // UPDI                                         |    27   |
       // VDD                                          |    28   |
       // GND                                          |    29   |
    };
    
    constexpr uint8_t ANZAHLPINS = ( sizeof(baseAddr) / sizeof(baseAddr[0]) ) ;
  }
}
