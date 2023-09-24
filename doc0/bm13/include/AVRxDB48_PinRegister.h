/*
  Devil-Elec
  Programmed and tested with: Arduino IDE 1.8.19 & avr-gcc 11.2.0
  23.01.2022
  AVRxDB48
  Numbering compatible with SpenceKonde labeling.
*/

#pragma once

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
    // | Base  | Base | CTRL | MASK | // BIT |   Pin   |   Pin   | 
       {0x0000, 0x0400, 0x10, 0x01},  //  0  |     0   |    44   | PA0
       {0x0000, 0x0400, 0x11, 0x02},  //  1  |     1   |    45   | PA1
       {0x0000, 0x0400, 0x12, 0x04},  //  2  |     2   |    46   | PA2
       {0x0000, 0x0400, 0x13, 0x08},  //  3  |     3   |    47   | PA3
       {0x0000, 0x0400, 0x14, 0x10},  //  4  |     4   |    48   | PA4
       {0x0000, 0x0400, 0x15, 0x20},  //  5  |     5   |     1   | PA5
       {0x0000, 0x0400, 0x16, 0x40},  //  6  |     6   |     2   | PA6
       {0x0000, 0x0400, 0x17, 0x80},  //  7  |     7   |     3   | PA7
       {0x0004, 0x0420, 0x10, 0x01},  //  0  |     8   |     4   | PB0
       {0x0004, 0x0420, 0x11, 0x02},  //  1  |     9   |     5   | PB1
       {0x0004, 0x0420, 0x12, 0x04},  //  2  |    10   |     6   | PB2
       {0x0004, 0x0420, 0x13, 0x08},  //  3  |    11   |     7   | PB3
       {0x0004, 0x0420, 0x14, 0x10},  //  4  |    12   |     8   | PB4
       {0x0004, 0x0420, 0x15, 0x20},  //  5  |    13   |     9   | PB5
       {0x0008, 0x0440, 0x10, 0x01},  //  0  |    14   |    10   | PC0
       {0x0008, 0x0440, 0x11, 0x02},  //  1  |    15   |    11   | PC1
       {0x0008, 0x0440, 0x12, 0x04},  //  2  |    16   |    12   | PC2
       {0x0008, 0x0440, 0x13, 0x08},  //  3  |    17   |    13   | PC3
       // VDDIO2                                       |    14
       // GND                                          |    15
       {0x0008, 0x0440, 0x14, 0x10},  //  4  |    18   |    16   | PC4
       {0x0008, 0x0440, 0x15, 0x20},  //  5  |    19   |    17   | PC5
       {0x0008, 0x0440, 0x16, 0x40},  //  6  |    20   |    18   | PC6
       {0x0008, 0x0440, 0x17, 0x80},  //  7  |    21   |    19   | PC7
       {0x000C, 0x0460, 0x10, 0x01},  //  0  |    22   |    20   | PD0
       {0x000C, 0x0460, 0x11, 0x02},  //  1  |    23   |    21   | PD1
       {0x000C, 0x0460, 0x12, 0x04},  //  2  |    24   |    22   | PD2
       {0x000C, 0x0460, 0x13, 0x08},  //  3  |    25   |    23   | PD3
       {0x000C, 0x0460, 0x14, 0x10},  //  4  |    26   |    24   | PD4
       {0x000C, 0x0460, 0x15, 0x20},  //  5  |    27   |    25   | PD5
       {0x000C, 0x0460, 0x16, 0x40},  //  6  |    28   |    26   | PD6
       {0x000C, 0x0460, 0x17, 0x80},  //  7  |    29   |    27   | PD7
       // AVDD                                         |    28
       // GND                                          |    29
       {0x0010, 0x0480, 0x10, 0x01},  //  0  |    30   |    30   | PE0
       {0x0010, 0x0480, 0x11, 0x02},  //  1  |    31   |    31   | PE1
       {0x0010, 0x0480, 0x12, 0x04},  //  2  |    32   |    32   | PE2
       {0x0010, 0x0480, 0x13, 0x08},  //  3  |    33   |    33   | PE3
       {0x0014, 0x04A0, 0x10, 0x01},  //  0  |    34   |    34   | PF0
       {0x0014, 0x04A0, 0x11, 0x02},  //  1  |    35   |    35   | PF1
       {0x0014, 0x04A0, 0x12, 0x04},  //  2  |    36   |    36   | PF2
       {0x0014, 0x04A0, 0x13, 0x08},  //  3  |    37   |    37   | PF3
       {0x0014, 0x04A0, 0x14, 0x10},  //  4  |    38   |    38   | PF4
       {0x0014, 0x04A0, 0x15, 0x20},  //  5  |    39   |    39   | PF5
       {0x0014, 0x04A0, 0x16, 0x40},  //  6  |    40   |    40   | PF6
       // UPDI                                         |    41
       // VDD                                          |    42
       // GND                                          |    43
    };
    
    constexpr uint8_t ANZAHLPINS = ( sizeof(baseAddr) / sizeof(baseAddr[0]) ) ;
  }
}
