/*
  Devil-Elec
  Programmed and tested with: Arduino IDE 1.8.19 & avr-gcc 11.2.0
  13.01.2022
  AVRxDB64 - Numbering compatible with SpenceKonde labeling.
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
       {0x0000, 0x0400, 0x10, 0x01},  //  0  |     0   |    62   | PA0
       {0x0000, 0x0400, 0x11, 0x02},  //  1  |     1   |    63   | PA1
       {0x0000, 0x0400, 0x12, 0x04},  //  2  |     2   |    64   | PA2
       {0x0000, 0x0400, 0x13, 0x08},  //  3  |     3   |     1   | PA3
       {0x0000, 0x0400, 0x14, 0x10},  //  4  |     4   |     2   | PA4
       {0x0000, 0x0400, 0x15, 0x20},  //  5  |     5   |     3   | PA5
       {0x0000, 0x0400, 0x16, 0x40},  //  6  |     6   |     4   | PA6
       {0x0000, 0x0400, 0x17, 0x80},  //  7  |     7   |     5   | PA7
       // VDD                                          |     6
       // GND                                          |     7
       {0x0004, 0x0420, 0x10, 0x01},  //  0  |     8   |     8   | PB0
       {0x0004, 0x0420, 0x11, 0x02},  //  1  |     9   |     9   | PB1
       {0x0004, 0x0420, 0x12, 0x04},  //  2  |    10   |    10   | PB2
       {0x0004, 0x0420, 0x13, 0x08},  //  3  |    11   |    11   | PB3
       {0x0004, 0x0420, 0x14, 0x10},  //  4  |    12   |    12   | PB4
       {0x0004, 0x0420, 0x15, 0x20},  //  5  |    13   |    13   | PB5
       {0x0004, 0x0420, 0x16, 0x40},  //  6  |    14   |    14   | PB6
       {0x0004, 0x0420, 0x17, 0x80},  //  7  |    15   |    15   | PB7
       
       {0x0008, 0x0440, 0x10, 0x01},  //  0  |    16   |    16   | PC0
       {0x0008, 0x0440, 0x11, 0x02},  //  1  |    17   |    17   | PC1
       {0x0008, 0x0440, 0x12, 0x04},  //  2  |    18   |    18   | PC2
       {0x0008, 0x0440, 0x13, 0x08},  //  3  |    19   |    19   | PC3
       // VDDIO2                                       |    20
       // GND                                          |    21
       {0x0008, 0x0440, 0x14, 0x10},  //  4  |    20   |    22   | PC4
       {0x0008, 0x0440, 0x15, 0x20},  //  5  |    21   |    23   | PC5
       {0x0008, 0x0440, 0x16, 0x40},  //  6  |    22   |    24   | PC6
       {0x0008, 0x0440, 0x17, 0x80},  //  7  |    23   |    25   | PC7
       
       {0x000C, 0x0460, 0x10, 0x01},  //  0  |    24   |    26   | PD0
       {0x000C, 0x0460, 0x11, 0x02},  //  1  |    25   |    27   | PD1
       {0x000C, 0x0460, 0x12, 0x04},  //  2  |    26   |    28   | PD2
       {0x000C, 0x0460, 0x13, 0x08},  //  3  |    27   |    29   | PD3
       {0x000C, 0x0460, 0x14, 0x10},  //  4  |    28   |    30   | PD4
       {0x000C, 0x0460, 0x15, 0x20},  //  5  |    29   |    31   | PD5
       {0x000C, 0x0460, 0x16, 0x40},  //  6  |    30   |    32   | PD6
       {0x000C, 0x0460, 0x17, 0x80},  //  7  |    31   |    33   | PD7
       // AVDD                                         |    34
       // AGND                                         |    35
       {0x0010, 0x0480, 0x10, 0x01},  //  0  |    32   |    36   | PE0
       {0x0010, 0x0480, 0x11, 0x02},  //  1  |    33   |    37   | PE1
       {0x0010, 0x0480, 0x12, 0x04},  //  2  |    34   |    38   | PE2
       {0x0010, 0x0480, 0x13, 0x08},  //  3  |    35   |    39   | PE3
       {0x0010, 0x0480, 0x14, 0x10},  //  4  |    36   |    40   | PE4
       {0x0010, 0x0480, 0x15, 0x20},  //  5  |    37   |    41   | PE5
       {0x0010, 0x0480, 0x16, 0x40},  //  6  |    38   |    42   | PE6
       {0x0010, 0x0480, 0x17, 0x80},  //  7  |    39   |    43   | PE7
       
       {0x0014, 0x04A0, 0x10, 0x01},  //  0  |    40   |    44   | PF0
       {0x0014, 0x04A0, 0x11, 0x02},  //  1  |    41   |    45   | PF1
       {0x0014, 0x04A0, 0x12, 0x04},  //  2  |    42   |    46   | PF2
       {0x0014, 0x04A0, 0x13, 0x08},  //  3  |    43   |    47   | PF3
       {0x0014, 0x04A0, 0x14, 0x10},  //  4  |    44   |    48   | PF4
       {0x0014, 0x04A0, 0x15, 0x20},  //  5  |    45   |    49   | PF5
       // UPDI                                         |    51
       {0x0018, 0x04C0, 0x10, 0x01},  //  0  |    46   |    52   | PG0
       {0x0018, 0x04C0, 0x11, 0x02},  //  1  |    47   |    53   | PG1
       {0x0018, 0x04C0, 0x12, 0x04},  //  2  |    48   |    54   | PG2
       {0x0018, 0x04C0, 0x13, 0x08},  //  3  |    49   |    55   | PG3
       // VDD                                          |    56
       // GND                                          |    57
       {0x0018, 0x04C0, 0x14, 0x10},  //  4  |    50   |    58   | PG4
       {0x0018, 0x04C0, 0x15, 0x20},  //  5  |    51   |    59   | PG5
       {0x0018, 0x04C0, 0x16, 0x40},  //  6  |    52   |    60   | PG6
       {0x0018, 0x04C0, 0x17, 0x80},  //  7  |    53   |    61   | PG7
       
       {0x0014, 0x04A0, 0x16, 0x40},  //  6  |    54   |    50   | PF6 | Resetpin!
    };
    
    constexpr uint8_t ANZAHLPINS = ( sizeof(baseAddr) / sizeof(baseAddr[0]) );
  }
}
