#include <iostream>
#include <array>

constexpr inline void crc16(uint16_t& crc, uint8_t value) {
    constexpr uint16_t crc_polynome = 0x1021;
    crc = crc ^ (((uint16_t)value) << 8);
    for(uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ crc_polynome;
        }
        else {
            crc = (crc << 1);
        }
    }
}

int main() {
    std::array<uint8_t, 19> f = {0xa8, 0x01, 0x08,
                             0x37, 0xf8,
                             0x2e, 0xf8,
                             0x2f, 0x20,
                             0x2e, 0xc8,
                             0x3b, 0x60,
                             0x2e, 0xe0,
                             0x22, 0x60,
                             0x2e, 0xe0,

//                             0x1a, 0x08
                            };
    uint16_t csum = 0;
    for(uint8_t v: f) {
        crc16(csum, v);
    }
    
    uint8_t h = csum >> 8;
    uint8_t l = csum;
    
    printf("h: %x, l: %x \n", h, l);
}
