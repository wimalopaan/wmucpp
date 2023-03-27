#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

namespace Bcd {
    struct Unpacked {
        const uint8_t d0{};
        const uint8_t d1{};
        const uint8_t d2{};
    };
    
    constexpr Unpacked toUnpacked(const uint16_t display_value) {
        uint16_t remaining{display_value};
        const uint8_t bcd0 = remaining % 10;
        remaining /= 10;
        const uint8_t bcd1 = remaining % 10;
        remaining /= 10;
        const uint8_t bcd2 = remaining % 10;
        return {bcd0, bcd1, bcd2};
    }
    
    namespace tests {
        static_assert([]{
            const auto [d0, d1, d2] = Bcd::toUnpacked(345);
            if (d2 != 3) return false;
            if (d1 != 4) return false;
            if (d0 != 5) return false;
            return true;
        }(), "Test1 failed");
        static_assert([]{
            const auto [d0, d1, d2] = Bcd::toUnpacked(987);
            if (d2 != 9) return false;
            if (d1 != 8) return false;
            if (d0 != 7) return false;
            return true;
        }(), "Test2 failed");
    }
}

namespace {
    constexpr uint8_t T1{0xFE};  // Basis Transistor 1 (PNP) Einerstelle
    constexpr uint8_t T2{0xFD};  // Basis Transistor 2  Zehnerstelle
    constexpr uint8_t T3{0xFB};  // Basis Transistor 3 Hunderterstelle
    
    constexpr uint8_t segmenttable[] { 0x03, 0xF3, 0x25, 0x0D, 0x99, 0x49, 0x41, 0x1F, 0x01, 0x19, 0xFE } ;

    constexpr uint8_t segments(const uint8_t d) {
        return segmenttable[d];
    }
    
    void init() {
        DDRD  = 0xFF; // PORTD auf Ausgang setzen
        DDRB  = 0xFF;
        PORTD = 0xFF; //Ausgabeport für 7-Segment-Ziffern
        PORTB = 0xFF; // Ansteuerung von T1 bis T3
    }
    
    void display(const uint16_t value) {
        const auto [d0, d1, d2] = Bcd::toUnpacked(value);
        
        PORTB = T1;
        PORTD = segments(d0);
        _delay_ms(7);
        
        PORTB = T2;
        PORTD = segments(d1);
        _delay_ms(7);
        
        PORTB = T3;
        PORTD = segments(d2);
        _delay_ms(7);
    }
}
int main() {
    init();
    const uint16_t value{345};
    while(true) {
        display(value);
    }
}
 
