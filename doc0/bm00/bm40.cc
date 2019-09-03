#include <mcu/pgm/pgmarray.h>
#include <etl/types.h>

struct Note {
    inline constexpr Note(uint16_t p, uint16_t l) : pitch(p), length(l) {}
    inline Note(const AVR::Pgm::Ptr<Note>& note_pgm) : pitch(pgm_read_word(&note_pgm->pitch)), length(pgm_read_word(&note_pgm->length)) {}

    const uint16_t pitch;
    const uint16_t length;
};

namespace  {
    constexpr Note a1{100, 200};
    constexpr Note a2{200, 300};
    constexpr Note a3{300, 400};
    constexpr Note a4{400, 500};
    constexpr Note a5{500, 600};
    
    constexpr auto array = AVR::Pgm::Array<Note, a1, a2, a3, a4, a5, a5, a4, a3, a2, a1>{};
    //constexpr auto array2 = AVR::Util::PgmArray<Note, a5, a4, a3, a2, a1>{};
    
}

volatile uint16_t r1;
volatile uint16_t r2;

int main() {
    etl::uint_ranged_circular<uint8_t, 0, array.size() - 1> index;
    
    while(true) {
        const auto n = array[index];
        r1 = n.pitch;
        r2 = n.length;
        ++index;
    }
    
    while(true) {
        for(const auto& n : array) {
            r1 = n.pitch;
            r2 = n.length;
        }
    }
}
