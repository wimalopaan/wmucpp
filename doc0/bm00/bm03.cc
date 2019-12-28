#include <mcu/avr.h>
#include <array>
#include <cmath>

//#define V1

#ifdef V1
const int PROGMEM ntctable[] = {
  1597, 1305, 1013, 851, 735, 643, 565, 495, 
  430, 368, 306, 242, 175, 98, 4, -133, 
  -270
};
 
// Diese Funktion ist unsicher (index out-of-range)

int ntc_gettemp(uint16_t adc_value) {
  int p1 = pgm_read_word(&(ntctable[ (adc_value >> 6)    ]));
  int p2 = pgm_read_word(&(ntctable[ (adc_value >> 6) + 1]));
  return p1 - ( (p1-p2) * (adc_value & 0x003f) ) / 64;
}

#else 

namespace AVR::NTC {
    template<auto Size, typename ValueType = int, auto AdcBits = 10>
    struct Lut final {
        Lut() = delete;
        //[  Die folgenden Definitionen sind nur convenience, kannst Du alles weglassen
        inline static constexpr size_t sizeMax{256}; // maximum number of intervals
        inline static constexpr size_t table_size{Size + 1}; 
        static_assert(Size <= sizeMax, "lut too big");

        static_assert(AdcBits <= 16);        
        using adc_type = std::conditional_t<(AdcBits <= 8), uint8_t, uint16_t>;
        inline static constexpr adc_type adc_mask{(uint32_t)(1 << AdcBits) - 1};
        
        inline static constexpr uint8_t size_bits{static_cast<uint8_t>(log10(table_size + 0.5) / log10(2.0))};
        static_assert(AdcBits > size_bits);
        inline static constexpr uint8_t adc_shift{AdcBits - size_bits};
        
        using value_type = ValueType;
        using size_type = std::conditional_t<(table_size <= 256), uint8_t, uint16_t>;
        //]
        static inline constexpr size_type size() {
            return Size;
        }
    private:
        // In diesem closure werden die lut-Daten berechnet: der Einfachheit halber habe ich einfach nur sin() genommen.
        // Alles, was Du hier berechnest, wird ausschließlich zur Compile-Zeit berechnent. 
        // Davon findet sich nichts im späteren Code.
        inline static constexpr auto init_data{[]{
            std::array<value_type, table_size> t{};
            for(size_type i{0}; i < t.size(); ++i) {
                t[i] = std::numeric_limits<value_type>::max() * sin(1.1 * i); // do whatever you want
                // ...
            }
            return t;
        }()};
        // Das folgende template kapselt den PGM-Zugriff
        template<typename> struct Pgm;
        template<auto... II>
        struct Pgm<std::index_sequence<II...>> final {
            Pgm() = delete;
            static_assert(sizeof(value_type) <= 4); // wegen pgm_read_dword()
            static_assert(std::is_fundamental_v<value_type>); // UDT benötigen einen ctor, der einen PGM-ptr akzeptiert, daher hier nicht betrachtet
            inline static std::pair<value_type, value_type> get(size_type i) {
                assert(i < size());
                if constexpr(sizeof(value_type) == 1) {
                    return {pgm_read_byte(&data[i]), pgm_read_byte(&data[i + 1])};
                }
                else if constexpr(sizeof(value_type) == 2) {
                    return {pgm_read_word(&data[i]), pgm_read_word(&data[i + 1])};
                }
                else if constexpr(sizeof(value_type) == 4) {
                    return {pgm_read_dword(&data[i]), pgm_read_dword(&data[i + 1])};
                }
            }
        private:
            inline static constexpr value_type data[] PROGMEM {init_data[II]...}; 
        };
        // Dies ist der entscheidende Meta-Funktions-Aufruf, um die Pgm-Daten zu generieren.
        using pgm = Pgm<std::make_index_sequence<Size>>; 
    public:
        // Die korrespondierende Funktion zu Deiner ntc_gettemp() Funktion
        static inline value_type get(adc_type v) {
            static_assert((adc_mask >> adc_shift) < size());
            v &= adc_mask;
            const size_type index{static_cast<size_type>(v >> adc_shift)};
            const auto [p1, p2] {pgm::get(index)};
            return p1 - ((p1 - p2) * v) / value_type{64};
        }
    };
}

using lut = AVR::NTC::Lut<16>;
#endif

volatile uint16_t adc; // fake adc

int main() {
#ifdef V1
    return ntc_gettemp(adc);
#else
    return lut::get(adc);
#endif
}
