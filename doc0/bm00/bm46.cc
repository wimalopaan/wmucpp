#include <mcu/avr.h>
#include <mcu/pgm/pgmstring.h>
#include <mcu/pgm/pgmarray.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/portmux.h>

#include <external/units/music.h>

#include <std/chrono>
#include <etl/output.h>
#include <etl/vector.h>
#include <etl/bitfield.h>
#include <string.h>

// https://www.cubeatsystems.com/tinymml/index.html
// https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML

// <note>|r|p|<|>|o|l[+-][<length>][.|..]
// note: cdefgab 
// r,p : pause
// o : Oktave
// l : default length
// length: 1, 2, 4, 8, 16, 32
// < : octave up
// > : octave down
// loop: [ ... ]2

// volume setting: @v0 = {15 8 7 6 5}
// Idee: DAC vom tiny1614 benutzen
// ???: wie auf eine Notenlänge verteilen?

using namespace AVR;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
using tonePwm = PWM::DynamicPwm<tcaPosition>;

namespace External::Music {
    template <char... chars>
    struct Mml {};
    
    template <typename C, C... chars>
    requires (std::is_same_v<C, char>)
    constexpr Mml<chars...> operator"" _mml() { 
        return {}; 
    }
    
    template<typename> struct Parser;
    
    template<char... CC>
    struct Parser<Mml<CC...>> {
        
        inline static constexpr size_t byte_size() {
            return sizeof...(CC);
        }

        inline static constexpr auto periods = []{
            etl::Vector<size_t, byte_size()> p;
            return p;
        }();
        
        inline static constexpr auto byte_data = []{
            std::array<std::byte, byte_size()> data{std::byte{CC}...};
            return data;
        }();
        
        inline static constexpr char mml[] {CC...};
    };
    
    template <typename, typename> struct Store;
    
    template <typename PWM, typename... MMs>
    struct Store<PWM, Meta::List<MMs...>> {
        using parsers = Meta::List<Parser<MMs>...>;
        
        inline static constexpr size_t size() {
            return sizeof...(MMs);
        }
        
        inline static constexpr size_t byte_size() {
            size_t sum{};
            Meta::visit<parsers>([&]<typename P>(Meta::Wrapper<P>){
                sum += P::byte_size();
            });
            return sum;
        }
        struct Melody {
            inline constexpr std::byte operator[](size_t index) {
                return std::byte{pgm_read_byte(&data[mOffset + index])};
            }
            const size_t mOffset;            
        };
        inline static constexpr Melody melody(uint8_t m) {
            size_t o = pgm_read_word(&offsets[m]);
            return Melody{o};
        }
    private:
        
        
        
        inline static constexpr auto periods PROGMEM = []{
            
        }();
        
        inline static constexpr auto data PROGMEM = []{
            std::array<std::byte, byte_size()> d{};
            auto it = d.begin();
            Meta::visit<parsers>([&]<typename P>(Meta::Wrapper<P>){
                                     const auto& pd = P::byte_data;
                                     std::copy(pd.begin(), pd.end(), it);
                                     it += P::byte_size();
                                 });
            return d;
        }();
        inline static constexpr auto offsets PROGMEM = []{
            std::array<size_t, size()> offs{};
            auto it = offs.begin() + 1;
            Meta::visit<Meta::rest<parsers>>([&]<typename P>(Meta::Wrapper<P>){
                                                 *it++ = P::byte_size();                                
                                             });
            return offs;
        }();
    }; 
    
    
//    template<typename... FS>
//    consteval uint16_t make_rep(FS...) {
//        return Parser<Meta::List<FS...>>::sum();
//    }
}

using namespace External::Music;

volatile uint8_t mindex;
volatile uint8_t bindex;

int main() {
//    auto bach13 = []{
//        return "T90L16REA>C<BEB>DC8E8<G+8>E8<AEA>C<BEB>DC8<A8R4R>ECE<A>C<EGF8A8>D8F8.D<B>D<GBDFE8G8>C8E8.C<A>C<F8>D8.<BGBE8>C8.<AFAD8B8>C4R4<RG>CED<G>DFE8G8<B8>G8C<G>CED<G>DFE8C8G8E8>C<AEACE<A>CD8F+8A8>C8<BGDG<B>D<GB>C8E8G8B8AF+D+F+<B>D<F+AG8>G8.ECE<A8>F+8.D<B>D<G8>E8.C<A>C<F+>GF+ED+F+<B>D+E4R4<<E2L16O2A8>A4G+8AEA>C<BEB>DC8<A8G+8E8AEA>C<BEB>DC8<A8>C8<A8>D<AFADF<A>C<B8>D8G8B8.GEGCE<GBA8>C8DF<B>D<G8B8>CE<A>C<F8D8G>GFGCG>CED<G>DFE8C8<B8G8>C<G>GED<G>DFE8C8R4RGEGCE<GBA8>C8E8G8F+ADF+<A>D<F+AG8B8>D8F+8EGCE<G>C<EGF+8A8B8>D+8RECE<A>CEGF+D<B>D<GB>DF+EC<A>C<F+A>C8.<B>C<AB8<B8>E>E<BGE<BGBE2"_mml;
//    };
//    auto scaleC = []{
//        return "L8 C D E F G A B > C"_mml;
//    };
    
    using scale = decltype("L8 C D E F G A B > C"_mml);
    using bach  = decltype("T90L16REA>C<BEB>DC8E8"_mml);
    
    using store = Store<tonePwm, Meta::List<scale, bach>>;
    
    return uint8_t(store::melody(mindex)[bindex]);
}
