//#include <mcu/avr.h>
//#include <mcu/pgm/pgmstring.h>
//#include <mcu/pgm/pgmarray.h>
//#include <mcu/internals/pwm.h>
//#include <mcu/internals/portmux.h>

//#include <external/units/music.h>

//#include <std/chrono>
//#include <std/stack>

//#include <etl/output.h>
//#include <etl/vector.h>
//#include <etl/bitfield.h>
//#include <string.h>

//// https://www.cubeatsystems.com/tinymml/index.html
//// https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML

//// <note>|r|p|<|>|o|l[+-][<length>][.|..]
//// note: cdefgab 
//// r,p : pause
//// o : Oktave
//// l : default length
//// length: 1, 2, 23, 4, 43, 8, 83, 16, 163, 32, 323, 64, 643
//// < : octave up
//// > : octave down
//// loop: [ ... ]

//// volume setting: @v0 = {15 8 7 6 5}
//// Idee: DAC vom tiny1614 benutzen
//// ???: wie auf eine Notenlänge verteilen?

//using namespace AVR;

//using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
//using tonePwm = PWM::DynamicPwm<tcaPosition>;

//namespace External::MML {
//    using namespace etl;
    
//    namespace Bits {
//        struct tone_tag;
//        struct duration_tag;
//        using tone_bits = etl::Bits<tone_tag, BitNumber<5>, BitPosition<0>>;
//        using duration_bits = etl::Bits<duration_tag, BitNumber<3>, BitPosition<5>>;
//        using note_t = etl::BitField<Meta::List<tone_bits, duration_bits>, std::byte>;
//    }
    
//    template <etl::Char... chars> struct Mml {};
    
//    namespace literals {
//        template <typename C, C... chars>
//        requires (std::is_same_v<C, char>)
//        constexpr Mml<etl::Char{chars}...> operator"" _mml() { 
//            return {}; 
//        }
//    }

//    struct Duration final {
//        constexpr explicit Duration(const uint8_t denom = 4) : denominator{denom} {}
//        constexpr void operator++() {
//            numerator *= 2;
//            denominator *= 2;
//            numerator += 1;
//        }
//        constexpr void toTriplet() {
//            denominator *= 3;
//            denominator /= 2;
//        }
//        constexpr void add(const Char c) {
//            uint8_t d = asDigit(c);
//            if (duration_chars == 0) {
//                denominator = d;
//            }
//            else if (duration_chars == 1) {
//                if (d == 3) {
//                    toTriplet();
//                }
//                else {
//                    denominator = denominator * 10 + d;
//                }
//            }
//            else if (duration_chars == 2) {
//                if (d == 3) {
//                    toTriplet();                        
//                }
//            }
//            ++duration_chars;
//        }
//    private:
//        uint8_t numerator{1};
//        uint8_t denominator{4};
//        uint8_t duration_chars{};
//    };

//    struct Octave final {
//        constexpr Octave() = default;
//        constexpr explicit Octave(uint8_t n) : number{n} {}
//        constexpr void operator++() {
//            if (number < 10) ++number;
//        }
//        constexpr void operator--() {
//            if (number > 0) --number;
//        }
//    private:
//        uint8_t number{4};
//    };
    
    
//    struct Note final {
//        inline static constexpr uint8_t rest_index = 255;
//        inline static constexpr uint8_t repeat_index = 254;
        
//        constexpr /*explicit */ Note(const Char c = Char{'c'}, const Duration d = Duration{}, const Octave o = Octave{}) :
//            scale_index{indexOfNoteLetter(c)}, octave{o}, duration{d} {}

//        constexpr explicit Note(const uint8_t i) : scale_index{i} {}
        
//        constexpr bool isRepeatMark() const {
//            return (scale_index == repeat_index);
//        }
//        constexpr bool isPause() const {
//            return (scale_index == rest_index);
//        }
                
//        constexpr void operator++() {
//            if (scale_index == 255) return;
//            if (scale_index < 11) {
//                scale_index += 1;
//            }
//            else {
//                scale_index = 0;
//                ++octave;
//            }
//        }
//        constexpr void operator--() {
//            if (scale_index == 255) return;
//            if (scale_index > 0) {
//                scale_index -= 1;
//            }
//            else {
//                scale_index = 11;
//                --octave;
//            }
//        }
//        constexpr void addToDuration(const Char c) {
//            duration.add(c);
//        }
//        constexpr void lengthen() {
//            ++duration;
//        }
    
//        inline static constexpr bool isNoteLetter(const Char c) {
//            return (std::end(letters) != std::find_if(std::begin(letters), std::end(letters),[&](auto i){
//              return (i.first == c);}));
//        }
//        inline static constexpr uint8_t indexOfNoteLetter(const Char c) {
//            for(uint8_t i{}; i < letters.size(); ++i) {
//                if (c == letters[i].first) {
//                    return i;
//                }
//            }
//            // flow of
//        }
//        inline static constexpr Note repeat_mark() {
//            return Note{repeat_index};
//        } 
//    private:
//        using p = std::pair<Char, uint8_t>;
//        inline static constexpr std::array letters {p{Char{'c'}, 0}, p{Char{'d'}, 2}, p{Char{'e'}, 4}, p{Char{'f'}, 5}, p{Char{'g'}, 7}, p{Char{'a'}, 9}, p{Char{'b'}, 11}, p{Char{'h'}, 11}, p{Char{'r'}, rest_index}, p{Char{'p'}, rest_index}};
//        uint8_t scale_index{}; // 0 ... 11, 255 für Pause, 254 für Repeat
//        Octave octave{}; // 0 ... 10
//        Duration duration{}; // to be multiplied with base-length
//    };
    
//    template<typename> struct Parser;
    
//    template<etl::Char... CC>
//    struct Parser<Mml<CC...>> final {
//        inline static constexpr size_t max_size = 3 * sizeof...(CC);
        
//        inline static constexpr size_t byte_size() {
//            constexpr auto d = parse();
//            static_assert(d, "parse error");
//            if constexpr(d) {
////                std::integral_constant<size_t, d->size()>::_;
//                return d->size();
//            }
//            return 0; 
//        }
        
//        inline static constexpr auto notes() {
//            constexpr auto data = parse();
//            static_assert(data, "parse error");

//            std::array<Note, byte_size()> d;
//            return d;            
//        };
        
//        inline static constexpr std::optional<etl::Vector<Note, max_size>> parse() {
//            etl::Vector<Note, max_size> notes;
            
//            std::stack<uint16_t, etl::Vector<uint16_t, 10>> repeat_stack;
            
//            enum class State {Start, Note, Length, Octave};
//            State state = State::Start;
//            Octave actual_octave{4};
//            Duration actual_duration{4};
            
//            Note n;
//            for(const Char cc : mml) {
//                const Char c = etl::toLower(cc);                              
//                switch(state) {
//                    case State::Start:
//                    if (Note::isNoteLetter(c)) {
//                        n = Note{c, actual_duration, actual_octave};
//                        state = State::Note;
//                    }
//                    else if (c == Char{'l'}) {
//                    }
//                    else {
//                        return {};
//                    }
//                    break;
//                case State::Note:
//                    if (Note::isNoteLetter(c)) {
//                        notes.push_back(n);
//                        n = Note{c, actual_duration, actual_octave};
//                    }
//                    else if (c == Char{'+'}) {
//                        ++n;
//                    }
//                    else if (c == Char{'-'}) {
//                        --n;
//                    }
//                    else if (isDigit(c)) {
//                        n.addToDuration(c);
//                    }
//                    else if (c == Char{'.'}) {
//                        n.lengthen();
//                    }
//                    else if (c == Char{' '}) {
//                        state = State::Start;
//                        notes.push_back(n);
//                    }
//                    else if (c == Char{'l'}) {
//                        state = State::Length;
//                        actual_duration = Duration{};
//                        notes.push_back(n);
//                    }
//                    else if (c == Char{'o'}) {
//                        state = State::Octave;
//                        notes.push_back(n);
//                    }
//                    else if (c == Char{'<'}) {
//                        state = State::Start;
//                        ++actual_octave;
//                        notes.push_back(n);
//                    }
//                    else if (c == Char{'>'}) {
//                        state = State::Start;
//                        --actual_octave;
//                        notes.push_back(n);
//                    }
//                    else if (c == Char{'['}) {
//                        state = State::Start;
//                        notes.push_back(n);
//                        repeat_stack.push(notes.size()); // actual position
//                    }
//                    else if (c == Char{']'}) {
//                        state = State::Start;
//                        notes.push_back(n);
//                        uint16_t position = repeat_stack.top();
//                        repeat_stack.pop();
//                        notes.push_back(Note::repeat_mark());                        
//                    }
//                    else {
//                        return {};
//                    }
//                    break;
//                case State::Length:
//                    if (isDigit(c)) {
//                        actual_duration.add(c);
//                    }
//                    else if (Note::isNoteLetter(c)) {
//                        n = Note{c, actual_duration, actual_octave};
//                        state = State::Note;
//                    }
//                    else {
//                        state = State::Start;
//                    }
//                    break;
//                case State::Octave:
//                    break;
//                }                    
//            }
//            if (state == State::Note) {
//                notes.push_back(n);
//            }            
//            return std::optional{notes};
//        }
        
//        inline static constexpr auto periods = []{
//            etl::Vector<size_t, byte_size()> p;
//            return p;
//        }();
        
//        inline static constexpr auto byte_data = []{
//            constexpr auto d = parse();
//            static_assert(d, "parse error");
//            if constexpr(d) {
//                std::array<External::MML::Bits::note_t, d->size()> data{};
//                return data;
//            }
//        }();
//    private:        
//        inline static constexpr std::array<etl::Char, sizeof...(CC)> mml{CC...};
//    };
    
//    template <typename, typename> struct Store;
    
//    template <typename PWM, typename... MMs>
//    struct Store<PWM, Meta::List<MMs...>> final {
//        using parsers = Meta::List<Parser<MMs>...>;
        
//        using size_type = etl::typeForValue_t<sizeof...(MMs)>;
//        using melody_index_type = etl::uint_ranged<size_type, 0, (sizeof...(MMs))>;
        
//        inline static constexpr size_type size() { // # of melodys
//            return sizeof...(MMs);
//        }
        
//        inline static constexpr size_t byte_size() { // total number of bytes of storage
//            size_t sum{};
//            Meta::visit<parsers>([&]<typename P>(Meta::Wrapper<P>){
//                                     sum += P::byte_size();
//                                 });
//            return sum;
//        }
//        struct Melody {
//            inline constexpr explicit Melody(const size_t offset) : mOffset{offset}{}
//            inline constexpr std::byte operator[](const size_t index) {
//                return std::byte{pgm_read_byte(&data[mOffset + index])};
//            }
//        private:
//            const size_t mOffset;            
//        };
//        inline static constexpr Melody melody(const melody_index_type m) {
//            assert(m < size());
//            size_t o = pgm_read_word(&offsets[m]);
//            return Melody{o};
//        }
//    private:
//        inline static constexpr auto periods PROGMEM = []{
            
//        }();
        
//        inline static constexpr auto data PROGMEM = []{
//            std::array<std::byte, byte_size()> d{};
//            auto it = d.begin();
//            Meta::visit<parsers>([&]<typename P>(Meta::Wrapper<P>){
//                                     //                                     const auto& pd = P::byte_data;
//                                     //                                     std::copy(pd.begin(), pd.end(), it);
//                                     it += P::byte_size();
//                                 });
//            return d;
//        }();
//        inline static constexpr auto offsets PROGMEM = []{
//            std::array<size_t, size()> offs{};
//            auto it = offs.begin() + 1;
//            Meta::visit<Meta::rest<parsers>>([&]<typename P>(Meta::Wrapper<P>){
//                                                 *it++ = P::byte_size();                                
//                                             });
//            return offs;
//        }();
//    }; 
    
    
//    template<typename P, auto Mml>
//    using StoreV = Store<P, decltype(Mml)>;
    
//    template<auto... Mmls>
//    struct L;
//}


//using namespace External::MML;
//using namespace External::MML::literals;

//using s = decltype("L8 C D E F G A B > C"_mml);
//using bach  = decltype("T90L16REA>C<BEB>DC8E8"_mml);
//using w = decltype("C D E F G"_mml);

//using l = L<"C D E F G"_mml, "L8 C D E F G A B > C"_mml>;

//constexpr auto w1 = "C D E F G"_mml;
//constexpr auto w2 = "C D E F G"_mml;
//using l1 = L<w1, w2>;

//using storev = StoreV<tonePwm, Meta::List<w>{}>;
//using store = Store<tonePwm, Meta::List<w>>;
////using store = Store<tonePwm, Meta::List<s, bach>>;

////std::integral_constant<size_t, store::byte_size()>::_;

//int main() {
    
    
//    store::melody_index_type mindex{};
//    size_t bindex{};
    
//    return uint8_t(store::melody(mindex)[bindex]);
//}
