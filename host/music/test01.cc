#include <chrono>
#include <iostream>

namespace std {
    template<typename E>
    struct enable_bitmask_operators final : std::false_type {};
    
    template<typename E>
    inline constexpr bool enable_bitmask_operators_v = enable_bitmask_operators<E>::value;
}

#include <etl/bitfield.h>
#include <etl/vector.h>
#include <etl/stack.h>

#define PROGMEM

template<typename B>
B pgm_read_byte(const B* const ptr) {
    return *ptr;
}

template<typename W>
W pgm_read_word(const W* const ptr) {
    return *ptr;
}

// https://www.cubeatsystems.com/tinymml/index.html
// https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML

// <note>|r|p|<|>|o|l[+-][<length>][.|..]
// note: cdefgab 
// r,p : pause
// o : Oktave
// l : default length
// length: 1, 2, 23, 4, 43, 8, 83, 16, 163, 32, 323, 64, 643
// < : octave up
// > : octave down
// loop: [ ... ]
// t120 : tempo

// volume setting: @v0 = {15 8 7 6 5}
// Idee: DAC vom tiny1614 benutzen
// ???: wie auf eine Notenlänge verteilen?

namespace External::MML {
    using namespace etl;

    namespace Bits {
        struct tone_tag;
        struct duration_tag;
        using tone_bits = etl::Bits<tone_tag, BitNumber<5>, BitPosition<0>>;
        using duration_bits = etl::Bits<duration_tag, BitNumber<3>, BitPosition<5>>;
        using note_t = etl::BitField<Meta::List<tone_bits, duration_bits>, std::byte>;
    }
    
    template <etl::Char... chars> struct Mml {};
    
    namespace literals {
        template <typename C, C... chars>
        requires (std::is_same_v<C, char>)
        constexpr Mml<etl::Char{chars}...> operator"" _mml() { 
            return {}; 
        }
    }

    struct Duration final {
        constexpr explicit Duration(const uint8_t denom = 4) : denominator{denom} {}
        constexpr void operator++() {
            numerator *= 2;
            denominator *= 2;
            numerator += 1;
        }
        constexpr void toTriplet() {
            denominator *= 3;
            denominator /= 2;
        }
        constexpr void add(const Char c) {
            uint8_t d = asDigit(c);
            if (duration_chars == 0) {
                denominator = d;
            }
            else if (duration_chars == 1) {
                if (d == 3) {
                    toTriplet();
                }
                else {
                    denominator = denominator * 10 + d;
                }
            }
            else if (duration_chars == 2) {
                if (d == 3) {
                    toTriplet();                        
                }
            }
            ++duration_chars;
        }
    private:
        uint8_t numerator{1};
        uint8_t denominator{4};
        uint8_t duration_chars{};
    };

    struct Octave final {
        constexpr Octave() = default;
        constexpr explicit Octave(uint8_t n) : number{n} {}
        constexpr void operator++() {
            if (number < 10) ++number;
        }
        constexpr void operator--() {
            if (number > 0) --number;
        }
        constexpr bool operator==(const Octave& rhs) {
            return number == rhs.number;
        }

    private:
        uint8_t number{4};
    };
    

    struct Tone final {
        inline static constexpr uint8_t rest_index = 255;
        inline static constexpr uint8_t repeat_index = 254;
        
        constexpr explicit Tone(const Char c = Char{'c'}, const Octave o = Octave{}) :
            scale_index{indexOfLetter(c)}, octave{o} {}
            
        constexpr void operator++() {
            if (scale_index == 255) return;
            if (scale_index < 11) {
                scale_index += 1;
            }
            else {
                scale_index = 0;
                ++octave;
            }
        }
        constexpr void operator--() {
            if (scale_index == 255) return;
            if (scale_index > 0) {
                scale_index -= 1;
            }
            else {
                scale_index = 11;
                --octave;
            }
        }

        inline static constexpr bool isLetterValid(const Char c) {
            return (std::end(letters) != std::find_if(std::begin(letters), std::end(letters),[&](auto i){
              return (i.first == c);}));
        }
        inline static constexpr uint8_t indexOfLetter(const Char c) {
            for(uint8_t i{}; i < letters.size(); ++i) {
                if (c == letters[i].first) {
                    return i;
                }
            }
            // flow of
        }
        inline static constexpr Tone repeat_mark() {
            return Tone{repeat_index};
        }
        constexpr uint8_t index() const {
            return scale_index;
        }
        constexpr bool operator==(const Tone& rhs) {
            return (scale_index == rhs.scale_index) && (octave == rhs.octave);
        }
    private:
        explicit constexpr Tone(const uint8_t i) : scale_index{i} {}
        using p = std::pair<Char, uint8_t>;
        inline static constexpr std::array letters {p{Char{'c'}, 0}, p{Char{'d'}, 2}, p{Char{'e'}, 4}, p{Char{'f'}, 5}, p{Char{'g'}, 7}, p{Char{'a'}, 9}, p{Char{'b'}, 11}, p{Char{'h'}, 11}, p{Char{'r'}, rest_index}, p{Char{'p'}, rest_index}};
        uint8_t scale_index{}; // 0 ... 11, 255 für Pause, 254 für Repeat
        Octave octave{}; // 0 ... 10
    };
    
    struct Note final {
        constexpr /*explicit */ Note(const Char c = Char{'c'}, const Duration d = Duration{}, const Octave o = Octave{}) :
            mTone{c, o}, mDuration{d} {}

//        constexpr bool isRepeatMark() const {
//            return (scale_index == repeat_index);
//        }
//        constexpr bool isPause() const {
//            return (scale_index == rest_index);
//        }
                
        constexpr void operator++() {
            ++mTone;
        }
        constexpr void operator--() {
            --mTone;
        }
        constexpr void addToDuration(const Char c) {
            mDuration.add(c);
        }
        constexpr void lengthen() {
            ++mDuration;
        }
        inline static constexpr Note repeat_mark() {
            return Note{Tone::repeat_mark()};
        } 
        constexpr Tone tone() const {
            return mTone;
        }
    private:
        constexpr explicit Note(const Tone t) : mTone{t} {}        
        Tone mTone;
        Duration mDuration; // to be multiplied with base-length
    };
    
    template<typename> struct Parser;
    
    template<etl::Char... CC>
    struct Parser<Mml<CC...>> final {
        inline static constexpr size_t max_size = 3 * sizeof...(CC);
        
        template<typename T>
        using vector_t = etl::Vector<T, max_size>;
        
//        inline static constexpr size_t byte_size() {
//            constexpr auto d = parse();
//            static_assert(d, "parse error");
//            if constexpr(d) {
////                std::integral_constant<size_t, d->size()>::_;
//                return d->size();
//            }
//            return 0; 
//        }
        
        inline static constexpr auto notes() {
            constexpr auto data = parse();
            static_assert(data, "parse error");
            if constexpr(data) {
                std::array<Note, data->size()> d;
                return d;            
            }
        };
        
        inline static constexpr std::optional<vector_t<Note>> parse() {
            vector_t<Note> notes;
            
            using s_t = vector_t<Note>::size_type;

            etl::stack<s_t, vector_t<s_t>> repeat_stack;
            
            enum class State {Start, Note, Length, Octave, Tempo};
            State state = State::Start;
            Octave actual_octave{4};
            Duration actual_duration{4};
            
            Note n;
            for(size_t i{}; i < mml.size(); ++i) {
                const Char cc{mml[i]};
                const Char c{etl::toLower(cc)};                              
                switch(state) {
                    case State::Start:
                    if (Tone::isLetterValid(c)) {
                        n = Note{c, actual_duration, actual_octave};
                        state = State::Note;
                    }
                    else if (c == Char{'l'}) {
                        state = State::Length;
                        actual_duration = Duration{};
                    }
                    else if (c == Char{'o'}) {
                        state = State::Octave;
                    }
                    else if (c == Char{'t'}) {
                        state = State::Tempo;
                    }
                    else if (c == Char{'<'}) {
                        ++actual_octave;
                    }
                    else if (c == Char{'>'}) {
                        --actual_octave;
                    }
                    else if (c == Char{' '}) {
                    }
                    else {
                        return {};
                    }
                    break;
                case State::Note:
                    if (Tone::isLetterValid(c)) {
                        notes.push_back(n);
                        n = Note{c, actual_duration, actual_octave};
                    }
                    else if (c == Char{'+'}) {
                        ++n;
                    }
                    else if (c == Char{'-'}) {
                        --n;
                    }
                    else if (isDigit(c)) {
                        n.addToDuration(c);
                    }
                    else if (c == Char{'.'}) {
                        n.lengthen();
                    }
                    else if (c == Char{' '}) {
                        state = State::Start;
                        notes.push_back(n);
                    }
                    else if (c == Char{'l'}) {
                        state = State::Length;
                        actual_duration = Duration{};
                        notes.push_back(n);
                    }
                    else if (c == Char{'o'}) {
                        state = State::Octave;
                        notes.push_back(n);
                    }
                    else if (c == Char{'<'}) {
                        state = State::Start;
                        ++actual_octave;
                        notes.push_back(n);
                    }
                    else if (c == Char{'>'}) {
                        state = State::Start;
                        --actual_octave;
                        notes.push_back(n);
                    }
                    else if (c == Char{'['}) {
                        state = State::Start;
                        notes.push_back(n);
                        repeat_stack.push(notes.size()); // actual position
                    }
                    else if (c == Char{']'}) {
                        state = State::Start;
                        notes.push_back(n);
//                        auto position = repeat_stack.top();
                        repeat_stack.pop();
                        notes.push_back(Note::repeat_mark());                        
                    }
                    else {
                        return {};
                    }
                    break;
                case State::Length:
                    if (isDigit(c)) {
                        actual_duration.add(c);
                    }
                    else if (Tone::isLetterValid(c)) {
                        n = Note{c, actual_duration, actual_octave};
                        state = State::Note;
                    }
                    else {
                        state = State::Start;
                    }
                    break;
                case State::Octave:
                    break;
                case State::Tempo:
                    if (isDigit(c)) {
                    }
                    else if (Tone::isLetterValid(c)) {
                        n = Note{c, actual_duration, actual_octave};
                        state = State::Note;
                    }
                    else {
                        assert(i > 0);
                        --i;
                        state = State::Start;
                    }
                    break;
                }                    
            }
            if (state == State::Note) {
                notes.push_back(n);
            }            
            return notes;
        }
        
//        inline static constexpr auto periods = []{
//            etl::Vector<size_t, byte_size()> p;
//            return p;
//        }();
        
//        inline static constexpr auto byte_data = []{
//            constexpr auto d = parse();
//            static_assert(d, "parse error");
//            if constexpr(d) {
//                std::array<External::MML::Bits::note_t, d->size()> data{};
//                for(size_t i{}; i < d->size(); ++i) {
//                    data[i].template set<External::MML::Bits::tone_bits>(std::byte{0x01});
//                    data[i].template set<External::MML::Bits::duration_bits>(std::byte{0x01});
//                }
//                return data;
//            }
//        }();
    private:        
        inline static constexpr std::array<etl::Char, sizeof...(CC)> mml{CC...};
    };
    
    template <typename, typename> struct Store;
    
    template <typename PWM, typename... MMs>
    struct Store<PWM, Meta::List<MMs...>> final {
        using parsers = Meta::List<Parser<MMs>...>;
        
        using size_type = etl::typeForValue_t<sizeof...(MMs)>;
        using melody_index_type = etl::uint_ranged<size_type, 0, (sizeof...(MMs))>;
        
        inline static constexpr size_type size() { // # of melodys
            return sizeof...(MMs);
        }
        
        inline static constexpr size_t note_size() { // total number of bytes of storage
            size_t sum{};
            Meta::visit<parsers>([&]<typename P>(Meta::Wrapper<P>){
                                     sum += P::notes().size();
                                 });
            return sum;
        }
        
        inline static constexpr auto tones() {
            etl::Vector<Tone, note_size()> tones;
            Meta::visit<parsers>([&]<typename P>(Meta::Wrapper<P>){
                                     for(const auto& n : P::notes()) {
                                         if (std::find(std::begin(tones), std::end(tones), n.tone()) == std::end(tones)) {
                                             tones.push_back(n.tone());
                                         }
                                     }
                                 });
            return tones;
        }
        
        struct Melody {
            inline constexpr explicit Melody(const size_t offset, const size_t s) : mOffset{offset}, mSize{s}{}
            inline constexpr External::MML::Bits::note_t operator[](const size_t index) const {
                return External::MML::Bits::note_t{pgm_read_byte(&data[mOffset + index])};
            }
            inline constexpr size_t size() const {
                return mSize;
            }
        private:
            const size_t mOffset;            
            const size_t mSize;            
        };
        inline static constexpr Melody melody(const melody_index_type m) {
            assert(m < size());
            size_t o = pgm_read_word(&offsets[m]);
            if (m < (size() - 1)) {
                size_t s = pgm_read_word(&offsets[m + 1]);
                return Melody{o, s};
            }
            else {
                size_t s1 = pgm_read_word(&offsets[m]);
                size_t s = note_size() - s1;
                return Melody{o, s};
            }
        }
//    private:
        inline static constexpr auto notes PROGMEM = []{
            etl::Vector<Note, note_size()> ns;
            
            Meta::visit<parsers>([&]<typename P>(Meta::Wrapper<P>){
                                     for(const auto& n : P::notes()) {
                                         ns.push_back(n);
                                     }
                                 });
            return ns;            
        }();
        inline static constexpr auto durations PROGMEM = []{
            
        }();
        inline static constexpr auto periods PROGMEM = []{
            
        }();
        
        inline static constexpr auto data PROGMEM = []{
            std::array<External::MML::Bits::note_t, note_size()> d{};
            auto it = d.begin();
            Meta::visit<parsers>([&]<typename P>(Meta::Wrapper<P>){
//                                     const auto& pd = P::byte_data;
//                                     std::copy(pd.begin(), pd.end(), it);
//                                     it += P::byte_size();
                                 });
            return d;
        }();
        inline static constexpr auto offsets PROGMEM = []{
            std::array<size_t, size()> offs{};
            auto it = offs.begin() + 1;
            Meta::visit<Meta::pop_back<parsers>>([&]<typename P>(Meta::Wrapper<P>){
                                                 *it++ = P::notes().size();                                
                                             });
            return offs;
        }();
    }; 
    
    
    template<typename P, auto Mml>
    using StoreV = Store<P, decltype(Mml)>;
    
    template<auto... Mmls>
    struct L;
}


using namespace External::MML;
using namespace External::MML::literals;

using s = decltype("L8 C D E F G A B > C"_mml);
using bach  = decltype("T90L16REA>C<BEB>DC8E8"_mml);
using w = decltype("C D E F G"_mml);

using l = L<"C D E F G"_mml, "L8 C D E F G A B > C"_mml>;

constexpr auto w1 = "C D E F G"_mml;
constexpr auto w2 = "C D E F G"_mml;
using l1 = L<w1, w2>;

using tonePwm = void;

using storev = StoreV<tonePwm, Meta::List<w>{}>;
using store = Store<tonePwm, Meta::List<w, s, bach>>;
//using store = Store<tonePwm, Meta::List<s, bach>>;

//std::integral_constant<size_t, store::byte_size()>::_;

int main() {
    std::cout << "Store size: " << (size_t)store::size() << '\n';
    std::cout << "Store note size: " << (size_t)store::note_size() << '\n';

    std::cout << "Store tonesize: " << (size_t)store::tones().size() << '\n';
    
    for(const auto& t : store::tones()) {
        std::cout << "t: " << (int)t.index() << '\n';
    }
    for(const auto& n : store::notes) {
        std::cout << "n: " << (int)n.tone().index() << '\n';
    }

    
//    for(const auto& b : store::data) {
//        std::cout << "b: " << (int)b.byte() << '\n';
//    }
//    for(const auto& b : store::offsets) {
//        std::cout << "o: " << (int)b << '\n';
//    }
    
//    for(store::melody_index_type m{}; m < store::size(); ++m) {
//        auto mel = store::melody(m);
//        for(size_t b{}; b < mel.size(); ++b) {
//            std::cout << "Melody[" << (size_t)m << "] {" << b << "}: " << (size_t)mel[b].byte() << '\n';
//        }
//    }
}

