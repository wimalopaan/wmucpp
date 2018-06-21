#include <cstddef>
#include <array>
#include <tuple>
#include <iostream>

template<typename C, C... CC>
constexpr auto operator"" _bytes(){
    return std::array<std::byte, sizeof...(CC)>{std::byte(CC)...};
}

constexpr std::byte operator"" _B(char c) {
    return std::byte(c);
}
constexpr std::byte operator"" _B(unsigned long long int c) {
    return std::byte(c);
}

template<typename U, auto L>
constexpr auto make_array(const char (&array)[L]) {
    return [&]<auto... II>(std::index_sequence<II...>) {
            std::array<std::byte, L-1> vv{U(array[II])...};
            return vv;
    }(std::make_index_sequence<L-1>{});
}

template<auto Length>
void tft_wr(const std::array<std::byte, Length>&) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
    std::cout << "len=" << Length << '\n';
}

template<auto Length>
void tft_wr(const uint8_t(&array)[Length]) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
    std::cout << "len=" << (Length - 1) << '\n';
}

template<char F, char S, auto Size>
struct Command {
    template<typename... VV>
    constexpr Command(VV... vv) : data{vv...} {}
    std::array<std::byte, Size> data;  
};

template<char F, char S, typename... VV>
constexpr auto command(VV... vv) {
    return Command<F, S, sizeof...(VV)>(vv...);
}

template<typename Commands>
void tft_write(const Commands& commands) {
    std::apply([](auto... cc){
        (..., [](auto c){}(cc));
        // Ausgabe auf Schnittstelle
    }, commands);
    
    [&]<auto... II>(std::index_sequence<II...>) {
        ([&](auto c){
            
        }(std::get<II>(commands)), ...);
    }(std::make_index_sequence<std::tuple_size<Commands>::value>{});
}

int main() {
    auto c1 = Command<'T', 'C', 2>{std::byte{8}, std::byte{1}};
    auto c2 = Command<'G', 'CD', 1>{std::byte{1}};
    
    auto c3 = command<'T', 'C'>(std::byte{8}, std::byte{1});
    
    auto sequence = std::tuple(c1, c2);
    
    constexpr auto init_sequence3 = std::make_tuple(
                command<'T', 'C'>(0_B, 1_B),
                command<'D', 'O'>(2_B)
                );
    
    tft_write(init_sequence3);
    
    constexpr auto init_sequence =
            "\x1bTC\0"              // text cursor off
            "\x1b""DO\2"            // rotate 180°
            "\x1b""FD\x8\x1"        // display color
            "\x1b""FZ\x8\x1"        // text color
            "\x1b""DL"              // display clear
            "\x1b""YZ\x0"           // no delay
            "\x1b""YH\x64"_bytes    // light on
            ;
    
    constexpr auto init_sequence2 = make_array<std::byte>(
                "\x1bTC\0"              // text cursor off
                "\x1b""DO\2"            // rotate 180°
                "\x1b""FD\x8\x1"        // display color
                "\x1b""FZ\x8\x1"        // text color
                "\x1b""DL"              // display clear
                "\x1b""YZ\x0"           // no delay
                "\x1b""YH\x64"          // light on
                );

    tft_wr(init_sequence2);        
    
    constexpr uint8_t init_string[] =
        "\x1bTC\0"              // text cursor off
        "\x1b""DO\2"            // rotate 180°
        "\x1b""FD\x8\x1"        // display color
        "\x1b""FZ\x8\x1"        // text color
        "\x1b""DL"              // display clear
        "\x1b""YZ\x0"           // no delay
        "\x1b""YH\x64"          // light on
        ;

    tft_wr(init_string);        
    
}

