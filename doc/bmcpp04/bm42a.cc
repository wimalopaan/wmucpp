#include <util/types.h>
#include <array>

template<uint16_t Size>
struct std::array<uint4_t, Size> final {
    typedef uint4_t value_type;
    typedef typename std::conditional<(Size < 256), uint8_t, uint16_t>::type size_type;
    typedef typename std::conditional<(Size < 128), int8_t, int16_t>::type signed_size_type;
    static_assert(Size <= std::numeric_limits<uint16_t>::max());
    
    inline static constexpr size_type data_size = (Size / 2) + (Size % 2);

//    constexpr const value_type* begin() const {
//        return &data[0];
//    }
//    constexpr const value_type* end() const {
//        return &data[Size];
//    }
//    constexpr const volatile value_type* begin() const volatile {
//        return &data[0];
//    }
//    constexpr const volatile value_type* end() const volatile {
//        return &data[Size];
//    }
//    constexpr value_type* begin() {
//        return &data[0];
//    }
//    constexpr value_type* end() {
//        return &data[Size];
//    }
//    constexpr volatile value_type* begin() volatile {
//        return &data[0];
//    }
//    constexpr volatile value_type* end() volatile {
//        return &data[Size];
//    }
//    inline constexpr value_type& operator[](size_type index) {
//        assert(index < Size);
//        return data[index];
//    }
    struct Access_v {
         value_type& data;
        const bool upper;
        inline constexpr void operator=(uint8_t v){
            if (upper) {
                data.upper = v;
            }
            else {
                data.lower= v;
            }
        }
    };
    inline constexpr Access_v operator[](size_type index)  {
        assert(index < Size);
        return Access_v{data[size_type(index / 2)], bool(index % 2)};
    }
//    inline constexpr const volatile value_type& operator[](size_type index) const volatile {
//        assert(index < Size);
//        return data[index];
//    }
//    inline constexpr const value_type& operator[](size_type index) const {
//        assert(index < Size);
//        return data[index];
//    }
    inline static constexpr size_type size = Size;
    value_type data[data_size] = {}; 
};

std::array<uint8_t, 5> a1;
std::array<uint4_t, 5> a2;

int main() {
    
    []<auto... II>(std::index_sequence<II...>){
        ((a2[II] = II), ...);
    }(std::make_index_sequence<5>{});
    
}
