#include <array>
#include <container/pgmarray.h>
#include <util/meta.h>

struct RightOpen;
struct LeftOpen;
struct Closed;
struct Open;

struct Adjacent;
struct Disjoint;

template<typename T, typename Type>
struct Boundary {
    constexpr Boundary(T value = 0): upper(value) {}
    typedef T value_type;
    constexpr bool includes(T v) const {
        if constexpr(std::is_same_v<Type, RightOpen>) {
            return (v < upper);
        }
        else {
            static_assert(Meta::always_false_v<T>, "other kinds not supported");
        }
    }
    static Boundary createFrom(const std::array<std::byte, sizeof(T)>& bytes) {
        if constexpr(std::is_same_v<T, uint8_t> && (bytes.size == 1)) {
            return Boundary(std::to_integer(bytes[0]));
        }
        else {
            static_assert(Meta::always_false<T>::value, "not implemented");
        }
    }
    T upper{};
};

template<typename T, typename Intervals, typename Function>
constexpr void lookup(const T& value, const Intervals& intervals, const Function& f) {
    for(typename Intervals::size_type n = 0; n < intervals.size; ++n) {
        if (intervals[n].includes(value)) {
            f(n);
            break;
        }
    }
}

struct Generator {
    typedef Boundary<uint8_t, RightOpen> boundary_type;
    inline static constexpr double rvcc = 10.0;
    inline static constexpr double rrow = 1.0;
    inline static constexpr double rcol = 3.9;
    inline static constexpr uint8_t nrow = 4;
    inline static constexpr uint16_t adcres = 256;
    inline static constexpr uint8_t nkey = 12;
    
    constexpr auto& operator()() const {
        return bounds;
    }

    inline static constexpr auto bounds = []{
        std::array<boundary_type, 11> bs;
        auto adcval = [](size_t i){
            return adcres * (1-rvcc/((i)%nrow*rrow+(i)/nrow*rcol+rvcc));
        };
        for(size_t i = 0; i < bs.size; ++i) {
            auto threshold = (uint16_t)((adcval(i)+adcval(i+1))/2);
            bs[i] = threshold;
        }
        return bs;
    }();
};

volatile uint8_t result = -1; 
volatile uint8_t value = 100;


int main() {
    auto mysum = [](auto... vv){return (vv + ... + 0);};
    result = mysum(1, 2, 5.0);
    
//    using lut = Util::Pgm::Converter<Generator>::pgm_type;
//    lookup(value, lut{}, [](auto index){result = index;});
}
