#include <cassert>
#include <cstdint>
#include <type_traits>
#include <limits>
#include <algorithm>

#define V1

#define NDEBUG

namespace detail {
    template<typename T> struct allways_false : std::false_type {};
    template<typename T> struct enclosing_type {static_assert(allways_false<T>::value, "No enclosing type available");};
    template<> struct enclosing_type<int8_t> {using type = int16_t;};
    template<> struct enclosing_type<int16_t> {using type = int32_t;};
    template<> struct enclosing_type<int32_t> {using type = int64_t;};
}

template<typename T>
using enclosing_t = typename detail::enclosing_type<T>::type;

template<typename T>
struct Width {
    constexpr explicit Width(T v) : value{v} {}
    const T value;
};
template<typename T>
struct Left {
    constexpr explicit Left(T v) : value{v} {}
    const T value;
};

#ifdef V1
template<typename T>
struct ClosedInterval {
    using value_type = enclosing_t<T>;

    explicit constexpr ClosedInterval(T left) : mLower{left} {}
    
    explicit constexpr ClosedInterval(Left<T> left, Width<T> width) : mLower{left.value}, mUpper{value_type{left.value} + width.value} {
        if (mLower >= mUpper) {
            using std::swap;
            swap(mLower, mUpper);
        }
        assert(mLower <= mUpper);
    }
    explicit constexpr ClosedInterval(Width<T> width, Left<T> left) : mLower{left.value}, mUpper{value_type{left.value} + width.value} {
        if (mLower >= mUpper) {
            using std::swap;
            swap(mLower, mUpper);
        }
        assert(mLower <= mUpper);
    }

    const ClosedInterval& width(T w) {
        mUpper = mLower + w;
        if (mLower >= mUpper) {
            using std::swap;
            swap(mLower, mUpper);
        }
        return *this;
    }
    constexpr bool contains(T value) const {
        assert(mLower <= mUpper);
        return (value >= mLower) && (value <= mUpper);
    }
private:
    value_type mLower{0};
    value_type mUpper{0};
};

template<>
struct ClosedInterval<int64_t> {
    using T = int64_t;
    using value_type = T;
    using unsigned_type = typename std::make_unsigned<T>::type;

    explicit constexpr ClosedInterval(T left) : mLower{left} {}
    
    explicit constexpr ClosedInterval(Left<T> left, Width<T> width) : mLower{left.value}, mWidth{width.value} {}
    
    explicit constexpr ClosedInterval(Width<T> width, Left<T> left) : mLower{left.value}, mWidth{width.value} {}

    const ClosedInterval& width(T w) {
        mWidth = w;
        return *this;
    }
    constexpr bool contains(T value) const {
        unsigned_type c_a = unsigned_type(value) - unsigned_type(mLower);
    
        if (mWidth >= 0 && value >= mLower)
            return c_a <= unsigned_type(mWidth);
    
        if (mWidth <= 0 && value <= mLower)
            return -c_a <= - unsigned_type(mWidth);
        
        return false;
    }
private:
    value_type mLower{0};
    value_type mWidth{0};
};

#else

template<typename T>
struct ClosedInterval {
    using value_type = T;
    using unsigned_type = typename std::make_unsigned<T>::type;

    explicit constexpr ClosedInterval(T left) : mLower{left} {}
    
    explicit constexpr ClosedInterval(Left<T> left, Width<T> width) : mLower{left.value}, mWidth{width.value} {}
    
    explicit constexpr ClosedInterval(Width<T> width, Left<T> left) : mLower{left.value}, mWidth{width.value} {}

    const ClosedInterval& width(T w) {
        mWidth = w;
        return *this;
    }
    constexpr bool contains(T value) const {
        unsigned_type c_a = unsigned_type(value) - unsigned_type(mLower);
    
        if (mWidth >= 0 && value >= mLower)
            return c_a <= unsigned_type(mWidth);
    
        if (mWidth <= 0 && value <= mLower)
            return -c_a <= - unsigned_type(mWidth);
        
        return false;
    }
private:
    value_type mLower{0};
    value_type mWidth{0};
};

#endif

bool in_range2(int16_t c, int16_t a, int16_t b) {
    using T = int16_t;
    using U = typename std::make_unsigned<T>::type;

    U c_a = (U) ((U) c - (U) a);

    if (b >= 0 && c >= a)
        // We have  0 <= c - a < 2^N  and  0 <= b < 2^{N-1} for N-bit
        // arithmetic and type T.  Hence the following comparison is
        // just  c - a <= b   i.e.  c <= a + b.
        return c_a <= (U) b;

    if (b < 0 && c <= a)
        // We have  0 <= a - c < 2^N  and  0 < -b <= 2^{N-1}.  Hence the
        // following comparison is just  -c + a <= -b   i.e.  c >= a + b.
        return (U) -c_a <= (U) - (U) b;
    
    return false;
}

bool in_range3(int32_t c, int32_t a, int32_t b) {
    using T = int32_t;
    using U = typename std::make_unsigned<T>::type;

    U c_a = (U) ((U) c - (U) a);

    if (b >= 0 && c >= a)
        // We have  0 <= c - a < 2^N  and  0 <= b < 2^{N-1} for N-bit
        // arithmetic and type T.  Hence the following comparison is
        // just  c - a <= b   i.e.  c <= a + b.
        return c_a <= (U) b;

    if (b < 0 && c <= a)
        // We have  0 <= a - c < 2^N  and  0 < -b <= 2^{N-1}.  Hence the
        // following comparison is just  -c + a <= -b   i.e.  c >= a + b.
        return (U) -c_a <= (U) - (U) b;
    
    return false;
}

template <typename T>
bool in_range (T c, T a, T b) {
    static_assert (std::is_integral<T>(), "T must be an integer type");
    static_assert (std::is_signed<T>(), "T must be signed");
    using U = typename std::make_unsigned<T>::type;

    U c_a = (U) ((U) c - (U) a);

    if (b >= 0 && c >= a)
        // We have  0 <= c - a < 2^N  and  0 <= b < 2^{N-1} for N-bit
        // arithmetic and type T.  Hence the following comparison is
        // just  c - a <= b   i.e.  c <= a + b.
        return c_a <= (U) b;

    if (b < 0 && c <= a)
        // We have  0 <= a - c < 2^N  and  0 < -b <= 2^{N-1}.  Hence the
        // following comparison is just  -c + a <= -b   i.e.  c >= a + b.
        return (U) -c_a <= (U) - (U) b;
    
    return false;
}

template<typename T>
struct Interval {
    constexpr inline Interval(Left<T> left, Width<T> width) : mLeft{left.value}, mWidth{width.value} {}
    constexpr inline T left() const {
        return mLeft;
    }
    constexpr inline T width() const {
        return mWidth;
    }
private:
    const T mLeft = 0;
    const T mWidth = 0;
};


template <typename T>
inline bool in_range (T c, Interval<T> interval) {
    static_assert (std::is_integral<T>(), "T must be an integer type");
    static_assert (std::is_signed<T>(), "T must be signed");
    using U = typename std::make_unsigned<T>::type;

    U c_a = (U) ((U) c - (U) interval.left());

    if (interval.width() >= 0 && c >= interval.left())
        // We have  0 <= c - a < 2^N  and  0 <= b < 2^{N-1} for N-bit
        // arithmetic and type T.  Hence the following comparison is
        // just  c - a <= b   i.e.  c <= a + b.
        return c_a <= (U) interval.width();

    if (interval.width() < 0 && c <= interval.left())
        // We have  0 <= a - c < 2^N  and  0 < -b <= 2^{N-1}.  Hence the
        // following comparison is just  -c + a <= -b   i.e.  c >= a + b.
        return (U) -c_a <= (U) - (U) interval.width();
    
    return false;
}


int main() {
    using value_type = int32_t;

    {
        value_type a{std::numeric_limits<value_type>::max() - 10};
        value_type b{20};
        volatile value_type c{a + 5};
        
//        auto i = ClosedInterval{Left{a}, Width{b}}; 
//        return  i.contains(c) + i.contains(c);
        
        return in_range3(c, a, b) + in_range3(c, a, b);
//        return in_range(c, Interval{Left{a}, Width{b}}) + in_range(c, Interval{Left{a}, Width{b}});
    }
    {
        value_type a{std::numeric_limits<value_type>::max() - 10};
        value_type b{20};
        volatile value_type c{a + 5};
        
    }
    {
        value_type a{std::numeric_limits<value_type>::max() - 10};
        value_type b{1};
        volatile value_type c{a + 5};
        
    }

    {
        value_type a{std::numeric_limits<value_type>::min() + 10};
        value_type b{-20};
        volatile value_type c{a - 5};
    
    }
    {
        value_type a{std::numeric_limits<value_type>::min() + 10};
        value_type b{-20};
        volatile value_type c{a - 5};
    
    }
    
}
