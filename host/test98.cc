#include <iostream>
#include <algorithm>
#include <cstddef>
#include <vector>
#include <array>
#include <cassert>

template<typename IndexType, typename ItemType, IndexType Size>
struct Test {
    using size_type = IndexType;
    size_type size() const {
        return Size;
    }
    ItemType& operator[](size_type) {
        return dummy;
    }
    const ItemType& operator[](size_type) const {
        return dummy;
    }
    ItemType dummy{0};
};


template<typename T1, typename T2>
struct Smaller {
    using type = typename std::conditional<std::numeric_limits<T1>::max() <= std::numeric_limits<T2>::max(), T1, T2>::type;
};
template<typename T1, typename T2>
struct Bigger {
    using type = typename std::conditional<std::numeric_limits<T1>::max() <= std::numeric_limits<T2>::max(), T2, T1>::type;
};

template<typename T>
void showType() {
    std::cout << __PRETTY_FUNCTION__ << '\n';
}

template <typename C1, typename C2>
void test (C1& c1, const C2& c2, typename C1::size_type off1, typename C2::size_type off2) {
    using index1_type = typename C1::size_type;
    using index2_type = typename C2::size_type;
    static_assert(std::is_unsigned<index1_type>::value);
    static_assert(std::is_unsigned<index2_type>::value);
    using index_type = typename Smaller<index1_type, index2_type>::type;
    using enclosing_type = typename Bigger<index1_type, index2_type>::type;
    
    assert(off1 <= c1.size());
    assert(off2 <= c2.size());
    
#ifndef NDEBUG
    showType<index_type>();
    showType<enclosing_type>();
#endif
    
    enclosing_type count = std::min(enclosing_type(c1.size() - off1), enclosing_type(c2.size() - off2));
    
    assert(count <= std::numeric_limits<index_type>::max());
    assert((count + off1) <= std::numeric_limits<index_type>::max());
    assert((count + off2) <= std::numeric_limits<index_type>::max());
    assert((count + off1) <= c1.size());
    assert((count + off2) <= c2.size());
    
    for (index_type i = 0; i < index_type(count); ++i) {
        std::cout << "Kopiere Element " << std::to_string(i) << std::endl;
        c1[index1_type(i + off1)] = c2[index2_type(i + off2)];
    }
}

int main () {
    {
        std::vector<int> c1(33);
        std::array<int, 42> c2;
    
        test(c1, c2, 3, 7);
    }
    {
        std::vector<int> c1(512);
        Test<uint8_t, int, 250> c2;
    
        test(c2, c1, 17, 7);
    }

}
