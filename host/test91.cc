#include <cstdint>
#include <type_traits>

#include <iostream>

#if 1

template<template<typename,typename> typename D, typename V, typename T>
struct A {
    using Derived = D<V,T>;
    using value_type = V;
    
    constexpr explicit A(value_type _value) : value{_value} {}

    constexpr value_type count() const {
        return value;
    }

    Derived& operator+=(const Derived& rhs) {
        value += rhs.value;
        Derived* p = static_cast<Derived*>(this);
        return *p;
    }
protected:
    value_type value;
};

template<template<typename, typename> typename C, typename U, typename V,
         typename = std::enable_if_t<std::is_base_of_v<A<C, U, V>, C<U, V>>> >
C<U, V> operator+( C<U, V> lhs, const C<U, V>& rhs ) {
    static_assert(std::is_base_of_v<A<C, U, V>, C<U, V>>);
    lhs += rhs;
    return lhs;
}

template<typename T, typename U>
struct B : public A<B, T, U> {
    using parent_type = A<B, T, U>;
    constexpr explicit B(T _value) : parent_type{_value} {}
};

template<typename T, typename U>
struct C : public A<C, T, U> {
    using parent_type = A<C, T, U>;
    constexpr explicit C(T _value) : parent_type{_value} {}
};

#else
template <typename T, typename U>
struct is_template_same : std::false_type {};

template <template<typename...> typename T, typename... U, typename... V>
struct is_template_same<T<U...>, T<V...>> : std::true_type {};

template <typename T, typename U>
inline constexpr bool is_template_same_v = is_template_same<T, U>::value;

template<typename C>
struct A;

template<template<typename, typename> typename C, typename U, typename V>
struct A<C<U, V>> {
    using Derived = C<U,V>;
    
    constexpr explicit A(U _value) : value{_value} {}
    constexpr U count() const {
        return value;
    }

//    template<
//        template<typename, typename> typename oC, typename oU, typename oV 
//        typename = std::enable_if_t<is_template_same_v<C<U, V>, oC<oU, oV>>>
//    >
    Derived& operator+=(const Derived& rhs) {
        value += rhs.count();
        Derived* p = static_cast<Derived*>(this);
        return *p;
    }
protected:
    U value;
};

template<
    template<typename, typename> typename C, template<typename, typename> typename oC, 
    typename U, typename V, typename oU, typename oV, 
    typename = std::enable_if_t<is_template_same_v<C<U, V>, oC<oU, oV>>>
>
C<U, V> operator+( C<U, V> lhs, const oC<oU, oV>& rhs ) {
    static_assert(std::is_base_of_v<A<C<U, V>>, C<U, V>>);
    lhs += rhs;
    return lhs;
}

template<typename T, typename U>
struct B : public A<B<T, U>> {
    using parent_type = A<B<T, U>>;
    constexpr explicit B(T _value) : parent_type{_value} {}
    //Funktioniert, jedoch in jeder Kind Klasse nötig:
    /*B<T, U>& operator+=(const B<T, U>& rhs) {
        parent_type::operator+=(rhs);
        return *this;
    }*/
};

template<typename T, typename U>
struct C : public A<C<T, U>> {
    using parent_type = A<C<T, U>>;
    constexpr explicit C(T _value) : parent_type{_value} {}
};
#endif

int main() {
    B<uint16_t, uint32_t> b1(10);
    B<uint16_t, uint32_t> b2(20);
    B<uint16_t, uint32_t> b3(30);
    C<uint16_t, uint32_t> c1(40);
    b1 += b2;
    b1 += b3;
    b1 = b1 + b2 + b3;
    b1 += b2 += b3; 
  
//    b1 += c1;
    
    std::cout << b1.count() << '\n';
}
