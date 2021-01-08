#include <iostream>
#include <vector>
#include <array>
#include <limits>

template<typename T>
class MyVector {
public:
    explicit MyVector(const std::initializer_list<T>&) {}
    
    using value_type = T;
    
    class Filler {
        friend class MyVector<T>;
    public:
        MyVector& with(const T&) {
            // ...
            return m;
        }
    private:
        explicit Filler(MyVector& v) : m{v} {}
        MyVector& m;
    };
    
    Filler fill(const size_t n) {
        reserve(n);
        return Filler(*this);
    }
    void reserve(const size_t n) {
        // ...
        mSize = n;
    }
private:
    size_t mSize{};
};

template<typename C>
class Filler {
public:
    explicit Filler(const size_t n) : mSize{n} {}
    C with(const typename C::value_type& v) {
        C c{};
        c.reserve(mSize);
        // ...
        return c;    
    }
private:
    const size_t mSize{};
};

template<typename C>
Filler<C> fill(const size_t n) {
    return Filler<C>{n};
}

class MyClass1 {
};
class MyClass2 {
public:
    MyClass2(int) {}
};
class MyClass3 {
public:
    explicit MyClass3(int) {}
};
class MyClass4 {
    
};

template<typename T>
struct Foo {
    T f() {
        T m{};  
        return m;
    }
};

int main() {
    Foo<int> f;
    std::cout << f.f() << '\n';
    
    {
        std::vector<int> v1(); // ups
        std::vector<int> v2(3);
        std::vector<int> v3(3, 2); // ist das [2, 2, 2] oder [3, 3] ?
        std::vector<int> v4(2, 3);    
//        std::vector<int> v5(1, 2, 3);  // warum geht das nicht? 
    }
    {
        std::vector<int> v1{};
        std::vector<int> v2{3};
        std::vector<int> v3{3, 2};
        std::vector<int> v4{2, 3};    
        std::vector<int> v5{1, 2, 3};    
    }
    {
        std::array<int, 10> a1; // warum uninitialisierte Elemente?
//        std::array<int, 10> a2(3); // warum geht das nicht?
    }
    {
        std::array<int, 10> a1{};
        std::array<int, 10> a2{3};
        std::array<int, 10> a3{3, 2};
        std::array<int, 10> a4{1, 2, 3};
    }
    {
        int i1(); // ups
        int i2(4); // s.o. Konsistenz
    }
    {
        int i1{}; 
        int i2{4};
    }
    {
        std::vector<MyClass1> v1;
    }    
    {
        std::vector<MyClass1> v2{3};
    }    
    {
//        MyVector<MyClass1> v2(3);        
    }    
    {
//        MyVector<MyClass1> v2{3};
    }    
    {
//        MyVector<MyClass2> v2(3);        
    }    
    {
        MyVector<MyClass2> v2{3};
    }    
    {
        auto v1 = MyVector<MyClass2>{}.fill(10).with(MyClass2{3});
        auto v2 = MyVector<int>{}.fill(10).with(1);
        
        auto v3 = fill<MyVector<MyClass2>>(10).with(MyClass2{3});
    }
#if 0
    
  // use uniform initialization

    std::vector<MyClass1> v1{MyClass1{1}};
  std::vector<MyClass2> v2{2};

  std::cout << v1.size() << '\n';
  std::cout << v2.size() << '\n';

  // use classic initialization

//  std::vector<MyClass3> v3={2}; // initlist
  std::vector<MyClass4> v4(2);

//  std::cout << v3.size() << '\n';
  std::cout << v4.size() << '\n';


  MyVector<MyClass1> v10{2}; // 
  MyVector<MyClass2> v20{2};

  std::cout << v10.size() << '\n';
  std::cout << v20.size() << '\n';

  // use classic initialization

//  std::vector<MyClass3> v3={2}; // initlist
  MyVector<MyClass4> v40(2);

//  std::cout << v3.size() << '\n';
  std::cout << v40.size() << '\n';
#endif
}
