// Test für guard variable 

#include <cstdint>

//struct A {
//    inline constexpr A(uint8_t v) : m{v} {} // without constexpr it should not compile, but does anymay
//    auto m1() const {
//        return m;
//    }
//private:
//     uint8_t m{0};
//};

//template<typename T>
//struct X {
//    static auto foo() {
//        return m.m1();
//    }
//    constinit inline static T m{2}; // requires constexpr ctor
//};
//int main() {
//    return X<A>::foo();    
//}

class Foo
{
public:
	Foo(int x) : x_(x) {}
	int x() { return x_; }

private:
	int x_;
};

void func()
{
	static Foo foo(42);
//	std::cout << "x = " << foo.x() << std::endl;
}

int main()
{
	func();
	return 0;
}
