module;

#include <new>
#include <array>

export module modA;

namespace A {
	namespace detail {
        void f();
	}
}

export namespace A {
    inline void test();
}
 
