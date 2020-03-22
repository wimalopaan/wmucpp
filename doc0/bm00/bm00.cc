struct A {
	A() {}
    volatile int m{42};
};


namespace {
    int  g() {
        static A a; // guards generated
        return a.m;
    }
}

int main() {
	return g();
}
