template<typename T = char>
class X {
public:
    X(double) {}
//private:
    X(int) {}
    
};


int main() {
    X x(1);
}
