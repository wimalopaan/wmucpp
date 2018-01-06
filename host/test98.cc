template<typename Updater>
class EdgeDetector {
public:
    EdgeDetector(Updater f) : mF{f}{}
    template<typename A = void>
    void update(const A& arg) {
        mF(arg);
    }
    void update() {
        mF();
    }
private:    
    Updater mF;
};


int main() {
    auto s1 = EdgeDetector{[]{}};
    s1.update();
    
    auto s2 = EdgeDetector{[](int){}};
    s2.update(1);
    
}
