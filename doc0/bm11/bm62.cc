#include <array>
#include <bit>
#include <cstring>

template<typename T>
struct A {
    T e0;
    T e1;
    T e2;
};
template<typename T>
struct B {
    T e0;
    T e1;
    T e2;
};

template<typename T>
constexpr A<T> asA(const B<T>& v){
    return std::bit_cast<A<T>>(v);
}

template<typename T>
union AB {
    A<T> a;
    B<T> b;
};

template<typename T>
union AB2 {
    struct {
        T e0;  
    } a;  
    struct {
        T e0;  
    } b;  
};

union U {
    int32_t a;
    int16_t b[2];
};

constexpr B<char> b{1, 2, 3};

template<typename T>
constexpr char test() {
    AB<T> ab;
    ab.a.e0 = 1; 
    return ab.b.e0;
}

consteval char test2() {
    U u;
    u.a = 1;
    return u.b[0];
}


enum event_type
{
    mouse_move, mouse_click
};
struct event
{
    event_type type;
};
struct move_event
{
    event_type type;
    int x, y;
};
struct click_event
{
    event_type type;
    int button;
};
union event_union
{
    event e;
    move_event move;
    click_event click;
};

constexpr int handle_event(const event_union* u)
{
    switch (u->e.type) // Zugriff über anderes Element
    {
        case mouse_move:
            // handle u.move
            break;
        case mouse_click:
            // handle u.click
            break;
    }
    return 0;
}




int main() {
//    constexpr event_union e {.click = {event_type::mouse_click, 1}};
//    constexpr auto r = handle_event(&e); 
    
    
//    constexpr auto t1 = test<char>();    
//    auto t2 = test2();    
    constexpr auto x = asA(b);
    return x.e0 + x.e1 + x.e2;

    
    //    return b.e0 + b.e1 + b.e2;
    
//    A<char> x;
//    std::memcpy(&x, &b, sizeof(x));
//    return x.e0 + x.e1 + x.e2;
}
