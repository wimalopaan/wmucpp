#include <cstdint>
#include <tuple>
#include <cassert>
#include <iostream>
#include <type_traits>
#include "container/tree.h"
#include "util/concepts.h"

template<typename D>
struct MenuItem {
    const int v = 0;
};

struct A : MenuItem<A> {
    constexpr A(int v = 0) : MenuItem<A>{v} {}
};
std::ostream& operator<<(std::ostream& out, const A& x) {
    return out << "A[" << x.v << "]";
}
struct B : MenuItem<B> {
    constexpr B(int v = 0) : MenuItem<B>{v} {}
};
std::ostream& operator<<(std::ostream& out, const B& x) {
    return out << "B[" << x.v << "]";
}
struct C : MenuItem<C> {
    constexpr C(int v = 0) : MenuItem<C>{v} {}
};
std::ostream& operator<<(std::ostream& out, const C& x) {
    return out << "C[" << x.v << "]";
}
struct D : MenuItem<D> {
    constexpr D(int v = 0) : MenuItem<D>{v} {}
};
std::ostream& operator<<(std::ostream& out, const D& x) {
    return out << "D[" << x.v << "]";
}
struct E : MenuItem<E> {
    constexpr E(int v = 0) : MenuItem<E>{v} {}
};
std::ostream& operator<<(std::ostream& out, const E& x) {
    return out << "E[" << x.v << "]";
}

struct Menu {
    constexpr Menu(int v) : mTitle(v) {}
    const int mTitle;
};

std::ostream& operator<<(std::ostream& out, const Menu& m) {
    return out << "Menu[" << m.mTitle << "]";
}

template<::detail::isNonTerminal T>
std::ostream& operator<<(std::ostream& out, const T& n) {
    out << "IN[";
    for(const auto& v : n.mChildren) {
        out << (int)v << ',';
    }
    out << n.mData;
    return out << "]";
}


auto flat_tree = [&]{
    constexpr auto tree = Node(Menu(1)
                               , A(7)
//                               Node(Menu(2),
//                                    B(6), 
//                                    C(5)
//                                    )
                               );

    constexpr auto ftree = make_tuple_of_tree(tree);
    return ftree;
};

template<auto... II, Util::Callable L>
constexpr auto inode_to_indexnode(std::index_sequence<II...>, const L& callable) {
    constexpr auto inode = callable();
    static_assert(isInode(inode), "us a collable retuning an INode<>");
    typedef typename decltype(inode)::type dataType;
    return IndexNode<dataType, Index<inode.mNumber>, ParentIndex<inode.mParent>, inode.mChildren[II]...>{inode.mData};
}


template<Util::Callable L>
//template<typename  L>
constexpr auto transform(const L& callable) {
    constexpr auto tuple = callable();
    static_assert(Util::isTuple(tuple), "use constexpr callabe returning a tuple");
    
    if constexpr(Util::size(tuple) == 0) {
        return std::tuple<>();
    }
    else {
        constexpr auto first = std::get<0>(tuple);    
        auto rest = []{return Util::tuple_tail(tuple);};
//        decltype(first)::_;
        
        if constexpr(isInode(first)) {
            constexpr auto indexnode = inode_to_indexnode(std::make_index_sequence<first.mChildren.size()>{}, [&](){return first;});
//            decltype(rest())::_;
            return std::tuple_cat(std::tuple(indexnode), transform(rest));        
        }
        else {
            return std::tuple_cat(std::tuple(first), transform(rest));
        }
    }
}

constexpr auto t2 = transform(flat_tree); 

//decltype(t2)::_;

template<typename T>
void inspect(T) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
}

int main() {
    inspect(flat_tree());
    inspect(t2);
    
    uint8_t sum = 0;
    
    constexpr auto topIndex = std::tuple_size<decltype(t2)>::value - 1;
    
    for(int n : children(std::get<topIndex>(t2))) {
        std::cout << n << '\n';
    }
    
    for(uint8_t i = 0; i < std::tuple_size<decltype(t2)>::value; ++i) {
        Meta::visitAt(t2, i, [](auto i) {
            inspect(i);
            return 0;
        });
    }
}
