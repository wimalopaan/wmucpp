#define NDEBUG

#include <cassert>
#include <cstdint>
#include <tuple>
#include "util/tuple.h"
#include "util/concepts.h"
#include "container/tree.h"

#ifndef NDEBUG
#include "console.h"
#include "simavr/simavrdebugconsole.h"
using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;
#endif

template<auto... II, Util::Callable L>
constexpr auto inode_to_indexnode(std::index_sequence<II...>, const L& callable) {
    constexpr auto inode = callable();
    static_assert(isInode(inode), "use a collable retuning an INode<>");
    typedef typename decltype(inode)::type dataType;
    return IndexNode<dataType, Index<inode.mNumber>, ParentIndex<inode.mParent>, inode.mChildren[II]...>{inode.mData};
}

template<Util::Callable L>
constexpr auto transform(const L& callable) {
    constexpr auto tuple = callable();
    static_assert(Util::isTuple(tuple), "use constexpr callabe returning a tuple");
    
    if constexpr(Util::size(tuple) == 0) {
        return std::tuple<>();
    }
    else {
        constexpr auto first = std::get<0>(tuple);    
//        constexpr auto rest = [&]{return Util::tuple_rest(tuple);}; // gcc-8.0 Problem 
        
        if constexpr(isInode(first)) {
            constexpr auto indexnode = inode_to_indexnode(std::make_index_sequence<first.mChildren.size>{}, [&]{return first;});
            return std::tuple_cat(std::tuple(indexnode), transform([&]{return Util::tuple_tail(tuple);}));        
        }
        else {
            return std::tuple_cat(std::tuple(first), transform([&]{return Util::tuple_tail(tuple);}));
        }
    }
}

struct A {
    constexpr A(uint8_t v = 0) : mValue(v) {}
    constexpr auto v() const {return mValue;}
    constexpr void v(uint8_t v) {mValue = v;}
    uint8_t mValue = 0;
};

namespace  {
    constexpr auto flat_tree = []{
        constexpr auto tree = Node(A(5), 
                                   A(4),
                                   Node(A(3),
                                        A(2), 
                                        A(1)
                                        )
                                   );
        constexpr auto ftree = make_tuple_of_tree(tree);
        return ftree;
    }();
    
    auto tree_list = transform([&]{return flat_tree;}); 
}

volatile uint8_t value = 42;

template<typename... T, typename C>
void traverse(std::tuple<T...>& tree, uint8_t start, const C& f) {
    Meta::visitAt(tree, start, [&](auto& item){
        if constexpr(isIndexNode(item)) {
            for(auto c: children(item)) {
                traverse(tree, c, f);
            }
            f(item.mData);
        }        
        else {
            f(item);
        }
        return 0;
    });
}

int main() {
    uint8_t sum = 0;

//    tree_list._;
    
    traverse(tree_list, Util::size(tree_list) - 1, [&](auto& item) {
        sum += item.v();
        item.v(value); // make sure runtime-code is generated
    });
#ifndef NDEBUG
    std::outl<terminal>(sum);
#endif
    return sum;
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif
