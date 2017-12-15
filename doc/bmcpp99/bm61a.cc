#include <cstdint>
#include <cstddef>
#include <utility>

#include "mcu/avr8.h"
#include "mcu/register.h"
#include "hal/flag.h"

#include "util/meta.h"
#include "util/type_traits.h"

using flagRegister = AVR::RegisterFlags<typename DefaultMcuType::GPIOR, 0, std::byte>;

// Meta-Funktion: template -> type, bspw: X -> X<Flag<FR, v>>
template<typename FR, template<typename> typename X, size_t v>
struct MakeNumberedFlag {
    typedef X<Hal::Flag<FR, v>> type;  
};

template<typename FlagRegister, template<typename> typename ... X>
struct Controller {
    static_assert(sizeof...(X) <= 8, "too much ressources");
    // template-Liste der Ressourcen-Templates
    using ressourceList = Meta::TList<X...>;
    static_assert(Meta::is_set_T<ressourceList>::value, "all ressources must be different");
    
    // Meta-Funktion: bspw: T -> T<Flag<FlagRegister, v>>
    template<template<typename> typename T, size_t v>
    using makeFlags = MakeNumberedFlag<FlagRegister, T, v>;
    
    // umwandeln der template-liste in eine Typen-Liste
    // bspw: TList<A, B> -> List<A<Flag<FlagRegister, 0>>, B<Flag<FlagRegister, 1>>>
    using numberedRessouceList = typename Meta::transformN_T<makeFlags, ressourceList>::type;

    // Metafunction: liefert den Typ der parametrierten Ressource und deren Index im FlagRegister    
    template<template<typename> typename T>
    struct get {
        // der index des templates T in der template-liste
        static constexpr auto index = Meta::index_T<ressourceList, T>::value;
        // der zugehörige Typ in der Typen-Liste mit demselben index
        typedef Meta::nth_element<index, numberedRessouceList> type;
    };
};

// Beispiel einer Ressource, die ein Flag benötigt
template<typename T, typename Flag>
struct A final {
    A() = delete;
    static void f(){
        Flag::set();
    }
    inline static T mData{};
};

// Beispiel einer Ressource, die ein Flag benötigt
template<typename Flag, typename T>
struct B final {
    B() = delete;
    static void f() {
        if (Flag::isSet()) { 
            Flag::reset();
        }
    }
    inline static T mData{};
};

// für den Controller müssen die zu parametrierenden Typen genau einen Parameter haben, das Flag
// diese Meta-Funktionen bildet den Typ A<...> auf AF<Flag> für den Controller ab
template<typename Flag>
using AF = A<uint8_t, Flag>;

// diese Meta-Funktionen bildet den Typ B<...> auf BF<Flag> für den Controller ab
template<typename Flag>
using BF = B<Flag , uint16_t>;

// registrieren der Ressourcen Typen (templates) im Controller
using controller = Controller<flagRegister, AF, BF>;

// ermitteln der durch den Controller konkret erzeugten Typen
using a = controller::get<AF>::type;
using b = controller::get<BF>::type;

int main() {
    a::f();
    b::f();
}