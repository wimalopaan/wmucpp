#include <cstdint>
#include <cstddef>
#include <cstdio>

namespace Debug {
    namespace Parameter {
        constexpr bool on = false;    
    }

    template<bool On> struct Printer;
    template<> struct Printer<true> {
        template<typename... VV>
        void operator()(VV... vv){
            std::printf(vv...);
        }
    };
    template<> struct Printer<false> {
        template<typename... VV>
        void operator()(VV...){}
    };
    template<typename... VV> void printf1(VV... vv) {
        Printer<Parameter::on>{}(vv...);
    }
    
    template<typename... VV> void printf(VV... vv) {
        if constexpr(Parameter::on) {
            std::printf(vv...);    
        }
    }
}

int main() {
    Debug::printf1("Bla: %d \n", 42);
}
