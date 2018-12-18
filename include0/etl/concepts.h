#pragma once

namespace etl {
    namespace Concepts {
        template<typename T, typename... ArgType>
        concept bool Callable = requires(T t) {
                t(ArgType{}...);
            };
        
        template<typename T>
        concept bool NamedFlag = requires(T) {
                T::value;
                {T::value} -> bool;
            };

        template<typename T>
        concept bool NamedConstant = requires(T) {
                T::value;
                {T::value} -> typename T::value_type;
            };

        template<typename T>
        concept bool Integral = std::is_integral<T>::value;    
        
        template<typename T>
        concept bool Unsigned = std::is_unsigned<T>::value;    
        
        template<typename T>
        concept bool Signed = std::is_signed<T>::value;    
        
        template<typename R>
        concept bool Range = requires (R r) { 
                typename R::value_type;
                r.begin();
                r.end();
            };

        template<typename T>
        concept bool Fundamental = std::is_fundamental<T>::value;
        
        template<typename T>
        concept bool NonFundamental = !std::is_fundamental<T>::value;
    
    }
}

