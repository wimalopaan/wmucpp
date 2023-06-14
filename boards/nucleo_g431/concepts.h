#pragma once

#include <type_traits>
#include <concepts>

namespace Concept {
    template<typename T> concept Flag = requires(T) {
        {T::value} -> std::convertible_to<bool>;                                    
    };
}

