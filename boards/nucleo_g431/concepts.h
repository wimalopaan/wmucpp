#pragma once

namespace Concept {
    template<typename T> concept Flag = requires(T) {
        {T::value} -> std::convertible_to<bool>;                                    
    };
}

