#pragma once

#include <cstdint>
#include <cstdint>
#include <cassert>
#include "util/meta.h"
#include "util/bits.h"

namespace Static {
    template<typename... Impl>
    struct InterfaceBase {
        inline static constexpr uint8_t size = sizeof...(Impl);
        static_assert(size < (std::numeric_limits<uint8_t>::max() - 1));
        using implementors = Meta::List<Impl...>;
        
        inline static constexpr auto typeBits = Util::minimumBitsForValue(size - 1);
        inline static constexpr auto adrBits = Util::minimumBitsForValue(RAMEND);
        
        inline static constexpr bool useSmall = (typeBits <= ((8 * sizeof(uintptr_t)) - adrBits));
//        inline static constexpr bool useSmall = false;
        
        template<typename Name>
        struct TypedPtr {
            template<typename Wrapper>
            inline auto get(Wrapper&) const -> typename Wrapper::type* {
                typedef typename Wrapper::type T;
                static_assert(Meta::contains<implementors, T>::value);
                assert(ptr);
                assert(id != std::numeric_limits<uint8_t>::max());
                return reinterpret_cast<T*>(ptr);
            }
            inline explicit operator bool() const {
                return ptr != nullptr;
            }
            inline bool operator!=(const TypedPtr& o) {
                return ptr != o.ptr;
            }
            inline uint8_t id() const {
                return mId;
            }
            uint8_t mId = std::numeric_limits<uint8_t>::max();
            void* ptr = nullptr;
        };
        template<typename Name, auto Bits>
        struct TypedPtrSmall {
            static inline constexpr uint8_t idShift = (8 * sizeof(uintptr_t) - Bits);
            static inline constexpr uintptr_t pmask = (1 << (idShift)) - 1;
            static inline constexpr uintptr_t idmask = (1 << Bits) - 1;
            
            template<typename Wrapper>
            inline auto get(Wrapper&) const -> typename Wrapper::type* {
                typedef typename Wrapper::type T;
                static_assert(Meta::contains<implementors, T>::value);
                assert(ptr);
                assert(id != std::numeric_limits<uint8_t>::max());
                return reinterpret_cast<T*>(uintptr_t(ptr) & pmask);
            }
            inline explicit operator bool() const {
                return ptr != nullptr;
            }
            inline bool operator!=(const TypedPtrSmall& o) {
                return ptr != o.ptr;
            }
            inline uint8_t id() const {
                return (uintptr_t(ptr) >> idShift) & idmask;
            }
            void* ptr = nullptr;
        };
        
        typedef typename std::conditional<useSmall, TypedPtrSmall<InterfaceBase<Impl...>, typeBits>, TypedPtr<InterfaceBase<Impl...>>>::type ptype;
    
        template<typename T>
        inline static constexpr ptype make_pointer(T& o) {
            static_assert(Meta::contains<implementors, T>::value);
            if constexpr(useSmall) {
                auto id = uintptr_t(Meta::index<implementors, T>::value << ptype::idShift);
                auto p  = uintptr_t(&o) | id;
                return ptype{(T*)p};
            }
            else {
                return ptype{Meta::index<implementors, T>::value, &o};
            }
        }
    };
    
}
