#pragma once

#include <cstdint>
#include <array>
#include <initializer_list>
#include "algorithm.h"

namespace etl {
    template<uint8_t Length, typename ValueType>
    class span {
    public:
        using value_type = ValueType;
        explicit constexpr span(ValueType* data) : mData{data} {}
        
        inline static constexpr uint8_t size()  {
            return Length;
        }
        template<typename C>
        inline constexpr void insertLeftFill(const C& data, value_type fill = value_type{' '}) {
            static_assert(std::size(data) <= Length);
            etl::copy(*this, data);
            constexpr auto offset = std::size(data);
            (void)offset;
            etl::fillOffset<offset>(*this, fill);
//            etl::fillOffset<std::size(data)>(*this, fill); // bug in gcc 9.0.1
        }
        template<typename... VV>
        inline constexpr void insertLeft(VV... vv) {
            static_assert(sizeof...(vv) <= Length);
            etl::copy(*this, vv...);
        }
        
        inline constexpr ValueType& operator[](uint8_t index) {
            assert(index < Length);
            assert(mData);
            return mData[index];   
        }
        inline constexpr const ValueType& operator[](uint8_t index) const {
            assert(index < Length);
            assert(mData);
            return mData[index];   
        }
    private:
        ValueType* const mData = nullptr;
    };
    
    template<uint8_t Offset, uint8_t Length, typename C>
    inline auto make_span(C& c) {
        static_assert((Offset + Length) <= C::size());
        
        using value_type = etl::propagate_cv_value_type_t<C>;
        
        return span<Length, value_type>(&c[Offset]);
    }
    
}
