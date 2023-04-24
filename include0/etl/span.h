#pragma once

#include <cstdint>
#include <array>
#include <initializer_list>
#include "algorithm.h"

namespace etl {
    
    template<auto Offset, auto Length, typename ValueType, auto N>
    struct subspan {
        explicit subspan(std::array<ValueType, N>& data) : mData{data}{}
        constexpr auto begin() {
            return mData.begin() + Offset;
        }
        constexpr auto end() {
            return mData.begin() + Offset + Length;
        }
    private:
        std::array<ValueType, N>& mData;
    };

    template<auto Offset, auto Length, typename ValueType, auto N>
    constexpr auto make_subspan(std::array<ValueType, N>& data) {
        return subspan<Offset, Length, ValueType, N>{data};
    }
        
    
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
//            static_assert(std::size(data) <= Length);
            static_assert(data.size() <= Length);
            etl::copy(*this, data);
//            constexpr auto offset = std::size(data);
            constexpr auto offset = data.size();
            (void)offset;
//            etl::fillOffset<offset>(*this, fill);
//            etl::fillOffset<std::size(data)>(*this, fill); // bug in gcc 9.0.1
            etl::fillOffset<data.size()>(*this, fill); // bug in gcc 9.0.1
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
    
    template<uint8_t Offset, uint16_t Length, typename C>
    inline auto make_span(C& c) {
        static_assert((Offset + Length) <= C::size());
        
        using value_type = etl::propagate_cv_value_type_t<C>;
        
        return span<Length, value_type>(&c[Offset]);
    }

    template<uint8_t Length, typename C>
    inline auto make_span(uint8_t offset, C& c) {
        assert((offset + Length) <= C::size());
        
        using value_type = etl::propagate_cv_value_type_t<C>;
        
        return span<Length, value_type>(&c[offset]);
    }
}
