/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <type_traits>
#include <utility>

#include "etl/scoped.h"
#include "etl/concepts.h"
#include "etl/meta.h"

namespace AVR {
    namespace detail {
        template<typename SubBits> struct register_bit_mask;
        template<typename SubBits> struct register_bit_position;
    }
    template<typename SubBits>
    using register_bit_mask_t = detail::register_bit_mask<SubBits>;
    template<typename SubBits>
    inline static constexpr auto register_bit_mask_v = detail::register_bit_mask<SubBits>::value;

    template<typename SubBits>
    using register_bit_position_t = detail::register_bit_position<SubBits>;
    template<typename SubBits>
    inline static constexpr auto register_bit_position_v = detail::register_bit_position<SubBits>::value;
    
    template<typename Component, typename BitType, typename Mode = ReadWrite, typename ValueType = std::byte, typename MCU = DefaultMcuType>
    struct FlagRegister;
    
    // FlagRegister: writing one to a bit clears(!) the bit (s.a. reset())
    template<typename Component, typename BitType, typename ValueType, typename MCU>
    struct FlagRegister<Component, BitType, ReadWrite, ValueType, MCU> final {
        typedef Component component_type;
        typedef ValueType value_type;    
        typedef BitType bit_type;
        
        FlagRegister() = delete;
        FlagRegister(const FlagRegister&) = delete;
        FlagRegister(FlagRegister&&) = delete;
        FlagRegister& operator=(const FlagRegister&) = delete;
        FlagRegister& operator=(FlagRegister&&) = delete;

        template<BitType F>
        void inline testAndReset(const auto& c) {
            if (isSet<F>()) {
                reset<F>();
                c();
            }
        }
        
        template<BitType F>
        bool inline isSet() const {
            return std::any(hwRegister & static_cast<value_type>(F));
        }
        template<BitType F>
        void inline reset() {
            hwRegister = static_cast<value_type>(F); // clears the bit by writing "1"
        }
        value_type inline raw() {
            return hwRegister;
        }
    private:
        volatile value_type hwRegister;
    };

    template<typename Component, typename BitType, typename ValueType, AVR::Concepts::At01DxSeries MCU>
    struct FlagRegister<Component, BitType, WriteOnly, ValueType, MCU> final {
        typedef Component component_type;
        typedef ValueType value_type;    
        typedef BitType bit_type;
        
        FlagRegister() = delete;
        FlagRegister(const FlagRegister&) = delete;
        FlagRegister(FlagRegister&&) = delete;
        FlagRegister& operator=(const FlagRegister&) = delete;
        FlagRegister& operator=(FlagRegister&&) = delete;
        
        template<BitType F>
        void inline reset() {
            hwRegister = static_cast<value_type>(F); // clears the bit by writing "1"
        }
    private:
        volatile value_type hwRegister;
    };
    
    template<typename Component, typename BitType, typename ValueType, typename MCU>
    struct FlagRegister<Component, BitType, ReadOnly, ValueType, MCU> final {
        typedef Component component_type;
        typedef ValueType value_type;    
        typedef BitType bit_type;
        
        FlagRegister() = delete;
        FlagRegister(const FlagRegister&) = delete;
        FlagRegister(FlagRegister&&) = delete;
        FlagRegister& operator=(const FlagRegister&) = delete;
        FlagRegister& operator=(FlagRegister&&) = delete;
        
        template<BitType F>
        bool inline isSet() const {
            return std::any(hwRegister & static_cast<value_type>(F));
        }
        template<BitType F>
        void inline waitFor() const {
            while(std::none(hwRegister & static_cast<value_type>(F)));
        }
        template<BitType F>
        void inline waitForCleared() const {
            while(std::any(hwRegister & static_cast<value_type>(F)));
        }
    private:
        volatile value_type hwRegister;
    };
    
    template<auto Bit, typename R>
    inline void waitFor(const R& r) {
        r.template waitFor<Bit>();
    }
    template<auto Bit, typename R>
    inline void set(R& r) {
        r.template set<Bit>();
    }
    template<auto Bit, typename R>
    inline void add(R& r) {
        r.template add<Bit>();
    }
    template<auto Bit, typename R>
    inline void clear(R& r) {
        r.template clear<Bit>();
    }
    
    template<typename Component, typename BitType, typename ValueType = uint8_t, typename MCU = DefaultMcuType>
    struct ControlRegister;

    template<typename Component, typename BitType, typename ValueType, AVR::Concepts::AtMega MCU>
    struct ControlRegister<Component, BitType, ValueType, MCU> final {
        typedef Component component_type;
        typedef ValueType value_type;    
        typedef BitType bit_type;
        
        ControlRegister() = delete;
        ControlRegister(const ControlRegister&) = delete;
        ControlRegister(ControlRegister&&) = delete;
        ControlRegister& operator=(const ControlRegister&) = delete;
        ControlRegister& operator=(ControlRegister&&) = delete;
        
        void inline set(BitType v) {
            hwRegister = static_cast<value_type>(v);
        }
        template<BitType F>
        void inline set() {
            hwRegister = static_cast<value_type>(F);
        }
        template<BitType F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        void inline setPartial(BitType v) {
            [[maybe_unused]] etl::Scoped<DI> di;
            hwRegister = (hwRegister & static_cast<value_type>(~F)) | (static_cast<value_type>(F) & static_cast<value_type>(v));
        }
        template<BitType F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        void inline add() {
            [[maybe_unused]] etl::Scoped<DI> di;
//            hwRegister |= static_cast<value_type>(F);
            hwRegister = hwRegister | static_cast<value_type>(F);
        }
        template<BitType F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        void inline clear() {
            [[maybe_unused]] etl::Scoped<DI> di;
//            hwRegister &= ~static_cast<value_type>(F);
            hwRegister = hwRegister & ~static_cast<value_type>(F);
        }
        template<BitType Mask>
        inline BitType get() const {
            return static_cast<BitType>(hwRegister & static_cast<value_type>(Mask));
        }
        template<uint8_t Mask>
        inline BitType get() const {
            return static_cast<BitType>(hwRegister & Mask);
        }
        template<BitType F>
        bool inline isSet() const {
            return hwRegister & static_cast<value_type>(F);
        }
        value_type inline raw() {
            return hwRegister;
        }
        BitType inline value() const {
            return static_cast<BitType>(hwRegister);
        }
        
    private:
        volatile value_type hwRegister;
    };

    template<typename Component, typename BitType, typename ValueType, AVR::Concepts::At01DxSeries MCU>
    struct ControlRegister<Component, BitType, ValueType, MCU> final {
        typedef Component component_type;
        typedef ValueType value_type;    
        typedef BitType bit_type;
        
        ControlRegister() = delete;
        ControlRegister(const ControlRegister&) = delete;
        ControlRegister(ControlRegister&&) = delete;
        ControlRegister& operator=(const ControlRegister&) = delete;
        ControlRegister& operator=(ControlRegister&&) = delete;
        
        void inline set(BitType v) {
            hwRegister = static_cast<value_type>(v);
        }
        template<BitType F>
        void inline set() {
            hwRegister = static_cast<value_type>(F);
        }
        template<BitType F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        void inline setPartial(BitType v) {
            [[maybe_unused]] etl::Scoped<DI> di;
            hwRegister = (hwRegister & static_cast<value_type>(~F)) | (static_cast<value_type>(F) & static_cast<value_type>(v));
        }
        template<BitType F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        void inline add() {
            [[maybe_unused]] etl::Scoped<DI> di;
//            hwRegister |= static_cast<value_type>(F);
            hwRegister = hwRegister | static_cast<value_type>(F);
        }
        template<BitType F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        void inline clear() {
            [[maybe_unused]] etl::Scoped<DI> di;
//            hwRegister &= ~static_cast<value_type>(F);
            hwRegister = hwRegister & ~static_cast<value_type>(F);
        }
        template<BitType Mask>
        inline BitType get() const {
            return static_cast<BitType>(hwRegister & static_cast<value_type>(Mask));
        }
        template<uint8_t Mask>
        inline BitType get() const {
            return static_cast<BitType>(hwRegister & Mask);
        }
        template<BitType F>
        bool inline isSet() const {
            return hwRegister & static_cast<value_type>(F);
        }
        value_type inline raw() const {
            return hwRegister;
        }
        BitType inline value() const {
            return static_cast<BitType>(hwRegister);
        }
    private:
        volatile value_type hwRegister;
    };


    template<typename Component, typename... BitTypes, typename ValueType, AVR::Concepts::At01DxSeries MCU>
    struct ControlRegister<Component, Meta::List<BitTypes...>, ValueType, MCU> final {
        using component_type = Component;
        using value_type = ValueType;    
        
        using type_list = Meta::List<BitTypes...>;        
        
        ControlRegister() = delete;
        ControlRegister(const ControlRegister&) = delete;
        ControlRegister(ControlRegister&&) = delete;
        ControlRegister& operator=(const ControlRegister&) = delete;
        ControlRegister& operator=(ControlRegister&&) = delete;
        
        template<typename F>
        requires (Meta::contains<type_list, F>::value)
        void inline set(F f) {
            hwRegister = static_cast<value_type>(f);
        }
        
        template<auto F>
        requires (Meta::contains<type_list, decltype(F)>::value)
        void inline set() {
            hwRegister = static_cast<value_type>(F);
        }
        
        template<typename F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        requires (Meta::contains<type_list, F>::value)
        void inline setPartial(F v) {
            [[maybe_unused]] etl::Scoped<DI> di;
            constexpr auto mask = AVR::register_bit_mask_v<F>;
            hwRegister = (hwRegister & static_cast<value_type>(~mask)) | (static_cast<value_type>(mask) & static_cast<value_type>(v));
        }
        template<typename F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        requires (Meta::contains<type_list, F>::value)
        void inline add(F v) {
            [[maybe_unused]] etl::Scoped<DI> di;
            hwRegister |= static_cast<value_type>(v);
        }
        template<auto F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        requires (Meta::contains<type_list, decltype(F)>::value)
        void inline add() {
            [[maybe_unused]] etl::Scoped<DI> di;
//            hwRegister |= static_cast<value_type>(F);
            hwRegister = hwRegister | static_cast<value_type>(F);
        }
        template<typename F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        requires (Meta::contains<type_list, F>::value)
        void inline clear(F v) {
            [[maybe_unused]] etl::Scoped<DI> di;
            hwRegister &= ~static_cast<value_type>(v);
        }
        template<auto F, typename DI = etl::DisbaleInterrupt<etl::RestoreState>>
        void inline clear() {
            [[maybe_unused]] etl::Scoped<DI> di;
//            hwRegister &= ~static_cast<value_type>(F);
            hwRegister = hwRegister & ~static_cast<value_type>(F);
        }
//        template<BitType Mask>
//        inline BitType get() {
//            return static_cast<BitType>(hwRegister & static_cast<value_type>(Mask));
//        }
//        template<uint8_t Mask>
//        inline BitType get() {
//            return static_cast<BitType>(hwRegister & Mask);
//        }
//        template<BitType F>
//        bool inline isSet() {
//            return hwRegister & static_cast<value_type>(F);
//        }
        auto inline raw() {
            return std::byte{hwRegister};
        }
//        BitType inline value() {
//            return static_cast<BitType>(hwRegister);
//        }
    private:
        volatile value_type hwRegister;
    };

    
    template<typename Component, typename Mode = ReadWrite, typename ValueType = uint8_t, typename MCU = DefaultMcuType>
    struct DataRegister;
    
    template<typename Component, typename ValueType, typename MCU>
    struct DataRegister<Component, UnUsed, ValueType, MCU> final {
        typedef Component component_type;
        typedef ValueType value_type;    
        DataRegister() = delete;
        DataRegister(const DataRegister&) = delete;
        DataRegister(DataRegister&&) = delete;
        DataRegister& operator=(const DataRegister&) = delete;
        DataRegister& operator=(DataRegister&&) = delete;
    private:    
        volatile value_type hwRegister; // needed to occupy space
    };
    
    template<typename Component, typename ValueType, typename MCU>
    struct DataRegister<Component, ReadOnly, ValueType, MCU> final {
        typedef Component component_type;
        typedef ValueType value_type;    
        DataRegister() = delete;
        DataRegister(const DataRegister&) = delete;
        DataRegister(DataRegister&&) = delete;
        DataRegister& operator=(const DataRegister&) = delete;
        DataRegister& operator=(DataRegister&&) = delete;
        
        inline const volatile value_type& operator*() const {
            return hwRegister;
        }
    private:    
        volatile value_type hwRegister;
    };
    
    template<typename Component, typename ValueType, typename MCU>
    struct DataRegister<Component, ReadWrite, ValueType, MCU> final {
        typedef Component component_type;
        typedef ValueType value_type;    
        DataRegister() = delete;
        DataRegister(const DataRegister&) = delete;
        DataRegister(DataRegister&&) = delete;
        DataRegister& operator=(const DataRegister&) = delete;
        DataRegister& operator=(DataRegister&&) = delete;
        
        inline volatile value_type& operator*() {
            return hwRegister;
        }
        inline const volatile value_type& operator*() const {
            return hwRegister;
        }
    private:    
        volatile value_type hwRegister;
    };
    
    template<typename Register, etl::Concepts::NamedConstant N, typename ValueType = std::byte>
    struct SbiCbiRegister final {
        static inline constexpr uint8_t reg_number = N::value;
    private:
        SbiCbiRegister() = delete;
        inline static constexpr auto reg = AVR::getBaseAddr<Register, reg_number>; // Funktionszeiger
        static_assert(AVR::isSBICBICapable<Register, reg_number>(), "not sbi-cbi capable");
        static_assert(sizeof(ValueType) == 1, "can only use byte-types");
    public:        
        using register_type = Register;
        using value_type = ValueType ;

        inline ValueType& raw() const {
            return *(reinterpret_cast<ValueType*>(reg()));
        }
    };
}
