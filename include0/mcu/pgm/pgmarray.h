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

#include <cstddef>
#include <array>
#include <type_traits>
#include <memory>
#include <etl/meta.h>
#include <etl/ranged.h>

#if __has_include(<avr/pgmspace.h>)
# include <avr/pgmspace.h>
#endif

#include "pgm.h"

namespace AVR {
    namespace Pgm {
        //		namespace detail {
        //			template<typename T>
        //			using maybe_cref = typename std::conditional<std::is_integral<T>::value || std::is_same_v<T, std::byte> || std::is_same_v<T, char>, T, const T&>::type;
        //		}
        
        //        template<typename T, detail::maybe_cref<T>... Ts> struct Array;
        template<typename T, T... Ts> struct Array;
        
        template<typename T, typename S = uint8_t>
        struct ArrayView final {
            //            template<typename U, detail::maybe_cref<U>... Ts> friend struct Array;
            template<typename U, U... Ts> friend struct Array;
            using value_type = T;
            using size_type = S;
            
            inline constexpr T operator[](const size_type index) const {
                return value_type{Ptr{&ptrToPgmData[index]}};
            }
            inline constexpr ArrayView() = default;
            
            struct Iterator final {
                friend class ArrayView;
                inline constexpr value_type operator*() {
                    return view[mIndex];
                }
                inline constexpr void operator++() {
                    ++mIndex;
                }
                inline constexpr bool operator!=(const Iterator& rhs) {
                    return mIndex != rhs.mIndex;
                }
            private:
                explicit constexpr Iterator(const ArrayView& v, const size_type index = 0) : mIndex(index), view{v} {}
                size_type mIndex{};
                const ArrayView& view;
            };
            
            inline constexpr Iterator begin() const {
                return Iterator(*this);
            }
            inline constexpr Iterator end() const {
                return Iterator(*this, size());
            }
            
            explicit operator bool() const {
                return (mSize > 0);
            }
            
            inline constexpr S size() const {
                return mSize;
            }
            
            template<typename U, U... Ts> 
            inline explicit constexpr ArrayView(const Array<U, Ts...>& a) : ptrToPgmData(a.data) {}
        private:
            inline explicit constexpr ArrayView(const Ptr<T>& pgm, const S& size) : ptrToPgmData(pgm.value), mSize{size} {}
            const T* ptrToPgmData{nullptr};
            S mSize{};            
        };
        
        
        template<typename T, T... Ts>
        //        template<typename T, detail::maybe_cref<T>... Ts>
        struct Array final {
            inline constexpr Array() = default;
            
            using value_type = std::remove_const_t<std::remove_reference_t<T>>;
            using size_type = typename std::conditional<(sizeof...(Ts) < 256), uint8_t, uint16_t>::type;
            
            
            inline static constexpr size_type size() {
                return sizeof... (Ts);
            }
            
            using ranged_type = etl::uint_ranged<size_type, 0, size() - 1>;
            
            inline static constexpr Ptr<value_type> ptr(const size_type index) {
                return Ptr{&data[index]};
            }
            
            inline static constexpr value_type value(const size_type index) {
                if constexpr(std::is_fundamental_v<T>) {
                    if constexpr(sizeof(T) == 1) {
                        return {pgm_read_byte((uint8_t*)&data[index])};
                    }
                    else if constexpr(sizeof(T) == 2) {
                        return {pgm_read_word((uint8_t*)&data[index])};
                    }
                    else if constexpr(sizeof(T) == 4) {
                        return {pgm_read_dword((uint8_t*)&data[index])};
                    }
                    else {
                        return value_type{Ptr{&data[index]}};
                    }
                }
                else if constexpr(std::is_same_v<std::byte, T>) {
                    return std::byte{pgm_read_byte((uint8_t*)&data[index])};
                }
                else {
                    return value_type{Ptr{&data[index]}};
                }
            }
            
            struct Iterator final {
                friend class Array;
                inline constexpr value_type operator*() {
                    return value(mIndex);
                }
                inline constexpr void operator++() {
                    ++mIndex;
                }
                inline constexpr bool operator!=(const Iterator& rhs) {
                    return mIndex != rhs.mIndex;
                }
            private:
                explicit constexpr Iterator(const size_type index = 0) : mIndex(index) {}
                size_type mIndex{0};
            };
            inline constexpr Iterator begin() const {
                return Iterator();
            }
            inline constexpr Iterator end() const {
                return Iterator(size());
            }
            inline constexpr value_type operator[](const size_type index) const {
                return value(index);
            }
            inline constexpr operator ArrayView<T, size_type>() const {
                return ArrayView{Ptr<T>{data}, size()};
            }
        private:
            inline static constexpr const value_type data[] PROGMEM = {value_type{Ts}...}; 
        };
        
        //		template<typename T, auto N, const std::array<T, N>& values>
        //		class Array1 final {
        //			using U = std::remove_const_t<std::remove_reference_t<T>>;
        //			template<typename> struct Mapper;
        //			using mapper = Mapper<std::make_index_sequence<N>>;
        //		public:
        //            constexpr Array1() = default;
        //			typedef typename std::conditional<(N < 256), uint8_t, uint16_t>::type size_type;
        //			inline static constexpr size_type size = N;
        //			typedef U type;
        //			typedef U value_type;
        
        //			inline static U value(size_type index) {
        //				if constexpr(std::is_same<uint8_t, T>::value || std::is_same<std::byte, T>::value || std::is_same<char, T>::value) {
        //					return U{pgm_read_byte((uint8_t*)& mapper::data[index])};
        //				}
        //                else if constexpr(std::is_same<int, T>::value || std::is_same<unsigned int, T>::value || std::is_same<int16_t, T>::value || std::is_same<uint16_t, T>::value) {
        //                    return U{pgm_read_word((uint8_t*)& mapper::data[index])};
        //				}
        //				else {
        //					std::array<std::byte, sizeof(T)> bytes;
        //					for(uint8_t i = 0; i < sizeof(T); ++i) {
        //						bytes[i] = std::byte{pgm_read_byte((uint8_t*)(&mapper::data[index]) + i)};
        //					}
        //					return U::createFrom(bytes);
        //				}
        //			}
        
        //			class Iterator final {
        //			public:
        //				constexpr Iterator(size_type index = 0) : mIndex(index) {}
        //				inline U operator*() {
        //					return value(mIndex);
        //				}
        //				inline void operator++() {
        //					++mIndex;
        //				}
        //				inline bool operator!=(const Iterator& rhs) {
        //					return mIndex != rhs.mIndex;
        //				}
        //			private:
        //				size_type mIndex = 0;
        //			};
        //			constexpr Iterator begin() const {
        //				return Iterator();
        //			}
        //			constexpr Iterator end() const {
        //				return Iterator(size);
        //			}
        //			U operator[](size_type index) const {
        //				return value(index);
        //			}
        //		private:
        //			template<auto... II>
        //			struct Mapper<std::index_sequence<II...>> {
        //				inline static constexpr const U data[N] PROGMEM = {values[II]...}; 
        //			};
        //		};
        
        //        template<const auto& A> class Array2;
        
        //        template<typename T, uint8_t N>
        //		class Array2<std::array<T, N>> final {
        //			using U = std::remove_const_t<std::remove_reference_t<T>>;
        //			template<typename> struct Mapper;
        //			using mapper = Mapper<std::make_index_sequence<N>>;
        //		public:
        //            constexpr Array2() = default;
        //			typedef typename std::conditional<(N < 256), uint8_t, uint16_t>::type size_type;
        //			inline static constexpr size_type size = N;
        //			typedef U type;
        //			typedef U value_type;
        
        //			inline static U value(size_type index) {
        //				if constexpr(std::is_same<uint8_t, T>::value || std::is_same<std::byte, T>::value || std::is_same<char, T>::value) {
        //					return U{pgm_read_byte((uint8_t*)& mapper::data[index])};
        //				}
        //                else if constexpr(std::is_same<int, T>::value || std::is_same<unsigned int, T>::value || std::is_same<int16_t, T>::value || std::is_same<uint16_t, T>::value) {
        //                    return U{pgm_read_word((uint8_t*)& mapper::data[index])};
        //				}
        //				else {
        //					std::array<std::byte, sizeof(T)> bytes;
        //					for(uint8_t i = 0; i < sizeof(T); ++i) {
        //						bytes[i] = std::byte{pgm_read_byte((uint8_t*)(&mapper::data[index]) + i)};
        //					}
        //					return U::createFrom(bytes);
        //				}
        //			}
        
        //			class Iterator {
        //			public:
        //				constexpr Iterator(size_type index = 0) : mIndex(index) {}
        //				inline U operator*() {
        //					return value(mIndex);
        //				}
        //				inline void operator++() {
        //					++mIndex;
        //				}
        //				inline bool operator!=(const Iterator& rhs) {
        //					return mIndex != rhs.mIndex;
        //				}
        //			private:
        //				size_type mIndex = 0;
        //			};
        //			constexpr Iterator begin() const {
        //				return Iterator();
        //			}
        //			constexpr Iterator end() const {
        //				return Iterator(size);
        //			}
        //			U operator[](size_type index) const {
        //				return value(index);
        //			}
        //		private:
        //			template<auto... II>
        //			struct Mapper<std::index_sequence<II...>> {
        //				inline static constexpr const U data[N] PROGMEM = {values[II]...}; 
        //			};
        //		};
        
        namespace Util {
            
            template <class Sequence1, class Sequence2>
            struct _merge_and_renumber;
            
            template <size_t... I1, size_t... I2>
            struct _merge_and_renumber<std::index_sequence<I1...>, std::index_sequence<I2...>>
                    : std::index_sequence<I1..., (sizeof...(I1) + I2)...>
            { };
            
            template <size_t N>
            struct make_index_sequence
                    : _merge_and_renumber<typename make_index_sequence<N/2>::type,
                    typename make_index_sequence<N - N/2>::type>
            { };
            
            template<> struct make_index_sequence<0> : std::index_sequence<> { };
            template<> struct make_index_sequence<1> : std::index_sequence<0> { };
            
//            template <typename T, T M, T ... Indx>
//            constexpr std::integer_sequence<T, Indx...> make_index_sequence_(std::false_type)
//            {
//                return {};
//            }
            
//            template <typename T, T M, T ... Indx>
//            constexpr auto make_index_sequence_(std::true_type)
//            {
//                return make_index_sequence_<T, M, Indx..., sizeof...(Indx)>(
//                        std::integral_constant<bool, sizeof...(Indx) + 1 < M>());
//            }
            
//            template <size_t M>
//            constexpr auto make_index_sequence()
//            {
//                return make_index_sequence_<size_t, M>(std::integral_constant<bool, (0 < M)>());
//            }
            
            template<typename Generator>
            class Converter {
                inline static constexpr auto mData = Generator{}();    
                
                using size_type = typename decltype(mData)::size_type;
                using value_type = typename decltype(mData)::value_type;
//                using index_list = std::make_integer_sequence<size_type, mData.size()>;
                using index_list = make_index_sequence<mData.size()>::type;
                
                //				inline static constexpr bool isPrimitive = std::is_same_v<uint8_t, value_type> || 
                //                                                           std::is_same_v<std::byte, value_type> ||    
                //                                                           std::is_same_v<int, value_type> || 
                //                                                           std::is_same_v<char, value_type>;
                
                //				template<bool, typename> struct Pgm;
                template<typename> struct Pgm;
                
                template<auto... I, typename ValueType> 
                struct Pgm<std::integer_sequence<ValueType, I...>> {
                    using type = Array<value_type, mData[I]...>; 
                };
                //				template<auto... I, typename ValueType> 
                //				struct Pgm<true, std::integer_sequence<ValueType, I...>> {
                //					using type = Array<value_type, mData[I]...>; 
                //				};
                //				template<auto... I, typename ValueType> 
                //				struct Pgm<false, std::integer_sequence<ValueType, I...>> {
                //					using type = Array1<value_type, mData.size(), mData>;
                //				};
            public:
                //				using pgm_type = typename Pgm<isPrimitive, index_list>::type;
                using pgm_type = typename Pgm<index_list>::type;
            };
        }
        
        namespace detail {
            template<typename Dest, typename Src>
            struct GeneratorScale {
                constexpr auto operator()() {
                    std::array<typename Dest::value_type, Src::Upper + 1> lut;
                    for(typename Src::value_type v{0}; auto& l : lut) {
                        const auto v2 = std::clamp(v, Src::Lower, Src::Upper);
                        l = etl::scaleTo<Dest>(Src{v2});
                        ++v;
                    }
                    return lut;
                }
            };
        }
        
        template<typename Dest, auto SrcL, auto SrcU, typename SrcT>
        Dest scaleTo(const etl::uint_ranged<SrcT, SrcL, SrcU>& in) {
            using Lut = AVR::Pgm::Util::Converter<detail::GeneratorScale<Dest, etl::uint_ranged<SrcT, SrcL, SrcU>>>::pgm_type;
            return Dest{Lut::value(in)};
        }
        
    }
}
