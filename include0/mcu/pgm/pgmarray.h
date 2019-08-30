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

#if __has_include(<avr/pgmspace.h>)
# include <avr/pgmspace.h>
#endif

#include "pgm.h"

namespace AVR {
	namespace Pgm {
		namespace detail {
			template<typename T>
			using maybe_cref = typename std::conditional<std::is_integral<T>::value || std::is_same_v<T, std::byte> || std::is_same_v<T, char>, T, const T&>::type;
		}
		
		template<typename T, detail::maybe_cref<T>... Ts>
		struct Array final {
            constexpr Array() = default;
			
            using value_type = std::remove_const_t<std::remove_reference_t<T>>;
			using size_type = typename std::conditional<(sizeof...(Ts) < 256), uint8_t, uint16_t>::type;
            
            inline static constexpr size_type size() {
                return sizeof... (Ts);
			}
            
			inline static value_type value(size_type index) {
                return {Ptr{&data[index]}};
			}
			
			struct Iterator {
                friend class Array;
				inline value_type operator*() {
					return value(mIndex);
				}
				inline void operator++() {
					++mIndex;
				}
				inline bool operator!=(const Iterator& rhs) {
					return mIndex != rhs.mIndex;
				}
			private:
                explicit constexpr Iterator(size_type index = 0) : mIndex(index) {}
                size_type mIndex{0};
			};
			constexpr Iterator begin() const {
				return Iterator();
			}
			constexpr Iterator end() const {
				return Iterator(size());
			}
			inline value_type operator[](size_type index) const {
				return value(index);
			}
		private:
			inline static constexpr value_type data[] PROGMEM = {value_type{Ts}...}; 
		};
		
		template<typename T, auto N, const std::array<T, N>& values>
		class Array1 final {
			using U = std::remove_const_t<std::remove_reference_t<T>>;
			Array1() = delete;
			template<typename> struct Mapper;
			using mapper = Mapper<std::make_index_sequence<N>>;
		public:
			typedef typename std::conditional<(N < 256), uint8_t, uint16_t>::type size_type;
			inline static constexpr size_type size = N;
			typedef U type;
			typedef U value_type;
			
			inline static U value(size_type index) {
				if constexpr(std::is_same<uint8_t, T>::value || std::is_same<std::byte, T>::value || std::is_same<char, T>::value) {
					return U{pgm_read_byte((uint8_t*)& mapper::data[index])};
				}
				else {
					std::array<std::byte, sizeof(T)> bytes;
					for(uint8_t i = 0; i < sizeof(T); ++i) {
						bytes[i] = std::byte{pgm_read_byte((uint8_t*)(&mapper::data[index]) + i)};
					}
					return U::createFrom(bytes);
				}
			}
			
			class Iterator {
			public:
				constexpr Iterator(size_type index = 0) : mIndex(index) {}
				inline U operator*() {
					return value(mIndex);
				}
				inline void operator++() {
					++mIndex;
				}
				inline bool operator!=(const Iterator& rhs) {
					return mIndex != rhs.mIndex;
				}
			private:
				size_type mIndex = 0;
			};
			constexpr Iterator begin() const {
				return Iterator();
			}
			constexpr Iterator end() const {
				return Iterator(size);
			}
			U operator[](size_type index) const {
				return value(index);
			}
		private:
			template<auto... II>
			struct Mapper<std::index_sequence<II...>> {
				inline static constexpr const U data[N] PROGMEM = {values[II]...}; 
			};
		};
		
		namespace Util {
			template<typename Generator>
			class Converter {
				inline static constexpr auto mData = Generator{}();    
				
				typedef typename decltype(mData)::size_type size_type;
				typedef typename decltype(mData)::value_type value_type;
				using index_list = std::make_integer_sequence<size_type, mData.size>;
				
				inline static constexpr bool isPrimitive =  std::is_same_v<uint8_t, value_type> || std::is_same_v<std::byte, value_type>|| std::is_same_v<char, value_type>;
				
				template<bool, typename> struct Pgm;
				template<auto... I, typename ValueType> 
				struct Pgm<true, std::integer_sequence<ValueType, I...>> {
					typedef Array<value_type, mData[I] ...> type; 
				};
				template<auto... I, typename ValueType> 
				struct Pgm<false, std::integer_sequence<ValueType, I...>> {
					typedef Array1<value_type, mData.size, mData> type;
				};
			public:
				using pgm_type = typename Pgm<isPrimitive, index_list>::type;
			};
		}
	}
}
