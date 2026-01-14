/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <type_traits>
#include <concepts>

#include "concepts.h"
#include "usarts.h"

namespace Mcu::Stm {
    using namespace Units::literals;
    namespace V4 {
        namespace detail {
            // this would vastly profit from C++26 static reflection
            template<typename T>
            concept hasAdapter = requires(T) {
                typename T::Adapter;
            };
            template<typename T>
            concept hasTx = requires(T) {
                typename T::Tx;
            };
            template<typename T>
            concept hasRx = requires(T) {
                typename T::Rx;
            };
            template<typename T>
            concept hasIsr = requires(T) {
                typename T::Isr;
            };
            template<typename T>
            concept hasDmaComponent = requires(T) {
                typename T::DmaChComponent;
            };

            template<typename T>
            struct getTp {
                using type = void;
            };
            template<typename T> requires(requires(T){typename T::tp;})
            struct getTp<T> {
                using type = T::tp;
            };
            template<typename T>
            using getTp_t = getTp<T>::type;


            template<typename T>
            struct getIsr {
                using type = void;
            };
            template<typename T> requires(hasIsr<T>)
            struct getIsr<T> {
                using type = T::Isr;
            };
            template<typename T>
            using getIsr_t = getIsr<T>::type;

            template<typename T>
            struct getRx {
                using type = void;
            };
            template<typename T> requires(hasRx<T>)
            struct getRx<T> {
                using type = T::Rx;
            };
            template<typename T>
            using getRx_t = getRx<T>::type;
            template<typename T>
            struct getTx {
                using type = void;
            };
            template<typename T> requires(hasTx<T>)
            struct getTx<T> {
                using type = T::Tx;
            };
            template<typename T>
            using getTx_t = getTx<T>::type;
            template<typename T>
            struct getDmaComponent {
                using type = void;
            };
            template<typename T> requires(hasDmaComponent<T>)
            struct getDmaComponent<T> {
                using type = T::DmaChComponent;
            };
            template<typename T>
            using getDmaComponent_t = getDmaComponent<T>::type;
            template<typename T>
            struct getInvert {
                static inline constexpr bool value{false};
            };
            template<typename T>
            requires(requires(T x){T::invert;})
            struct getInvert<T>{
                static inline constexpr bool value = T::invert;
            };
            template<typename T>
            static inline constexpr bool getInvert_v = getInvert<T>::value;

            template<typename T>
            struct getSwap{
                static inline constexpr bool value{false};
            };
            template<typename T>
            requires(requires(T x){T::rxtxswap;})
            struct getSwap<T>{
                static inline constexpr bool value = T::rxtxswap;
            };
            template<typename T>
            static inline constexpr bool getSwap_v = getSwap<T>::value;

            template<typename T>
            struct getParity{
                static inline constexpr Uarts::Parity value{Uarts::Parity::None};
            };
            template<typename T>
            requires(requires(T x){T::parity;})
            struct getParity<T>{
                static inline constexpr Uarts::Parity value = T::parity;
            };
            template<typename T>
            static inline constexpr auto getParity_v = getParity<T>::value;

            template<typename T>
            struct getSize {
                static inline constexpr uint16_t value{};
            };
            template<typename T>
            requires(requires(T x){T::size;})
            struct getSize<T>{
                static inline constexpr size_t value = T::size;
            };
            template<typename T>
            static inline constexpr auto getSize_v = getSize<T>::value;

            template<typename T>
            struct getEnable {
                static inline constexpr bool value{false};
            };
            template<typename T>
            requires(requires(T x){T::enable;})
            struct getEnable<T>{
                static inline constexpr bool value = T::enable;
            };
            template<typename T>
            static inline constexpr bool getEnable_v = getEnable<T>::value;

            template<typename T>
            struct getFifo {
                static inline constexpr bool value{false};
            };
            template<typename T>
            requires(requires(T x){T::fifo;})
            struct getFifo<T>{
                static inline constexpr bool value = T::fifo;
            };
            template<typename T>
            static inline constexpr bool getFifo_v = getFifo<T>::value;

            template<typename T>
            struct getTxComplete {
                static inline constexpr bool value{false};
            };
            template<typename T>
            requires(requires(T x){T::txComplete;})
            struct getTxComplete<T>{
                static inline constexpr bool value = T::txComplete;
            };
            template<typename T>
            static inline constexpr bool getTxComplete_v = getTxComplete<T>::value;

            template<typename T>
            struct getIdle {
                static inline constexpr bool value{false};
            };
            template<typename T>
            requires(requires(T x){T::idle;})
            struct getIdle<T>{
                static inline constexpr bool value = T::idle;
            };
            template<typename T>
            static inline constexpr bool getIdle_v = getIdle<T>::value;

            template<typename T>
            struct getSingleBuffer{
                static inline constexpr bool value{false};
            };
            template<typename T>
            requires(requires(T x){T::singleBuffer;})
            struct getSingleBuffer<T>{
                static inline constexpr bool value = T::singleBuffer;
            };
            template<typename T>
            static inline constexpr bool getSingleBuffer_v = getSingleBuffer<T>::value;

            template<typename T>
            struct getAdapter {
                using type = void;
            };
            template<typename T> requires(requires(T){typename T::Adapter;})
            struct getAdapter<T> {
                using type = T::Adapter;
            };
            template<typename T>
            using getAdapter_t = getAdapter<T>::type;

            template<typename T>
            struct getRxTxLinesDifferent{
                static inline constexpr bool value{false};
            };
            template<typename T>
            requires(requires(T x){T::RxTxLinesDifferent;})
            struct getRxTxLinesDifferent<T>{
                static inline constexpr bool value = T::RxTxLinesDifferent;
            };
            template<typename T>
            static inline constexpr bool getRxTxLinesDifferent_v = getRxTxLinesDifferent<T>::value;
        }
    }
}
