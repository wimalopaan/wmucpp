/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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

#include <stdint.h>
#include "concepts.h"

namespace sort {
//    template<Util::Array C>
//    void bubbleSort(C& a) __attribute__((noinline));
    //[bubblesort
    template<Util::Array C>
    void bubbleSort(C& a) {
        typedef typename C::size_type size_type;
        using std::swap;
        for(size_type i = 1; i < C::size;) {
            if ((a[i - 1] > a[i])) {
                swap(a[i - 1], a[i]);
                i = 0;
            }
            else {
                ++i;
            }
        }
    }
    //]
    namespace detail::qsort {
        namespace sedgewick {
            //[sedgewick
            template<Util::Array C>
            void quickSort(C& a, typename C::signed_size_type l, typename C::signed_size_type r) {
                using std::swap;
                typename C::signed_size_type i = l-1, j = r, p = l-1, q = r; 
                if (r <= l) return;
                auto v = a[r];
                while(true) { 
                    while (a[++i] < v) ;
                    while (v < a[--j]) if (j == l) break;
                    if (i >= j) break;
                    swap(a[i], a[j]);
                    if (a[i] == v) { ++p; swap(a[p], a[i]); }
                    if (v == a[j]) { --q; swap(a[j], a[q]); }
                } 
                swap(a[i], a[r]); j = i-1; i = i+1;
                for (auto k = l; k < p; k++, j--) swap(a[k], a[j]);
                for (auto k = r-1; k > q; k--, i++) swap(a[i], a[k]);
                quickSort(a, l, j);
                quickSort(a, i, r);
            }
            //]
        }
        namespace bentley {
            //[bentley -vecswap
            //[vecswap
            template<typename T, typename SizeType>
            void vecswap(T* a, T* b, SizeType n) {
                for(SizeType i = 0; i < n; ++i) {
                    using std::swap;
                    swap(*a++, *b++);
                }
            }
            //]
            //[med
            template<typename T>
            T* med3(T* a, T* b, T* c) {
                return (*a < *b) ?
                            ((*b < *c) ? b : ((*a < *c) ? c : a ))
                          :((*b > *c) ? b : ((*a < *c) ? a : c ));
            }
            //]
            template<typename T, typename SizeType>
            void qsort(T* a, SizeType n) {
                using std::swap;
                
                T *pa, *pb, *pc, *pd, *pl, *pm, *pn;
                SizeType d, r, swap_cnt;
loop:
                swap_cnt = 0;
                if (n < 7) {
                    for (pm = a + 1; pm < (a + n); ++pm)
                        for (pl = pm; pl > a && (*(pl - 1) > *pl);
                             --pl)
                            swap(*pl, *(pl - 1));
                    return;
                }
                pm = a + (n / 2);
                if (n > 7) {
                    pl = a;
                    pn = a + (n - 1);
                    if (n > 40) {
                        d = (n / 8);
                        pl = med3(pl, pl + d, pl + 2 * d);
                        pm = med3(pm - d, pm, pm + d);
                        pn = med3(pn - 2 * d, pn - d, pn);
                    }
                    pm = med3(pl, pm, pn);
                }
                swap(*a, *pm);
                pa = pb = a + 1;
                
                pc = pd = a + (n - 1);
                for (;;) {
                    while ((pb <= pc) && (*pb <= *a)) {
                        if (*pb == *a) {
                            swap_cnt = 1;
                            swap(*pa, *pb);
                            pa += 1;
                        }
                        pb += 1;
                    }
                    while (pb <= pc && (*pc >= *a)) {
                        if (*pc == *a) {
                            swap_cnt = 1;
                            swap(*pc, *pd);
                            pd -= 1;
                        }
                        pc -= 1;
                    }
                    if (pb > pc)
                        break;
                    swap(*pb, *pc);
                    swap_cnt = 1;
                    pb += 1;
                    pc -= 1;
                }
                if (swap_cnt == 0) {  /* Switch to insertion sort */
                    for (pm = a + 1; pm < a + n; ++pm)
                        for (pl = pm; pl > a && (*(pl - 1) > *pl);
                             --pl)
                            swap(*pl, *(pl - 1));
                    return;
                }
                
                pn = a + n;
                r = std::min(pa - a, pb - pa);
                vecswap(a, pb - r, r);
                r = std::min(pd - pc, pn - pd - 1);
                vecswap(pb, pn - r, r);
                if ((r = pb - pa) > 1)
                    qsort(a, r);
                if ((r = pd - pc) > 1) {
                    a = pn - r;
                    n = r;
                    goto loop;
                }
            }
            //]
        }
        namespace naiv {
            //[naiv
            template<Util::Array C>
            auto pivot(C& v, typename C::size_type start,  typename C::size_type stop, typename C::size_type position) -> typename C::size_type {
                using std::swap;
                swap(v[start], v[position]);
                
                typename C::size_type low = start + 1;
                typename C::size_type high = stop;
                while (low < high)
                    if (v[low] < v[start])
                        ++low;
                    else if (v[--high] < v[start])
                        swap (v[low], v[high]);
                
                swap (v[start], v[--low]);
                return low;
            }
            template<Util::Array C>
            void quickSort(C& v, typename C::size_type low, typename C::size_type high) {
                if (low >= high)
                    return;
                
                typename C::size_type pivotIndex = (low + high) / 2;
                
                pivotIndex = pivot (v, low, high, pivotIndex);
                
                if (low < pivotIndex)
                    quickSort(v, low, pivotIndex);
                if (pivotIndex < high)
                    quickSort(v, pivotIndex + 1, high);
            }
            //]
        }
    }
    //[traits
    namespace Algorithm {
        struct QuickNaiv {};
        struct QuickBentleyMcIlroy {};
        struct QuickSedgeWick {};
    }
    //]
    //[wrapper
    template <typename Algo = Algorithm::QuickBentleyMcIlroy, Util::Array A> 
    void quickSort(A& v){
        if constexpr(v.size > 1) {
            if constexpr(std::is_same<Algo, Algorithm::QuickBentleyMcIlroy>::value) {
                detail::qsort::bentley::qsort(&v[0], v.size);
            }
            else if constexpr(std::is_same<Algo, Algorithm::QuickSedgeWick>::value) {
                detail::qsort::sedgewick::quickSort(v, 0, v.size - 1);
            }
            else if constexpr(std::is_same<Algo, Algorithm::QuickNaiv>::value) {
                detail::qsort::naiv::quickSort(v, 0, v.size - 1);
            }
            else {
                static_assert(Algo::isValid);
            }
        }
    }
    //]
}