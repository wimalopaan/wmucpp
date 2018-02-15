/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>
#include <array>

#include "container/stringbuffer.h"
#include "container/pgmstring.h"

#include "util/static_interface.h"

namespace UI {
    template<typename Buffer, typename Key = uint8_t>
    class MenuItem {
    public:
        bool isSelected() const {return mSelected;}
        virtual void putTextInto(Buffer&) const = 0;
        virtual MenuItem* processKey(Key) {return this;}
        virtual bool hasChildren() const {return false;}
        virtual void parent(MenuItem*) {}
    protected:
        bool mSelected = false;
    private:
//        virtual ~MenuItem() {};
    };
    
    template<uint8_t Length, typename C>
    class span {
    public:
        typedef C type;
        inline static constexpr uint8_t size = Length;
        explicit span(C* data) : mData{data} {}
        inline C& operator[](uint8_t index) {
            assert(index < Length);
            assert(mData);
            return mData[index];   
        }
    private:
        C* mData = nullptr;
    };
    
    template<uint8_t Offset, uint8_t Length, typename C>
    inline span<Length, typename C::type> make_span(C& c) {
        static_assert((Offset + Length) <= C::size);
        return span<Length, typename C::type>(&c[Offset]);
    }
    
    namespace Static {
        
        template<typename... Impl>
        struct MenuItem : ::Static::InterfaceBase<Impl...> {
            using base = ::Static::InterfaceBase<Impl...>;
            using implementors = typename base::implementors;
            using ptype = typename base::ptype;
        
            static bool isSelected(const ptype& n) {
                bool r = false;
                Meta::visitAt<implementors>(n.id(), [&](auto i){
                    r = n.get(i)->isSelected();
                });
                return r;
            }
            template<typename Buffer>
            static void putTextInto(const ptype& n, Buffer& b) {
                Meta::visitAt<implementors>(n.id(), [&](auto i){
                    n.get(i)->putTextInto(b);
                });
            }
            template<typename Buffer>
            static void textTo(const ptype& n, Buffer& b) {
                Meta::visitAt<implementors>(n.id(), [&](auto i){
                    n.get(i)->textTo(b);
                });
            }
            template<typename Key>
            static ptype processKey(const ptype& n, Key k) {
                ptype r;
                Meta::visitAt<implementors>(n.id(), [&](auto i){
                    r = n.get(i)->processKey(k);
                });
                return r;
            }
            static bool hasChildren(const ptype& n) {
                bool r = false;
                Meta::visitAt<implementors>(n.id(), [&](auto i){
                    r = n.get(i)->hasChildren();
                });
                return r;
            }
            static void parent(const ptype& n, ptype p) {
                Meta::visitAt<implementors>(n.id(), [&](auto i){
                    n.get(i)->parent(p);
                });
            }
        };
        
    }
}
