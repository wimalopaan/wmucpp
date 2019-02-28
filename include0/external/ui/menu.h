#include <cstdint>
#include <array>

#include "etl/stringbuffer.h"
#include "mcu/pgm/pgmstring.h"

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
}
