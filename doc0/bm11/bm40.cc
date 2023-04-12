#include <cstdint>
#include <mcu/avr.h>

#if 1
namespace Pins {
    namespace Addr {
        // Arduino Nano Every
        struct Offset {
            // VPORTs
            const uint8_t vDir  = 0x00;
            const uint8_t vOut  = 0x01;
            const uint8_t vIn   = 0x02;
            const uint8_t vFlag = 0x03;
            // PORTs
            const uint8_t dir       = 0x00;
            const uint8_t dirSet    = 0x01;
            const uint8_t dirClear  = 0x02;
            const uint8_t dirToggle = 0x03;
            const uint8_t out       = 0x04;
            const uint8_t outSet    = 0x05;
            const uint8_t outClear  = 0x06;
            const uint8_t outToggle = 0x07;
            const uint8_t in        = 0x08;
            const uint8_t flag      = 0x09;   // Interrupt Flag
            const uint8_t portCtrl  = 0x0A;   // Slewrate Bit
        };
        constexpr Offset addrOffset;
        
        struct Address {
            uint16_t vport;  
            uint16_t port;
            uint8_t  pinCtrl;
            uint8_t  mask;
        };
        
        // The index is always the Arduino pin number.
        constexpr Address baseAddr[] {
            // | VPORT | PORT | PINn | BIT  |
            // | Base  | Base | CTRL | MASK | // BIT  | PIN
            {0x0008, 0x0440, 0x15, 0x20},  //  5   |   0
            {0x0008, 0x0440, 0x14, 0x10},  //  4   |   1
            {0x0000, 0x0400, 0x10, 0x01},  //  0   |   2
            {0x0014, 0x04A0, 0x15, 0x20},  //  5   |   3
            {0x0008, 0x0440, 0x16, 0x40},  //  6   |   4
            {0x0004, 0x0420, 0x12, 0x04},  //  2   |   5
            {0x0014, 0x04A0, 0x14, 0x10},  //  4   |   6
            {0x0000, 0x0400, 0x11, 0x02},  //  1   |   7
            {0x0010, 0x0480, 0x13, 0x08},  //  3   |   8
            {0x0004, 0x0420, 0x10, 0x01},  //  0   |   9
            {0x0004, 0x0420, 0x11, 0x02},  //  1   |  10
            {0x0010, 0x0480, 0x10, 0x01},  //  0   |  11
            {0x0010, 0x0480, 0x11, 0x02},  //  1   |  12
            {0x0010, 0x0480, 0x12, 0x04},  //  2   |  13
            {0x000C, 0x0460, 0x13, 0x08},  //  3   |  14
            {0x000C, 0x0460, 0x12, 0x04},  //  2   |  15
            {0x000C, 0x0460, 0x11, 0x02},  //  1   |  16
            {0x000C, 0x0460, 0x10, 0x01},  //  0   |  17
            {0x0014, 0x04A0, 0x12, 0x04},  //  2   |  18
            {0x0014, 0x04A0, 0x13, 0x08},  //  3   |  19
            {0x000C, 0x0460, 0x14, 0x10},  //  4   |  20
            {0x000C, 0x0460, 0x15, 0x20},  //  5   |  21
            // -----------------------------------------------------
            // because of pin double assignment 18<>22 and 19<>23
            // PA2 and PA3 are used for I2C
            {0x0000, 0x0400, 0x12, 0x04},  //  2   |  22, PA2   (18)
            {0x0000, 0x0400, 0x13, 0x08},  //  3   |  23, PA3   (19) 
        };
        
        constexpr uint8_t ANZAHLPINS = ( sizeof(baseAddr) / sizeof(baseAddr[0]) ) - 2;
    }
}

//enum class byte : uint8_t {};

//constexpr byte operator"" _byte(char v) {
//    return byte{v};
//}
//constexpr byte operator"" _byte(unsigned long long v) {
//    return byte(v);
//}

namespace PortmegaAVR0
{
    // Direction 
    constexpr register8_t* regVPORTdir(const uint8_t pin) {
        using namespace Pins::Addr;
        return (register8_t*) (baseAddr[pin].vport + addrOffset.vDir);
    }
    
    // Output Level
    constexpr register8_t* regVPORTout(const uint8_t pin) {
        using namespace Pins::Addr;
        return (register8_t*) (baseAddr[pin].vport + addrOffset.vOut);
    }
    
//    inline volatile byte& regVPORTin(const uint8_t pin) {
//        using namespace Pins::Addr;
//        return *reinterpret_cast<volatile byte*>(baseAddr[pin].vport + addrOffset.vIn);
//    }

    inline register8_t& regVPORTflag(const uint8_t pin) {
        using namespace Pins::Addr;
        return *reinterpret_cast<register8_t*>(baseAddr[pin].vport + addrOffset.vFlag);
    }
}
#endif

namespace MCU {
    struct A;
    struct B;
    struct C;
    struct D;
    struct E;
    struct F;
    
    // structure-mapping for MCU Port    
    template<typename L> 
    struct Port {
        using letter = L;
        volatile std::byte dir;
        volatile std::byte dirset;
        volatile std::byte dirclr;
        volatile std::byte dirtgl;
        volatile std::byte dirout;
        // usw.
    };
    // structure-mapping for MCU VPort    
    template<typename L> 
    struct VPort {
        using letter = L;
        volatile std::byte dir;
        volatile std::byte out;
        volatile std::byte in;
        volatile std::byte intflags;
    };
    
    // Meta-Function: maps MCU Port -> MCU VPort
    template<typename> struct PortToVPort;
    template<typename L> struct PortToVPort<Port<L>> {using type = VPort<L>;};
    
    // Meta-Function: maps MCU component -> address
    template<typename C> struct Address;
    template<> struct Address<VPort<A>> {static constexpr uintptr_t value = 0x0000;};
    template<> struct Address<VPort<B>> {static constexpr uintptr_t value = 0x0004;};
    template<> struct Address<VPort<C>> {static constexpr uintptr_t value = 0x0008;};
    template<> struct Address<VPort<D>> {static constexpr uintptr_t value = 0x000a;};
    template<> struct Address<VPort<E>> {static constexpr uintptr_t value = 0x000c;};
    template<> struct Address<VPort<F>> {static constexpr uintptr_t value = 0x000e;};
    
    template<> struct Address<Port<A>> {static constexpr uintptr_t value = 0x0400;};
    template<> struct Address<Port<B>> {static constexpr uintptr_t value = 0x0420;};
    template<> struct Address<Port<C>> {static constexpr uintptr_t value = 0x0440;};
    template<> struct Address<Port<D>> {static constexpr uintptr_t value = 0x0460;};
    template<> struct Address<Port<E>> {static constexpr uintptr_t value = 0x0480;};
    template<> struct Address<Port<F>> {static constexpr uintptr_t value = 0x04a0;};
    
    // get type-safe pointer to MCU component
    template<typename Component>
    inline Component& getBaseAddress() {
        return *reinterpret_cast<Component*>(Address<Component>::value);
    }
}

namespace Boards {
    using namespace MCU;
    struct NanoEvery;
    
    // Meta-Function: maps board pin number -> MCU component     
    template<uint8_t Pin, typename Board> struct PinToPort;
    template<> struct PinToPort<0, NanoEvery> {using type = Port<C>;};
    template<> struct PinToPort<1, NanoEvery> {using type = Port<C>;};
    template<> struct PinToPort<2, NanoEvery> {using type = Port<A>;};
    template<> struct PinToPort<3, NanoEvery> {using type = Port<F>;};
    template<> struct PinToPort<4, NanoEvery> {using type = Port<C>;};
    template<> struct PinToPort<5, NanoEvery> {using type = Port<B>;};
    template<> struct PinToPort<6, NanoEvery> {using type = Port<F>;};
    template<> struct PinToPort<7, NanoEvery> {using type = Port<A>;};
    template<> struct PinToPort<8, NanoEvery> {using type = Port<E>;};
    template<> struct PinToPort<9, NanoEvery> {using type = Port<B>;};
    // u.s.w.
}
namespace HAL {
    
    template<uint8_t Pin, typename Board>
    struct Button {
        using port = Boards::PinToPort<Pin, Board>::type;
        using vport = MCU::PortToVPort<port>::type;
        static constexpr auto mcu_vport = MCU::getBaseAddress<vport>;
        
        static void init() {
            mcu_vport().dir = std::byte{0x55}; 
        }
    };
}

using b = HAL::Button<1, Boards::NanoEvery>;

int main() {
    b::init();
    
    *PortmegaAVR0::regVPORTdir(1) = 0x66;
}
