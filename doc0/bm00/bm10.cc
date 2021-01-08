
#include <stdint.h>
#include <utility>

namespace Peripheral {
   	using reg32_t =uint32_t;
	using reg16_t =uint16_t;

	constexpr reg32_t PERIPH_BASE          = 0x40000000U;
	constexpr reg32_t APB1PERIPH_BASE      = PERIPH_BASE;
	constexpr reg32_t APB2PERIPH_BASE      = PERIPH_BASE + 0x00010000UL;
	constexpr reg32_t AHB1PERIPH_BASE      = PERIPH_BASE + 0x00020000UL;
	constexpr reg32_t AHB2PERIPH_BASE      = PERIPH_BASE + 0x10000000UL;

    // type save set bit
    template<reg32_t ra,typename T, typename ... Args>
    constexpr void set_bit(Args ... args ) {
        static_assert((... && std::is_same_v<Args, T>));   // parameter test
        reg32_t mask =(... | args);                       // folding OR
        *reinterpret_cast<volatile reg32_t*>(ra) |= mask; // set bitmask
    };

    // type save clear bit
    template<reg32_t ra,typename T, typename ... Args>
    constexpr void clear_bit(Args ... args ) {
        static_assert((... && std::is_same_v<Args, T>));   // parameter test
        reg32_t mask =(... | args);                       // folding OR
        *reinterpret_cast<volatile reg32_t*>(ra) &= ~mask; // clear bitmask
    };
}

namespace Peripheral::RCC{

    namespace Regs {
        constexpr reg32_t RCC_BASE              = AHB1PERIPH_BASE + 0x3800UL;
        // RCC register
        constexpr reg32_t RCC_CR                = RCC_BASE;
        constexpr reg32_t RCC_PLLCFGCR          = RCC_BASE + 0x04U;
        constexpr reg32_t RCC_CFGCR             = RCC_BASE + 0x08U;
        constexpr reg32_t RCC_CIR               = RCC_BASE + 0x08U;
        constexpr reg32_t RCC_AHB1RSTR          = RCC_BASE + 0x10U;
        constexpr reg32_t RCC_AHB2RSTR          = RCC_BASE + 0x14U;
        constexpr reg32_t RCC_APB1RSTR          = RCC_BASE + 0x20U;
        constexpr reg32_t RCC_APB2RSTR          = RCC_BASE + 0x24U;
        constexpr reg32_t RCC_AHB1ENR           = RCC_BASE + 0x30U;
        constexpr reg32_t RCC_AHB2ENR           = RCC_BASE + 0x34U;
        constexpr reg32_t RCC_APB1ENR           = RCC_BASE + 0x40U;
        constexpr reg32_t RCC_APB2ENR           = RCC_BASE + 0x44U;
        constexpr reg32_t RCC_AHB1LENR          = RCC_BASE + 0x50U;
        constexpr reg32_t RCC_AHB2LENR          = RCC_BASE + 0x54U;
        constexpr reg32_t RCC_APB1LENR          = RCC_BASE + 0x60U;
        constexpr reg32_t RCC_APB2LENR          = RCC_BASE + 0x64U;
        constexpr reg32_t RCC_BDCR              = RCC_BASE + 0x78U;
        constexpr reg32_t RCC_CSR               = RCC_BASE + 0x74U;
        constexpr reg32_t RCC_SSGR              = RCC_BASE + 0x80U;
        constexpr reg32_t RCC_PLLi2SCFGR        = RCC_BASE + 0x84U;
	}

    enum AHB1_Peripheral_Clocks {
        GPIOA_CLK = 1<<0,
        GPIOB_CLK = 1<<1,
        GPIOC_CLK = 1<<2,
        GPIOD_CLK = 1<<3,
        GPIOE_CLK = 1<<4,
        GPIOH_CLK = 1<<7,
        CRC_CLK   = 1<<12,
    };

                // AHB1
        template< typename ... Args>
        constexpr void AHB1_EnableClock(Args ... args) {
            set_bit<Regs::RCC_AHB1ENR,AHB1_Peripheral_Clocks,Args ...>(std::forward<Args>(args)...);
        }

        template< typename ... Args>
        constexpr void AHB1_DisableClock(Args ... args) {
            clear_bit<Regs::RCC_AHB1ENR,AHB1_Peripheral_Clocks,Args ...>(std::forward<Args>(args)...);
        }

    // AHB2
    enum AHB2_Peripheral_Clocks {
	    OTGFS_CLK = 1<<7,
    };
    
}

using namespace Peripheral::RCC;

int main(void) {

 AHB1_EnableClock(CRC_CLK,GPIOA_CLK,GPIOB_CLK,GPIOC_CLK,GPIOD_CLK);

 //   Compiler fehler     
 //   AHB1_EnableClock(CRC_CLK,GPIOA_CLK,OTGFS_CLK); 
 //   AHB1_EnableClock(OTGFS_CLK);
 //   AHB1_EnableClock(4711);


};
