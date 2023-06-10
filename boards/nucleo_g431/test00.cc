#include <stm32g4xx.h>

#include <type_traits>

struct Stm32G431;

template<bool F>
using UseInterrupts = std::integral_constant<bool, F>;

namespace STM32 {
    template<typename T> concept Flag = requires(T) {
        T::value;                                    
    };
    
    template<typename MCU> struct isG4xx : std::false_type {};
    template<> struct isG4xx<Stm32G431> : std::true_type {};
        
    template<typename MCU> concept G4xx = isG4xx<MCU>::value;

    struct HSI;
    
    template<uint32_t Freq, typename ClockSource = HSI> struct ClockConfig; 
    template<>
    struct ClockConfig<170'000'000, HSI> {
        static inline constexpr uint8_t pllM{4};
        static inline constexpr uint8_t pllN{85};
        static inline constexpr uint8_t pllR{2};
        static inline constexpr uint8_t pllP{2};
        static inline constexpr uint8_t pllQ{2};
        
        static inline constexpr uint32_t f{170'000'000};
        static inline constexpr uint32_t systick{f / 1000};
    };
    
    template<typename Config, typename MCU = void> struct Clock;
    template<typename Config, STM32::G4xx MCU>
    struct Clock<Config, MCU> {
        using config = Config;
        static inline void init() {
            RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
            RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
            
            PWR->CR5 &= PWR_CR5_R1MODE;
            PWR->CR1 |= PWR_CR1_VOS_0;
            
            RCC->CR |= RCC_CR_HSION;
            while (!(RCC->CR & RCC_CR_HSIRDY));
            
            RCC->PLLCFGR |= (Config::pllP << RCC_PLLCFGR_PLLPDIV_Pos);
            RCC->PLLCFGR |= (Config::pllQ << RCC_PLLCFGR_PLLQ_Pos);
            RCC->PLLCFGR |= (Config::pllR << RCC_PLLCFGR_PLLR_Pos);
            RCC->PLLCFGR |= (Config::pllN << RCC_PLLCFGR_PLLN_Pos);
            RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos);
            
            RCC->CR |= RCC_CR_PLLON;
            while (!(RCC->CR & RCC_CR_PLLRDY));
            
            RCC->CFGR |= RCC_CFGR_SW_PLL;
            while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
        }        
    };

    template<typename Clock,  typename UseInterrupts = std::false_type, typename MCU = void> struct SystemTimer;
    template<typename Clock, Flag UseInterrupts, STM32::G4xx MCU>
    struct SystemTimer<Clock, UseInterrupts, MCU> {
        inline static void init() {
            SysTick->LOAD = Clock::config::systick;
            SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos);

            if constexpr(UseInterrupts::value) {
                SysTick->CTRL |= (1 << SysTick_CTRL_TICKINT_Pos);                
            }
        }
        template<typename F = UseInterrupts, typename = std::enable_if_t<!F::value>>
        inline static void periodic(const auto f) {
            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
                ++mValue;
                f();
            }
        }
        template<typename F = UseInterrupts, typename = std::enable_if_t<F::value>>
        inline static void 
        isr() {
            ++mValue;
        }
    private:
//        inline static Interrupt::volatile_atomic<uint32_t> mValue{0};
        inline static uint32_t mValue{0};
    public:
        inline static volatile auto& value{mValue};
    };
}

template<typename HW, typename MCU = void>
struct Devices {
    using clock = STM32::Clock<STM32::ClockConfig<170'000'000, STM32::HSI>, MCU>;
    using systemTimer = STM32::SystemTimer<clock, UseInterrupts<false>, MCU>; 
    
    static inline void init() {
        systemTimer::init();
    }
};

using devs = Devices<void, Stm32G431>;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    
    static inline void init() {
        devs::clock::init();
    }
    static inline void periodic() {
        
    }
    static inline void ratePeriodic() {
        
    }
};

using gfsm = GFSM<devs>;

//extern "C" void SysTick_Handler()  {                               
//    devs::systemTimer::isr();
//}

uint32_t r;

int main() {
    devs::init();
    
    gfsm::init();
    
//    SysTick_Config(SystemCoreClock / 1000);
    while(true) {
        gfsm::periodic();
        devs::systemTimer::periodic([]{
            gfsm::ratePeriodic();
            
            r = devs::systemTimer::value;
            
        });
    }
}
