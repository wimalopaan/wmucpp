	.cpu cortex-m4
	.arch armv7e-m
	.eabi_attribute 27, 1	@ Tag_ABI_HardFP_use
	.eabi_attribute 28, 1	@ Tag_ABI_VFP_args
	.eabi_attribute 20, 1	@ Tag_ABI_FP_denormal
	.eabi_attribute 21, 1	@ Tag_ABI_FP_exceptions
	.eabi_attribute 23, 3	@ Tag_ABI_FP_number_model
	.eabi_attribute 24, 1	@ Tag_ABI_align8_needed
	.eabi_attribute 25, 1	@ Tag_ABI_align8_preserved
	.eabi_attribute 26, 1	@ Tag_ABI_enum_size
	.eabi_attribute 30, 2	@ Tag_ABI_optimization_goals
	.eabi_attribute 34, 1	@ Tag_CPU_unaligned_access
	.eabi_attribute 18, 4	@ Tag_ABI_PCS_wchar_t
@ GNU C++23 (Arch Repository) version 13.1.0 (arm-none-eabi)
@	compiled by GNU C version 12.2.1 20230201, GMP version 6.2.1, MPFR version 4.2.0, MPC version 1.3.1, isl version isl-0.26-GMP
@ warning: MPFR header version 4.2.0 differs from library version 4.2.0-p9.
@ GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
@ options passed: -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -march=armv7e-m+fp -O3 -std=c++23 -fno-exceptions -fno-unwind-tables -fno-rtti -fno-threadsafe-statics -funsigned-char -funsigned-bitfields -fshort-enums -ffunction-sections -fdata-sections -fconcepts -ftemplate-depth=2048 -fstrict-aliasing
	.text
	.section	.text._ZN7DevicesIvN3Mcu3Stm9Stm32G431EE4initEv,"axG",%progbits,Devices<void, Mcu::Stm::Stm32G431>::init(),comdat
	.align	1
	.p2align 2,,3
	.weak	Devices<void, Mcu::Stm::Stm32G431>::init()
	.syntax unified
	.thumb
	.thumb_func
	.type	Devices<void, Mcu::Stm::Stm32G431>::init(), %function
Devices<void, Mcu::Stm::Stm32G431>::init():
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:37:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	ldr	r3, .L18	@ tmp176,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:42:             FLASH->ACR |= FLASH_ACR_LATENCY_5WS 
	ldr	r0, .L18+4	@ tmp182,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:37:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	ldr	r2, [r3, #96]	@ _46, MEM[(struct RCC_TypeDef *)1073876992B].APB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:47:             PWR->CR5 &= PWR_CR5_R1MODE;
	ldr	r1, .L18+8	@ tmp185,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:37:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	orr	r2, r2, #1	@ _47, _46,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/devices.h:29:     static inline void init() {
	push	{r4}	@
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:37:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	str	r2, [r3, #96]	@ _47, MEM[(struct RCC_TypeDef *)1073876992B].APB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:38:             RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	ldr	r2, [r3, #88]	@ _48, MEM[(struct RCC_TypeDef *)1073876992B].APB1ENR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:38:             RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	orr	r2, r2, #268435456	@ _49, _48,
	str	r2, [r3, #88]	@ _49, MEM[(struct RCC_TypeDef *)1073876992B].APB1ENR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:40:             RCC->CFGR |= RCC_CFGR_HPRE_DIV2;            
	ldr	r2, [r3, #8]	@ _50, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:40:             RCC->CFGR |= RCC_CFGR_HPRE_DIV2;            
	orr	r2, r2, #128	@ _51, _50,
	str	r2, [r3, #8]	@ _51, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:42:             FLASH->ACR |= FLASH_ACR_LATENCY_5WS 
	ldr	r2, [r0]	@ _52, MEM[(struct FLASH_TypeDef *)1073881088B].ACR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:42:             FLASH->ACR |= FLASH_ACR_LATENCY_5WS 
	orr	r2, r2, #1792	@ _53, _52,
	orr	r2, r2, #5	@ _53, _53,
	str	r2, [r0]	@ _53, MEM[(struct FLASH_TypeDef *)1073881088B].ACR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:47:             PWR->CR5 &= PWR_CR5_R1MODE;
	ldr	r2, [r1, #128]	@ _54, MEM[(struct PWR_TypeDef *)1073770496B].CR5
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:47:             PWR->CR5 &= PWR_CR5_R1MODE;
	and	r2, r2, #256	@ _55, _54,
	str	r2, [r1, #128]	@ _55, MEM[(struct PWR_TypeDef *)1073770496B].CR5
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:48:             PWR->CR1 |= PWR_CR1_VOS_0;
	ldr	r2, [r1]	@ _56, MEM[(struct PWR_TypeDef *)1073770496B].CR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:48:             PWR->CR1 |= PWR_CR1_VOS_0;
	orr	r2, r2, #512	@ _57, _56,
	str	r2, [r1]	@ _57, MEM[(struct PWR_TypeDef *)1073770496B].CR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:50:             RCC->CR |= RCC_CR_HSION;
	ldr	r2, [r3]	@ _58, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:50:             RCC->CR |= RCC_CR_HSION;
	orr	r2, r2, #256	@ _59, _58,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/devices.h:29:     static inline void init() {
	sub	sp, sp, #12	@,,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:50:             RCC->CR |= RCC_CR_HSION;
	str	r2, [r3]	@ _59, MEM[(struct RCC_TypeDef *)1073876992B].CR
.L2:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:51:             while (!(RCC->CR & RCC_CR_HSIRDY));
	ldr	r2, [r3]	@ _60, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:51:             while (!(RCC->CR & RCC_CR_HSIRDY));
	lsls	r2, r2, #21	@, _60,
	bpl	.L2		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:53:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos) 
	ldr	r0, [r3, #12]	@ _62, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:53:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos) 
	ldr	r2, .L18+12	@ _63,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:61:             while (!(RCC->CR & RCC_CR_PLLRDY));
	ldr	r1, .L18	@ tmp198,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:53:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos) 
	orrs	r2, r2, r0	@, _63, _63, _62
	str	r2, [r3, #12]	@ _63, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:60:             RCC->CR |= RCC_CR_PLLON;
	ldr	r2, [r3]	@ _64, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:60:             RCC->CR |= RCC_CR_PLLON;
	orr	r2, r2, #16777216	@ _65, _64,
	str	r2, [r3]	@ _65, MEM[(struct RCC_TypeDef *)1073876992B].CR
.L3:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:61:             while (!(RCC->CR & RCC_CR_PLLRDY));
	ldr	r3, [r1]	@ _66, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:61:             while (!(RCC->CR & RCC_CR_PLLRDY));
	lsls	r3, r3, #6	@, _66,
	bpl	.L3		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:63:             RCC->CFGR |= RCC_CFGR_SW_PLL;
	ldr	r3, [r1, #8]	@ _68, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:65:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	ldr	r2, .L18	@ tmp202,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:63:             RCC->CFGR |= RCC_CFGR_SW_PLL;
	orr	r3, r3, #3	@ _69, _68,
	str	r3, [r1, #8]	@ _69, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
.L4:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:65:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	ldr	r3, [r2, #8]	@ _70, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:65:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	and	r3, r3, #12	@ tmp203, _70,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:65:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	cmp	r3, #12	@ tmp203,
	bne	.L4		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:67:             MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);            
	ldr	r3, [r2, #8]	@ _72, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/arm.h:9:             TPI->ACPR = Clock::config::f / f - 1;
	ldr	r0, .L18+16	@ tmp206,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:78:             SysTick->LOAD = Clock::config::systick;
	mov	r1, #-536813568	@ tmp208,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:67:             MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);            
	bic	r3, r3, #240	@ _73, _72,
	str	r3, [r2, #8]	@ _73, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/arm.h:9:             TPI->ACPR = Clock::config::f / f - 1;
	movs	r4, #84	@ tmp207,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:78:             SysTick->LOAD = Clock::config::systick;
	ldr	r3, .L18+20	@ tmp209,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/arm.h:9:             TPI->ACPR = Clock::config::f / f - 1;
	str	r4, [r0, #16]	@ tmp207, MEM[(struct TPI_Type *)3758358528B].ACPR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:78:             SysTick->LOAD = Clock::config::systick;
	str	r3, [r1, #20]	@ tmp209, MEM[(struct SysTick_Type *)3758153744B].LOAD
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:79:             SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos) 
	ldr	r0, [r1, #16]	@ _18, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:45:             auto temp = mcuGpio->MODER;
	ldr	r3, .L18+24	@ tmp214,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:79:             SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos) 
	orr	r0, r0, #5	@ _19, _18,
	str	r0, [r1, #16]	@ _19, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:20:             RCC->AHB2ENR |= Letter::ahb2Bit;
	ldr	r1, [r2, #76]	@ _16, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:21:             mcuDac->CR |= DAC_CR_EN1;
	ldr	r0, .L18+28	@ tmp222,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:20:             RCC->AHB2ENR |= Letter::ahb2Bit;
	orr	r1, r1, #2	@ _17, _16,
	str	r1, [r2, #76]	@ _17, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:45:             auto temp = mcuGpio->MODER;
	ldr	r1, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:46:             temp &= ~(GPIO_MODER_MODE0 << moderPos);
	bic	r1, r1, #196608	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:48:                 temp |= 0x01UL << moderPos;
	orr	r1, r1, #65536	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:50:             mcuGpio->MODER = temp;
	str	r1, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:45:             auto temp = mcuGpio->MODER;
	ldr	r1, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:46:             temp &= ~(GPIO_MODER_MODE0 << moderPos);
	bic	r1, r1, #768	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:48:                 temp |= 0x01UL << moderPos;
	orr	r1, r1, #256	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:50:             mcuGpio->MODER = temp;
	str	r1, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:41:             mcuGpio->OSPEEDR |= (0x11 << moderPos);
	ldr	r1, [r3, #8]	@ _8, MEM[(struct GPIO_TypeDef *)1207960576B].OSPEEDR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:41:             mcuGpio->OSPEEDR |= (0x11 << moderPos);
	orr	r1, r1, #4352	@ _9, _8,
	str	r1, [r3, #8]	@ _9, MEM[(struct GPIO_TypeDef *)1207960576B].OSPEEDR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:19:                 RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
	ldr	r1, [r2, #76]	@ _4, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:19:                 RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
	orr	r1, r1, #65536	@ _5, _4,
	str	r1, [r2, #76]	@ _5, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:21:             mcuDac->CR |= DAC_CR_EN1;
	ldr	r1, [r0]	@ _6, MEM[(struct DAC_TypeDef *)1342179328B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:21:             mcuDac->CR |= DAC_CR_EN1;
	orr	r1, r1, #1	@ _7, _6,
	str	r1, [r0]	@ _7, MEM[(struct DAC_TypeDef *)1342179328B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:27:             RCC->CCIPR |= 0x02 << RCC_CCIPR_ADC12SEL_Pos;
	ldr	r1, [r2, #136]	@ _29, MEM[(struct RCC_TypeDef *)1073876992B].CCIPR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:27:             RCC->CCIPR |= 0x02 << RCC_CCIPR_ADC12SEL_Pos;
	orr	r1, r1, #536870912	@ _30, _29,
	str	r1, [r2, #136]	@ _30, MEM[(struct RCC_TypeDef *)1073876992B].CCIPR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:28:             RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
	ldr	r1, [r2, #76]	@ _31, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:30:             MODIFY_REG(mcuAdc->CR , ADC_CR_DEEPPWD_Msk, 0x00 << ADC_CR_DEEPPWD_Pos); 
	mov	r3, #1342177280	@ tmp228,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:28:             RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
	orr	r1, r1, #8192	@ _32, _31,
	str	r1, [r2, #76]	@ _32, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:30:             MODIFY_REG(mcuAdc->CR , ADC_CR_DEEPPWD_Msk, 0x00 << ADC_CR_DEEPPWD_Pos); 
	ldr	r2, [r3, #8]	@ _33, MEM[(struct ADC_TypeDef *)1342177280B].CR
	bic	r2, r2, #536870912	@ _34, _33,
	str	r2, [r3, #8]	@ _34, MEM[(struct ADC_TypeDef *)1342177280B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:31:             mcuAdc->CR |= ADC_CR_ADVREGEN;
	ldr	r2, [r3, #8]	@ _35, MEM[(struct ADC_TypeDef *)1342177280B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:20:             volatile uint32_t w = us * 170;
	movw	r0, #17000	@ tmp232,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:31:             mcuAdc->CR |= ADC_CR_ADVREGEN;
	orr	r2, r2, #268435456	@ _36, _35,
	str	r2, [r3, #8]	@ _36, MEM[(struct ADC_TypeDef *)1342177280B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:20:             volatile uint32_t w = us * 170;
	str	r0, [sp, #4]	@ tmp232, w
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:21:             while(w != 0) {
	ldr	r3, [sp, #4]	@ w.25_3, w
	cbz	r3, .L5	@ w.25_3,
.L6:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:22:                 w = w - 1;
	ldr	r3, [sp, #4]	@ w.23_37, w
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:22:                 w = w - 1;
	subs	r3, r3, #1	@ _38, w.23_37,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:22:                 w = w - 1;
	str	r3, [sp, #4]	@ _38, w
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:21:             while(w != 0) {
	ldr	r3, [sp, #4]	@ w.25_39, w
	cmp	r3, #0	@ w.25_39
	bne	.L6	@
.L5:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:35:             mcuAdc->ISR = ADC_ISR_ADRDY; // clear flag
	mov	r3, #1342177280	@ tmp233,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:39:             mcuAdcCommon->CCR |= (0x0b << ADC_CCR_PRESC_Pos); // prescaler 256
	ldr	r1, .L18+32	@ tmp235,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:35:             mcuAdc->ISR = ADC_ISR_ADRDY; // clear flag
	movs	r2, #1	@ tmp234,
	str	r2, [r3]	@ tmp234, MEM[(struct ADC_TypeDef *)1342177280B].ISR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:39:             mcuAdcCommon->CCR |= (0x0b << ADC_CCR_PRESC_Pos); // prescaler 256
	ldr	r2, [r1, #8]	@ _40, MEM[(struct ADC_Common_TypeDef *)1342178048B].CCR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:39:             mcuAdcCommon->CCR |= (0x0b << ADC_CCR_PRESC_Pos); // prescaler 256
	orr	r2, r2, #2883584	@ _41, _40,
	str	r2, [r1, #8]	@ _41, MEM[(struct ADC_Common_TypeDef *)1342178048B].CCR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:40:             MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_RES_Msk, (0x00 << ADC_CFGR_RES_Pos)); // 12 bit
	ldr	r2, [r3, #12]	@ _42, MEM[(struct ADC_TypeDef *)1342177280B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:44:             mcuAdc->SQR1 = (0x00 << ADC_SQR1_L_Pos) | (16 << ADC_SQR1_SQ1_Pos);
	mov	r0, #1024	@ tmp240,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:40:             MODIFY_REG(mcuAdc->CFGR , ADC_CFGR_RES_Msk, (0x00 << ADC_CFGR_RES_Pos)); // 12 bit
	bic	r2, r2, #24	@ _43, _42,
	str	r2, [r3, #12]	@ _43, MEM[(struct ADC_TypeDef *)1342177280B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:44:             mcuAdc->SQR1 = (0x00 << ADC_SQR1_L_Pos) | (16 << ADC_SQR1_SQ1_Pos);
	str	r0, [r3, #48]	@ tmp240, MEM[(struct ADC_TypeDef *)1342177280B].SQR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:46:             mcuAdc->CR |= ADC_CR_ADEN;
	ldr	r2, [r3, #8]	@ _44, MEM[(struct ADC_TypeDef *)1342177280B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:46:             mcuAdc->CR |= ADC_CR_ADEN;
	orr	r2, r2, #1	@ _45, _44,
	str	r2, [r3, #8]	@ _45, MEM[(struct ADC_TypeDef *)1342177280B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/devices.h:41:     }
	add	sp, sp, #12	@,,
	@ sp needed	@
	ldr	r4, [sp], #4	@,
	bx	lr	@
.L19:
	.align	2
.L18:
	.word	1073876992
	.word	1073881088
	.word	1073770496
	.word	289428786
	.word	-536608768
	.word	170000
	.word	1207960576
	.word	1342179328
	.word	1342178048
	.size	Devices<void, Mcu::Stm::Stm32G431>::init(), .-Devices<void, Mcu::Stm::Stm32G431>::init()
	.section	.rodata.main.str1.4,"aMS",%progbits,1
	.align	2
.LC0:
	.ascii	"systick: \000"
	.align	2
.LC1:
	.ascii	" r: \000"
	.align	2
.LC2:
	.ascii	" a: \000"
	.align	2
.LC3:
	.ascii	"State: \000"
	.align	2
.LC4:
	.ascii	" adc: \000"
	.align	2
.LC5:
	.ascii	" ac: \000"
	.align	2
.LC6:
	.ascii	"000102030405060708091011121314151617181920212223242"
	.ascii	"526272829303132333435363738394041424344454647484950"
	.ascii	"515253545556575859606162636465666768697071727374757"
	.ascii	"67778798081828384858687888990919293949596979899\000"
	.section	.text.startup.main,"ax",%progbits
	.align	1
	.p2align 2,,3
	.syntax unified
	.thumb
	.thumb_func
	.type	main, %function
main:
	@ args = 0, pretend = 0, frame = 216
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}	@
	sub	sp, sp, #220	@,,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:19:         devs::init();
	bl	Devices<void, Mcu::Stm::Stm32G431>::init()		@
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:53:             mcuGpio->BSRR |= (0x01UL << N);
	ldr	r2, .L372	@ tmp364,
	ldr	r10, .L372+40	@ tmp835,
	ldr	r3, [r2, #24]	@ _3, MEM[(struct GPIO_TypeDef *)1207960576B].BSRR
	ldr	r8, .L372+44	@ tmp833,
	ldr	r9, .L372+48	@ tmp842,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	ldr	r5, .L372+4	@ tmp855,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:53:             mcuGpio->BSRR |= (0x01UL << N);
	orr	r3, r3, #256	@ _4, _3,
	str	r3, [r2, #24]	@ _4, MEM[(struct GPIO_TypeDef *)1207960576B].BSRR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:89:                 ++mValue;
	ldr	r3, .L372+8	@ tmp930,
	ldr	r0, [r3]	@ prephitmp_871, mValue
	ldr	r3, .L372+12	@ tmp931,
	add	r6, sp, #10	@ _667,,
	ldr	r2, [r3]	@ ivtmp.345, r
	add	r7, sp, #5	@ _809,,
	add	r3, sp, #12	@ tmp847,,
.L157:
	adds	r1, r2, #1	@ ivtmp.345, ivtmp.345,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:88:             if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
	mov	ip, #-536813568	@ tmp368,
.L21:
	ldr	r4, [ip, #16]	@ _5, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:88:             if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
	lsls	r4, r4, #15	@, _5,
	mov	r2, r1	@ ivtmp.345, ivtmp.345
	add	r1, r1, #1	@ ivtmp.345, ivtmp.345,
	bpl	.L21		@,
	ldr	r1, .L372+12	@ tmp932,
	str	r2, [r1]	@ ivtmp.345, r
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:89:                 ++mValue;
	ldr	r1, .L372+8	@ tmp933,
	adds	r0, r0, #1	@ prephitmp_871, prephitmp_871,
	str	r0, [r1]	@ prephitmp_871, mValue
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:29:         switch(mState) {
	ldrb	r1, [r10]	@ zero_extendqisi2	@ mState, mState
	cmp	r1, #3	@ mState,
	bhi	.L22		@
	tbb	[pc, r1]	@ mState
.L24:
	.byte	(.L27-.L24)/2
	.byte	(.L26-.L24)/2
	.byte	(.L25-.L24)/2
	.byte	(.L23-.L24)/2
	.p2align 1
.L23:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:59:             return mcuAdc->DR;
	mov	r1, #1342177280	@ tmp392,
	ldr	r4, .L372+16	@ tmp936,
	ldr	r1, [r1, #64]	@ _24, MEM[(struct ADC_TypeDef *)1342177280B].DR
	strh	r1, [r4]	@ movhi	@ _24, aValue
.L29:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:49:             mcuAdc->CR |= ADC_CR_ADSTART;
	mov	r4, #1342177280	@ tmp829,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:46:             mState = State::StartConv;
	movs	r1, #2	@ tmp827,
	strb	r1, [r10]	@ tmp827, mState
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:49:             mcuAdc->CR |= ADC_CR_ADSTART;
	ldr	r1, [r4, #8]	@ _27, MEM[(struct ADC_TypeDef *)1342177280B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:49:             mcuAdc->CR |= ADC_CR_ADSTART;
	orr	r1, r1, #4	@ _28, _27,
	str	r1, [r4, #8]	@ _28, MEM[(struct ADC_TypeDef *)1342177280B].CR
.L22:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:64:         ++a;
	ldrh	r1, [r8]	@, a
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:24:             mcuDac->DHR12R1 = v;
	ldr	r4, .L372+20	@ tmp403,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:64:         ++a;
	adds	r1, r1, #1	@ tmp397, a,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:65:         a &= 0x0fff;
	ubfx	r1, r1, #0, #12	@ _31, tmp397,,
	strh	r1, [r8]	@ movhi	@ _31, a
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:24:             mcuDac->DHR12R1 = v;
	str	r1, [r4, #8]	@ _31, MEM[(struct DAC_TypeDef *)1342179328B].DHR12R1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:67:         if (++c == 1000) {
	ldr	r1, [r9]	@ c, c
	adds	r1, r1, #1	@ _34, c,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:67:         if (++c == 1000) {
	cmp	r1, #1000	@ _34,
	beq	.L30		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:67:         if (++c == 1000) {
	str	r1, [r9]	@ _34, c
.L31:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:59:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	ldr	r4, .L372	@ tmp823,
	ldr	r1, [r4, #20]	@ _36, MEM[(struct GPIO_TypeDef *)1207960576B].ODR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:59:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	eor	r1, r1, #16	@ tmp824, _36,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:59:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	orr	r1, r1, #1048576	@ _38, tmp824,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:59:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	str	r1, [r4, #24]	@ _38, MEM[(struct GPIO_TypeDef *)1207960576B].BSRR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:74:     }
	b	.L157		@
.L25:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:56:             return !(mcuAdc->ISR & ADC_ISR_EOC);
	mov	r1, #1342177280	@ tmp382,
	ldr	r1, [r1]	@ _20, MEM[(struct ADC_TypeDef *)1342177280B].ISR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:39:             if (!devs::adc1::busy()) {
	lsls	r4, r1, #29	@, _20,
	bpl	.L22		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:41:                 ++ac;
	ldr	r4, .L372+24	@ tmp934,
	ldrh	r1, [r4]	@, ac
	adds	r1, r1, #1	@ tmp390, ac,
	strh	r1, [r4]	@ movhi	@ tmp390, ac
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:40:                 mState = State::ReadResult;
	movs	r1, #3	@ tmp385,
	strb	r1, [r10]	@ tmp385, mState
	b	.L22		@
.L27:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:31:             mState = State::Init;
	movs	r1, #1	@ tmp375,
	strb	r1, [r10]	@ tmp375, mState
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:32:         break;
	b	.L22		@
.L26:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/adc.h:52:             return mcuAdc->ISR & ADC_ISR_ADRDY;
	mov	r1, #1342177280	@ tmp377,
	ldr	r1, [r1]	@ _18, MEM[(struct ADC_TypeDef *)1342177280B].ISR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:34:             if (devs::adc1::ready()) {
	lsls	r1, r1, #31	@, _18,
	bmi	.L29		@,
	b	.L22		@
.L30:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:68:             c = 0;
	movs	r2, #0	@ tmp408,
	str	r2, [r9]	@ tmp408, c
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	r4, .L372+28	@ p,
	movs	r0, #115	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp409,
	b	.L32		@
.L34:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_64 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r0, .L33	@ c,
.L32:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _66, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _66,
	bpl	.L34		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _69, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _69,
	bpl	.L34		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _71,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L354	@ _71,
.L35:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _71,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _71
	beq	.L35	@
.L354:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r0, [r2]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_64 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r0, #0	@ c
	bne	.L32	@
.L33:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:38:             std::to_chars(std::begin(buffer), std::end(buffer), v);
	ldr	r2, .L372+8	@ tmp937,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	strh	r0, [sp, #8]	@ movhi	@ c, MEM[(struct array *)_436]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:38:             std::to_chars(std::begin(buffer), std::end(buffer), v);
	ldr	r4, [r2]	@ __val, MEM[(const volatile long unsigned int &)&mValue]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	strd	r0, r0, [sp]	@ c, c,
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ __val
	bne	.L37	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r2, #48	@ tmp421,
	strb	r2, [sp]	@ tmp421, MEM[(char *)_436]
.L38:
	mov	r0, sp	@ ivtmp.327,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp854,
	b	.L54		@
.L51:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r0, r6	@ ivtmp.327, _667
	beq	.L50		@,
.L54:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r4, [r0], #1	@ zero_extendqisi2	@ _75, MEM[(const value_type &)_771]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r4, .L50	@ _75,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _76, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _76,
	bpl	.L51		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _117, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _117,
	bpl	.L51		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _119,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L355	@ _119,
.L52:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _119,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _119
	beq	.L52	@
.L355:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r0, r6	@ ivtmp.327, _667
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r4, [r2]	@ _75, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	bne	.L54		@,
.L50:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	r4, .L372+32	@ p,
	movs	r0, #32	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp470,
	b	.L55		@
.L57:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_57 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r0, .L56	@ c,
.L55:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _59, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _59,
	bpl	.L57		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _120, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _120,
	bpl	.L57		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _122,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L356	@ _122,
.L58:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _122,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _122
	beq	.L58	@
.L356:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r0, [r2]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_57 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r0, #0	@ c
	bne	.L55	@
.L56:
	ldr	r2, .L372+12	@ tmp941,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	strh	r0, [sp, #8]	@ movhi	@ c, MEM[(struct array *)_436]
	ldr	r4, [r2]	@ __val, r
	strd	r0, r0, [sp]	@ c, c,
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ __val
	bne	.L60	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r2, #48	@ tmp482,
	strb	r2, [sp]	@ tmp482, MEM[(char *)_436]
.L61:
	mov	r0, sp	@ ivtmp.303,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp853,
	b	.L77		@
.L74:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r0, r6	@ ivtmp.303, _667
	beq	.L73		@,
.L77:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r4, [r0], #1	@ zero_extendqisi2	@ _126, MEM[(const value_type &)_687]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r4, .L73	@ _126,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _127, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _127,
	bpl	.L74		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _168, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _168,
	bpl	.L74		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _170,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L357	@ _170,
.L75:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _170,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _170
	beq	.L75	@
.L357:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r0, r6	@ ivtmp.303, _667
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r4, [r2]	@ _126, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	bne	.L77		@,
.L73:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	r4, .L372+36	@ p,
	movs	r0, #32	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp531,
	b	.L78		@
.L373:
	.align	2
.L372:
	.word	1207960576
	.word	1374389535
	.word	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::r
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::aValue
	.word	1342179328
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::ac
	.word	.LC0+1
	.word	.LC1+1
	.word	.LC2+1
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::mState
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::a
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::c
.L80:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_50 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r0, .L79	@ c,
.L78:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _52, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _52,
	bpl	.L80		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _171, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _171,
	bpl	.L80		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _173,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L358	@ _173,
.L81:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _173,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _173
	beq	.L81	@
.L358:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r0, [r2]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_50 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r0, #0	@ c
	bne	.L78	@
.L79:
	ldrh	r4, [r8]	@ _47, a
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	str	r0, [sp]	@ c, MEM[(struct array *)_436]
	strb	r0, [sp, #4]	@ c, MEM[(struct array *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ _47
	bne	.L83	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r2, #48	@ tmp543,
	strb	r2, [sp]	@ tmp543, MEM[(char *)_436]
.L84:
	mov	r1, sp	@ ivtmp.283,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	ip, #-536870912	@ tmp852,
.L96:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r0, [r1], #1	@ zero_extendqisi2	@ _177, MEM[(const value_type &)_858]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r0, .L92	@ _177,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r2, [ip, #3712]	@ _178, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r4, r2, #31	@, _178,
	bpl	.L93		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r2, [ip, #3584]	@ _239, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r2, r2, #31	@, _239,
	bpl	.L93		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r2, [ip]	@ _629, MEM[(struct ITM_Type *)3758096384B].PORT[0].u32
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r2, .L95	@ _629,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	mov	r4, #-536870912	@ tmp599,
.L94:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r2, [r4]	@ _241, MEM[(struct ITM_Type *)3758096384B].PORT[0].u32
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r2, #0	@ _241
	beq	.L94	@
.L95:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	mov	r2, #-536870912	@ tmp597,
	strb	r0, [r2]	@ _177, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
.L93:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r1, r7	@ ivtmp.283, _809
	bne	.L96		@,
.L92:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp600,
	ldr	r1, [r2, #3712]	@ _44, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r4, r1, #31	@, _44,
	bpl	.L97		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _242, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r0, r1, #31	@, _242,
	bmi	.L348		@,
.L97:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:70:             IO::outl<trace>("State: ", (uint8_t)mState, " adc: ", aValue, " ac: ", ac);
	ldrb	r4, [r10]	@ zero_extendqisi2	@ mState.44_35, mState
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	ip, .L374+24	@ p,
	movs	r0, #83	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp610,
	b	.L100		@
.L102:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [ip], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_261 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r0, .L101	@ c,
.L100:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _263, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _263,
	bpl	.L102		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _266, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _266,
	bpl	.L102		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _268,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L359	@ _268,
.L103:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _268,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _268
	beq	.L103	@
.L359:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r0, [r2]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [ip], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_261 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r0, #0	@ c
	bne	.L100	@
.L101:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	strh	r0, [sp]	@ movhi	@ c, MEM[(struct array *)_436]
	strb	r0, [sp, #2]	@ c, MEM[(struct array *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ mState.44_35
	bne	.L105	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r2, #48	@ tmp622,
.L350:
	strb	r2, [sp]	@ tmp622, MEM[(char *)_436]
	mov	r1, sp	@ ivtmp.269,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	ip, #-536870912	@ tmp851,
.L106:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r0, [r1], #1	@ zero_extendqisi2	@ _272, MEM[(const value_type &)_846]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r0, .L116	@ _272,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r2, [ip, #3712]	@ _273, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r2, r2, #31	@, _273,
	bpl	.L113		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r2, [ip, #3584]	@ _334, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r4, r2, #31	@, _334,
	bpl	.L113		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r2, [ip]	@ _61, MEM[(struct ITM_Type *)3758096384B].PORT[0].u32
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r2, .L115	@ _61,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	mov	r4, #-536870912	@ tmp674,
.L114:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r2, [r4]	@ _336, MEM[(struct ITM_Type *)3758096384B].PORT[0].u32
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r2, #0	@ _336
	beq	.L114	@
.L115:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	mov	r2, #-536870912	@ tmp672,
	strb	r0, [r2]	@ _272, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
.L113:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	add	r2, sp, #3	@ tmp947,,
	cmp	r1, r2	@ ivtmp.269, tmp947
	bne	.L106		@,
.L116:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	r4, .L374	@ p,
	movs	r0, #32	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp675,
	b	.L112		@
.L118:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_254 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r0, .L117	@ c,
.L112:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _256, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _256,
	bpl	.L118		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _337, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _337,
	bpl	.L118		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _339,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L360	@ _339,
.L119:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _339,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _339
	beq	.L119	@
.L360:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r0, [r2]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_254 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r0, #0	@ c
	bne	.L112	@
.L117:
	ldr	r2, .L374+4	@ tmp948,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	str	r0, [sp]	@ c, MEM[(struct array *)_436]
	ldrh	r4, [r2]	@ _42, aValue
	strb	r0, [sp, #4]	@ c, MEM[(struct array *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ _42
	bne	.L121	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r2, #48	@ tmp687,
	strb	r2, [sp]	@ tmp687, MEM[(char *)_436]
.L122:
	mov	r1, sp	@ ivtmp.249,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	ip, #-536870912	@ tmp850,
.L134:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r0, [r1], #1	@ zero_extendqisi2	@ _343, MEM[(const value_type &)_824]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r0, .L130	@ _343,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r2, [ip, #3712]	@ _344, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r2, r2, #31	@, _344,
	bpl	.L131		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r2, [ip, #3584]	@ _405, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r4, r2, #31	@, _405,
	bpl	.L131		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r2, [ip]	@ _626, MEM[(struct ITM_Type *)3758096384B].PORT[0].u32
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r2, .L133	@ _626,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	mov	r4, #-536870912	@ tmp743,
.L132:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r2, [r4]	@ _407, MEM[(struct ITM_Type *)3758096384B].PORT[0].u32
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r2, #0	@ _407
	beq	.L132	@
.L133:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	mov	r2, #-536870912	@ tmp741,
	strb	r0, [r2]	@ _343, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
.L131:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r1, r7	@ ivtmp.249, _809
	bne	.L134		@,
.L130:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	r4, .L374+8	@ p,
	movs	r0, #32	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp744,
	b	.L135		@
.L137:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_247 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r0, .L136	@ c,
.L135:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _249, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _249,
	bpl	.L137		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _408, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _408,
	bpl	.L137		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _410,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L361	@ _410,
.L138:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _410,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _410
	beq	.L138	@
.L361:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r0, [r2]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_247 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r0, #0	@ c
	bne	.L135	@
.L136:
	ldr	r2, .L374+12	@ tmp950,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	str	r0, [sp]	@ c, MEM[(struct array *)_436]
	ldrh	r4, [r2]	@ _43, ac
	strb	r0, [sp, #4]	@ c, MEM[(struct array *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ _43
	bne	.L140	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r2, #48	@ tmp756,
	strb	r2, [sp]	@ tmp756, MEM[(char *)_436]
.L141:
	mov	r1, sp	@ ivtmp.228,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	ip, #-536870912	@ tmp849,
.L153:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r0, [r1], #1	@ zero_extendqisi2	@ _414, MEM[(const value_type &)_807]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r0, .L149	@ _414,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r2, [ip, #3712]	@ _415, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r2, r2, #31	@, _415,
	bpl	.L150		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r2, [ip, #3584]	@ _476, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r4, r2, #31	@, _476,
	bpl	.L150		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r2, [ip]	@ _498, MEM[(struct ITM_Type *)3758096384B].PORT[0].u32
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r2, .L152	@ _498,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	mov	r4, #-536870912	@ tmp812,
.L151:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r2, [r4]	@ _478, MEM[(struct ITM_Type *)3758096384B].PORT[0].u32
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r2, #0	@ _478
	beq	.L151	@
.L152:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	mov	r2, #-536870912	@ tmp810,
	strb	r0, [r2]	@ _414, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
.L150:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r1, r7	@ ivtmp.228, _809
	bne	.L153		@,
.L149:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp813,
	ldr	r1, [r2, #3712]	@ _39, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r0, r1, #31	@, _39,
	bpl	.L154		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _479, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _479,
	bmi	.L353		@,
.L154:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:89:                 ++mValue;
	ldr	r2, .L374+16	@ tmp952,
	ldr	r0, [r2]	@ prephitmp_871, mValue
	movs	r2, #0	@ ivtmp.345,
	b	.L31		@
.L98:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
	.thumb
	.syntax unified
.L348:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _244,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _244
	beq	.L98	@
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	mov	r2, #-536870912	@ tmp605,
	movs	r1, #10	@ tmp606,
	strb	r1, [r2]	@ tmp606, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
	b	.L97		@
.L105:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ mState.44_35,
	bls	.L107		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ mState.44_35,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L374+20	@,
	mov	r2, #201	@,
	mov	r0, r3	@, tmp847
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	bls	.L362		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	bl	memcpy		@
	mov	r3, r0	@ tmp847,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r0, r2, r5, r4	@ tmp862, tmp640, tmp855, mState.44_35
	lsrs	r2, r2, #5	@ tmp639, tmp640,
	movs	r1, #100	@ tmp643,
	mls	r1, r1, r2, r4	@ tmp644, tmp643, tmp639, mState.44_35
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:109: 	__first[0] = '0' + __val;
	uxtb	r4, r2	@ mState.44_35, tmp639
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r2, [r3, r1, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.207_785 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r2, [sp, #1]	@ unaligned	@ vect__314.208, MEM <vector(2) char> [(char *)_436]
.L107:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:109: 	__first[0] = '0' + __val;
	add	r2, r4, #48	@ tmp665, mState.44_35,
	uxtb	r2, r2	@ _331, tmp665
	b	.L350		@
.L155:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
	.thumb
	.syntax unified
.L353:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _481,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _481
	beq	.L155	@
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	mov	r2, #-536870912	@ tmp818,
	movs	r1, #10	@ tmp819,
	strb	r1, [r2]	@ tmp819, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
	b	.L154		@
.L140:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ __unsigned_val,
	bls	.L142		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ __unsigned_val,
	bls	.L363		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	cmp	r4, #1000	@ __unsigned_val,
	bcc	.L172		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movw	r2, #9999	@ tmp763,
	cmp	r4, r2	@ __unsigned_val, tmp763
	ite	ls		@
	movls	fp, #3	@, prephitmp_751,
	movhi	fp, #4	@, prephitmp_751,
.L146:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L374+20	@,
	movs	r2, #201	@,
	mov	r0, r3	@, tmp847
	bl	memcpy		@
	add	r2, fp, #-1	@ tmp770, prephitmp_751,
	mov	r3, r0	@ tmp847,
	add	r1, sp, r2	@ ivtmp.234,, tmp770
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	mov	lr, #100	@ tmp775,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	movw	ip, #9999	@ tmp797,
.L147:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r0, r2, r5, r4	@ tmp864, tmp772, tmp855, __unsigned_val
	lsrs	r2, r2, #5	@ tmp771, tmp772,
	mls	fp, lr, r2, r4	@ tmp776, tmp775, tmp771, __unsigned_val
	mov	r0, r4	@ __val, __unsigned_val
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	cmp	r0, ip	@ __val, tmp797
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r4, [r3, fp, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.217_776 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r4, [r1], #-2	@ unaligned	@ vect__467.218, MEM <vector(2) char> [(char *)vectp.220_773]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:97: 	  __val /= 100;
	mov	r4, r2	@ __unsigned_val, tmp771
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	bhi	.L147		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:102:       if (__val >= 10)
	cmp	r0, #1000	@ __val,
	bcc	.L142		@,
.L144:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r2, sp, #216	@ tmp951,,
	add	r4, r2, r4, lsl #1	@ tmp798, tmp951, __unsigned_val,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r2, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_665][_471], MEM[(char[201] *)_665][_471]
	strb	r2, [sp, #1]	@ MEM[(char[201] *)_665][_471], MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r2, [r4, #-204]	@ zero_extendqisi2	@ _473, MEM[(char[201] *)_665][__num_470]
.L148:
	strb	r2, [sp]	@ _473, MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:235:       return __res;
	b	.L141		@
.L121:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ __unsigned_val,
	bls	.L123		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ __unsigned_val,
	bls	.L364		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	cmp	r4, #1000	@ __unsigned_val,
	bcc	.L170		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movw	r2, #9999	@ tmp694,
	cmp	r4, r2	@ __unsigned_val, tmp694
	ite	ls		@
	movls	fp, #3	@, prephitmp_752,
	movhi	fp, #4	@, prephitmp_752,
.L127:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L374+20	@,
	movs	r2, #201	@,
	mov	r0, r3	@, tmp847
	bl	memcpy		@
	add	r2, fp, #-1	@ tmp701, prephitmp_752,
	mov	r3, r0	@ tmp847,
	add	r1, sp, r2	@ ivtmp.254,, tmp701
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	mov	lr, #100	@ tmp706,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	movw	ip, #9999	@ tmp728,
.L128:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r0, r2, r5, r4	@ tmp863, tmp703, tmp855, __unsigned_val
	lsrs	r2, r2, #5	@ tmp702, tmp703,
	mls	fp, lr, r2, r4	@ tmp707, tmp706, tmp702, __unsigned_val
	mov	r0, r4	@ __val, __unsigned_val
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	cmp	r0, ip	@ __val, tmp728
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r4, [r3, fp, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.212_781 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r4, [r1], #-2	@ unaligned	@ vect__396.213, MEM <vector(2) char> [(char *)vectp.215_778]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:97: 	  __val /= 100;
	mov	r4, r2	@ __unsigned_val, tmp702
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	bhi	.L128		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:102:       if (__val >= 10)
	cmp	r0, #1000	@ __val,
	bcc	.L123		@,
.L125:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r2, sp, #216	@ tmp949,,
	add	r4, r2, r4, lsl #1	@ tmp729, tmp949, __unsigned_val,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r2, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_665][_400], MEM[(char[201] *)_665][_400]
	strb	r2, [sp, #1]	@ MEM[(char[201] *)_665][_400], MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r2, [r4, #-204]	@ zero_extendqisi2	@ _402, MEM[(char[201] *)_665][__num_399]
.L129:
	strb	r2, [sp]	@ _402, MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:235:       return __res;
	b	.L122		@
.L375:
	.align	2
.L374:
	.word	.LC4+1
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::aValue
	.word	.LC5+1
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::ac
	.word	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue
	.word	.LC6
	.word	.LC3+1
.L83:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ __unsigned_val,
	bls	.L85		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ __unsigned_val,
	bls	.L365		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	cmp	r4, #1000	@ __unsigned_val,
	bcc	.L168		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movw	r2, #9999	@ tmp550,
	cmp	r4, r2	@ __unsigned_val, tmp550
	ite	ls		@
	movls	fp, #3	@, prephitmp_753,
	movhi	fp, #4	@, prephitmp_753,
.L89:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L376	@,
	movs	r2, #201	@,
	mov	r0, r3	@, tmp847
	bl	memcpy		@
	add	r2, fp, #-1	@ tmp557, prephitmp_753,
	mov	r3, r0	@ tmp847,
	add	r1, sp, r2	@ ivtmp.288,, tmp557
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	mov	lr, #100	@ tmp562,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	movw	ip, #9999	@ tmp584,
.L90:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r0, r2, r5, r4	@ tmp861, tmp559, tmp855, __unsigned_val
	lsrs	r2, r2, #5	@ tmp558, tmp559,
	mls	fp, lr, r2, r4	@ tmp563, tmp562, tmp558, __unsigned_val
	mov	r0, r4	@ __val, __unsigned_val
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	cmp	r0, ip	@ __val, tmp584
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r4, [r3, fp, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.202_790 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r4, [r1], #-2	@ unaligned	@ vect__230.203, MEM <vector(2) char> [(char *)vectp.205_787]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:97: 	  __val /= 100;
	mov	r4, r2	@ __unsigned_val, tmp558
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	bhi	.L90		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:102:       if (__val >= 10)
	cmp	r0, #1000	@ __val,
	bcc	.L85		@,
.L87:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r2, sp, #216	@ tmp945,,
	add	r4, r2, r4, lsl #1	@ tmp585, tmp945, __unsigned_val,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r2, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_665][_234], MEM[(char[201] *)_665][_234]
	strb	r2, [sp, #1]	@ MEM[(char[201] *)_665][_234], MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r2, [r4, #-204]	@ zero_extendqisi2	@ _236, MEM[(char[201] *)_665][__num_233]
.L91:
	strb	r2, [sp]	@ _236, MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:235:       return __res;
	b	.L84		@
.L37:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ __val,
	bls	.L39		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ __val,
	bls	.L366		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	cmp	r4, #1000	@ __val,
	bcc	.L158		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movw	r2, #9999	@ tmp428,
	cmp	r4, r2	@ __val, tmp428
	bls	.L367		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	ldr	r2, .L376+4	@ tmp433,
	cmp	r4, r2	@ __val, tmp433
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	it	ls
	movls	fp, #5	@ _92,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	bls	.L42		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	ldr	r2, .L376+8	@ tmp429,
	cmp	r4, r2	@ __val, tmp429
	bls	.L368		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	ldr	r2, .L376+12	@ tmp430,
	cmp	r4, r2	@ __val, tmp430
	bcc	.L160		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	ldr	r2, .L376+16	@ tmp938,
	cmp	r4, r2	@ __val, tmp938
	bls	.L161		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	ldr	r2, .L376+20	@ tmp939,
	cmp	r4, r2	@ __val, tmp939
	bls	.L162		@,
	movs	r2, #5	@ __n,
.L46:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	add	fp, r2, #5	@ _92, __n,
.L42:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L376	@,
	movs	r2, #201	@,
	mov	r0, r3	@, tmp847
	bl	memcpy		@
	sub	r2, fp, #2	@ tmp440, _92,
	mov	r3, r0	@ tmp847,
	add	r1, sp, r2	@ ivtmp.332,, tmp440
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	mov	lr, #100	@ tmp445,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	movw	ip, #9999	@ tmp454,
.L48:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r0, r2, r5, r4	@ tmp859, tmp442, tmp855, __val
	lsrs	r2, r2, #5	@ tmp441, tmp442,
	mls	fp, lr, r2, r4	@ tmp446, tmp445, tmp441, __val
	mov	r0, r4	@ __val, __val
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	cmp	r0, ip	@ __val, tmp454
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r4, [r3, fp, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.192_803 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r4, [r1], #-2	@ unaligned	@ vect__108.193, MEM <vector(2) char> [(char *)vectp.195_797]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:97: 	  __val /= 100;
	mov	r4, r2	@ __val, tmp441
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	bhi	.L48		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:102:       if (__val >= 10)
	cmp	r0, #1000	@ __val,
	bcc	.L39		@,
.L41:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r2, sp, #216	@ tmp940,,
	add	r4, r2, r4, lsl #1	@ tmp455, tmp940, __val,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r2, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_665][_112], MEM[(char[201] *)_665][_112]
	strb	r2, [sp, #1]	@ MEM[(char[201] *)_665][_112], MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r2, [r4, #-204]	@ zero_extendqisi2	@ _114, MEM[(char[201] *)_665][__num_111]
.L49:
	strb	r2, [sp]	@ _114, MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:235:       return __res;
	b	.L38		@
.L60:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ __val,
	bls	.L62		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ __val,
	bls	.L369		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	cmp	r4, #1000	@ __val,
	bcc	.L163		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movw	r2, #9999	@ tmp489,
	cmp	r4, r2	@ __val, tmp489
	bls	.L370		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	ldr	r2, .L376+4	@ tmp494,
	cmp	r4, r2	@ __val, tmp494
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	it	ls
	movls	fp, #5	@ _143,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	bls	.L65		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	ldr	r2, .L376+8	@ tmp490,
	cmp	r4, r2	@ __val, tmp490
	bls	.L371		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	ldr	r2, .L376+12	@ tmp491,
	cmp	r4, r2	@ __val, tmp491
	bcc	.L165		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	ldr	r2, .L376+16	@ tmp942,
	cmp	r4, r2	@ __val, tmp942
	bls	.L166		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	ldr	r2, .L376+20	@ tmp943,
	cmp	r4, r2	@ __val, tmp943
	bls	.L167		@,
	movs	r2, #5	@ __n,
.L69:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	add	fp, r2, #5	@ _143, __n,
.L65:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L376	@,
	movs	r2, #201	@,
	mov	r0, r3	@, tmp847
	bl	memcpy		@
	sub	r2, fp, #2	@ tmp501, _143,
	mov	r3, r0	@ tmp847,
	add	r1, sp, r2	@ ivtmp.308,, tmp501
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	mov	lr, #100	@ tmp506,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	movw	ip, #9999	@ tmp515,
.L71:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r0, r2, r5, r4	@ tmp860, tmp503, tmp855, __val
	lsrs	r2, r2, #5	@ tmp502, tmp503,
	mls	fp, lr, r2, r4	@ tmp507, tmp506, tmp502, __val
	mov	r0, r4	@ __val, __val
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	cmp	r0, ip	@ __val, tmp515
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r4, [r3, fp, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.197_795 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r4, [r1], #-2	@ unaligned	@ vect__159.198, MEM <vector(2) char> [(char *)vectp.200_792]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:97: 	  __val /= 100;
	mov	r4, r2	@ __val, tmp502
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	bhi	.L71		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:102:       if (__val >= 10)
	cmp	r0, #1000	@ __val,
	bcc	.L62		@,
.L64:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r2, sp, #216	@ tmp944,,
	add	r4, r2, r4, lsl #1	@ tmp516, tmp944, __val,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r2, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_665][_163], MEM[(char[201] *)_665][_163]
	strb	r2, [sp, #1]	@ MEM[(char[201] *)_665][_163], MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r2, [r4, #-204]	@ zero_extendqisi2	@ _165, MEM[(char[201] *)_665][__num_162]
.L72:
	strb	r2, [sp]	@ _165, MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:235:       return __res;
	b	.L61		@
.L62:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:109: 	__first[0] = '0' + __val;
	add	r2, r4, #48	@ tmp521, __val,
	uxtb	r2, r2	@ _165, tmp521
	b	.L72		@
.L39:
	add	r2, r4, #48	@ tmp460, __val,
	uxtb	r2, r2	@ _114, tmp460
	b	.L49		@
.L85:
	add	r2, r4, #48	@ tmp590, __unsigned_val,
	uxtb	r2, r2	@ _236, tmp590
	b	.L91		@
.L362:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	bl	memcpy		@
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r2, sp, #216	@ tmp946,,
	add	r4, r2, r4, lsl #1	@ tmp629, tmp946, mState.44_35,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	mov	r3, r0	@ tmp847,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r2, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_665][_329], MEM[(char[201] *)_665][_329]
	strb	r2, [sp, #1]	@ MEM[(char[201] *)_665][_329], MEM[(char *)_436]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r2, [r4, #-204]	@ zero_extendqisi2	@ _331, MEM[(char[201] *)_665][__num_328]
	b	.L350		@
.L123:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:109: 	__first[0] = '0' + __val;
	add	r2, r4, #48	@ tmp734, __unsigned_val,
	uxtb	r2, r2	@ _402, tmp734
	b	.L129		@
.L142:
	add	r2, r4, #48	@ tmp803, __unsigned_val,
	uxtb	r2, r2	@ _473, tmp803
	b	.L148		@
.L165:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	mov	fp, #7	@ _143,
	b	.L65		@
.L160:
	mov	fp, #7	@ _92,
	b	.L42		@
.L166:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	mov	fp, #8	@ _143,
	b	.L65		@
.L161:
	mov	fp, #8	@ _92,
	b	.L42		@
.L167:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	mov	fp, #9	@ _143,
	b	.L65		@
.L162:
	mov	fp, #9	@ _92,
	b	.L42		@
.L363:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	mov	r0, r3	@, tmp847
	ldr	r1, .L376	@,
	movs	r2, #201	@,
	bl	memcpy		@
	mov	r3, r0	@ tmp847,
	b	.L144		@
.L370:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	mov	fp, #4	@ _143,
	b	.L65		@
.L163:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	mov	fp, #3	@ _143,
	b	.L65		@
.L369:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	mov	r0, r3	@, tmp847
	ldr	r1, .L376	@,
	movs	r2, #201	@,
	bl	memcpy		@
	mov	r3, r0	@ tmp847,
	b	.L64		@
.L367:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	mov	fp, #4	@ _92,
	b	.L42		@
.L158:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	mov	fp, #3	@ _92,
	b	.L42		@
.L366:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	mov	r0, r3	@, tmp847
	ldr	r1, .L376	@,
	movs	r2, #201	@,
	bl	memcpy		@
	mov	r3, r0	@ tmp847,
	b	.L41		@
.L168:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	mov	fp, #2	@ prephitmp_753,
	b	.L89		@
.L365:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	mov	r0, r3	@, tmp847
	ldr	r1, .L376	@,
	movs	r2, #201	@,
	bl	memcpy		@
	mov	r3, r0	@ tmp847,
	b	.L87		@
.L170:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	mov	fp, #2	@ prephitmp_752,
	b	.L127		@
.L364:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	mov	r0, r3	@, tmp847
	ldr	r1, .L376	@,
	movs	r2, #201	@,
	bl	memcpy		@
	mov	r3, r0	@ tmp847,
	b	.L125		@
.L172:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	mov	fp, #2	@ prephitmp_751,
	b	.L146		@
.L371:
	movs	r2, #1	@ __n,
	b	.L69		@
.L368:
	movs	r2, #1	@ __n,
	b	.L46		@
.L377:
	.align	2
.L376:
	.word	.LC6
	.word	99999
	.word	999999
	.word	10000000
	.word	99999999
	.word	999999999
	.size	main, .-main
	.weak	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::ac
	.section	.bss._ZN4GFSMI7DevicesIvN3Mcu3Stm9Stm32G431EEE2acE,"awG",%nobits,GFSM<Devices<void, Mcu::Stm::Stm32G431> >::ac,comdat
	.align	1
	.type	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::ac, %object
	.size	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::ac, 2
GFSM<Devices<void, Mcu::Stm::Stm32G431> >::ac:
	.space	2
	.weak	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::aValue
	.section	.bss._ZN4GFSMI7DevicesIvN3Mcu3Stm9Stm32G431EEE6aValueE,"awG",%nobits,GFSM<Devices<void, Mcu::Stm::Stm32G431> >::aValue,comdat
	.align	1
	.type	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::aValue, %object
	.size	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::aValue, 2
GFSM<Devices<void, Mcu::Stm::Stm32G431> >::aValue:
	.space	2
	.weak	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::mState
	.section	.bss._ZN4GFSMI7DevicesIvN3Mcu3Stm9Stm32G431EEE6mStateE,"awG",%nobits,GFSM<Devices<void, Mcu::Stm::Stm32G431> >::mState,comdat
	.type	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::mState, %object
	.size	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::mState, 1
GFSM<Devices<void, Mcu::Stm::Stm32G431> >::mState:
	.space	1
	.weak	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::a
	.section	.bss._ZN4GFSMI7DevicesIvN3Mcu3Stm9Stm32G431EEE1aE,"awG",%nobits,GFSM<Devices<void, Mcu::Stm::Stm32G431> >::a,comdat
	.align	1
	.type	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::a, %object
	.size	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::a, 2
GFSM<Devices<void, Mcu::Stm::Stm32G431> >::a:
	.space	2
	.weak	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::r
	.section	.bss._ZN4GFSMI7DevicesIvN3Mcu3Stm9Stm32G431EEE1rE,"awG",%nobits,GFSM<Devices<void, Mcu::Stm::Stm32G431> >::r,comdat
	.align	2
	.type	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::r, %object
	.size	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::r, 4
GFSM<Devices<void, Mcu::Stm::Stm32G431> >::r:
	.space	4
	.weak	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::c
	.section	.bss._ZN4GFSMI7DevicesIvN3Mcu3Stm9Stm32G431EEE1cE,"awG",%nobits,GFSM<Devices<void, Mcu::Stm::Stm32G431> >::c,comdat
	.align	2
	.type	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::c, %object
	.size	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::c, 4
GFSM<Devices<void, Mcu::Stm::Stm32G431> >::c:
	.space	4
	.weak	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue
	.section	.bss._ZN3Mcu3Stm11SystemTimerINS0_5ClockINS0_11ClockConfigIXtlN5Units9frequencyItSt5ratioILx1ELx1000000EEEELm170EEEXtlNS5_ImS6_ILx1ELx1EEEELm1000EEENS0_3HSIEEENS0_9Stm32G431EEESt17integral_constantIbLb0EESD_E6mValueE,"awG",%nobits,Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue,comdat
	.align	2
	.type	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue, %object
	.size	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue, 4
Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue:
	.space	4
