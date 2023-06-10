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
@ options passed: -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -march=armv7e-m+fp -O3 -std=c++23 -fno-exceptions -fno-unwind-tables -fno-rtti -fno-threadsafe-statics -funsigned-char -funsigned-bitfields -fshort-enums -fconcepts -ftemplate-depth=2048 -fstrict-aliasing
	.text
	.section	.text._ZN5STM325ClockINS_11ClockConfigILm170000000ENS_3HSIEEE9Stm32G431E4initEv,"axG",%progbits,STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>::init(),comdat
	.align	1
	.p2align 2,,3
	.weak	STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>::init()
	.syntax unified
	.thumb
	.thumb_func
	.type	STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>::init(), %function
STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>::init():
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:40:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	ldr	r3, .L12	@ tmp144,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:43:             PWR->CR5 &= PWR_CR5_R1MODE;
	ldr	r2, .L12+4	@ tmp148,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:40:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	ldr	r1, [r3, #96]	@ _1, MEM[(struct RCC_TypeDef *)1073876992B].APB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:40:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	orr	r1, r1, #1	@ _2, _1,
	str	r1, [r3, #96]	@ _2, MEM[(struct RCC_TypeDef *)1073876992B].APB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:41:             RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	ldr	r1, [r3, #88]	@ _3, MEM[(struct RCC_TypeDef *)1073876992B].APB1ENR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:41:             RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	orr	r1, r1, #268435456	@ _4, _3,
	str	r1, [r3, #88]	@ _4, MEM[(struct RCC_TypeDef *)1073876992B].APB1ENR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:43:             PWR->CR5 &= PWR_CR5_R1MODE;
	ldr	r1, [r2, #128]	@ _5, MEM[(struct PWR_TypeDef *)1073770496B].CR5
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:43:             PWR->CR5 &= PWR_CR5_R1MODE;
	and	r1, r1, #256	@ _6, _5,
	str	r1, [r2, #128]	@ _6, MEM[(struct PWR_TypeDef *)1073770496B].CR5
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:44:             PWR->CR1 |= PWR_CR1_VOS_0;
	ldr	r1, [r2]	@ _7, MEM[(struct PWR_TypeDef *)1073770496B].CR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:44:             PWR->CR1 |= PWR_CR1_VOS_0;
	orr	r1, r1, #512	@ _8, _7,
	str	r1, [r2]	@ _8, MEM[(struct PWR_TypeDef *)1073770496B].CR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:46:             RCC->CR |= RCC_CR_HSION;
	ldr	r2, [r3]	@ _9, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:46:             RCC->CR |= RCC_CR_HSION;
	orr	r2, r2, #256	@ _10, _9,
	str	r2, [r3]	@ _10, MEM[(struct RCC_TypeDef *)1073876992B].CR
.L2:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:47:             while (!(RCC->CR & RCC_CR_HSIRDY));
	ldr	r2, [r3]	@ _11, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:47:             while (!(RCC->CR & RCC_CR_HSIRDY));
	lsls	r2, r2, #21	@, _11,
	bpl	.L2		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:49:             RCC->PLLCFGR |= (Config::pllP << RCC_PLLCFGR_PLLPDIV_Pos);
	ldr	r1, [r3, #12]	@ _13, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:56:             while (!(RCC->CR & RCC_CR_PLLRDY));
	ldr	r2, .L12	@ tmp168,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:49:             RCC->PLLCFGR |= (Config::pllP << RCC_PLLCFGR_PLLPDIV_Pos);
	orr	r1, r1, #268435456	@ _14, _13,
	str	r1, [r3, #12]	@ _14, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:50:             RCC->PLLCFGR |= (Config::pllQ << RCC_PLLCFGR_PLLQ_Pos);
	ldr	r1, [r3, #12]	@ _15, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:50:             RCC->PLLCFGR |= (Config::pllQ << RCC_PLLCFGR_PLLQ_Pos);
	orr	r1, r1, #4194304	@ _16, _15,
	str	r1, [r3, #12]	@ _16, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:51:             RCC->PLLCFGR |= (Config::pllR << RCC_PLLCFGR_PLLR_Pos);
	ldr	r1, [r3, #12]	@ _17, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:51:             RCC->PLLCFGR |= (Config::pllR << RCC_PLLCFGR_PLLR_Pos);
	orr	r1, r1, #67108864	@ _18, _17,
	str	r1, [r3, #12]	@ _18, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:52:             RCC->PLLCFGR |= (Config::pllN << RCC_PLLCFGR_PLLN_Pos);
	ldr	r1, [r3, #12]	@ _19, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:52:             RCC->PLLCFGR |= (Config::pllN << RCC_PLLCFGR_PLLN_Pos);
	orr	r1, r1, #21760	@ _20, _19,
	str	r1, [r3, #12]	@ _20, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:53:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos);
	ldr	r1, [r3, #12]	@ _21, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:53:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos);
	orr	r1, r1, #64	@ _22, _21,
	str	r1, [r3, #12]	@ _22, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:55:             RCC->CR |= RCC_CR_PLLON;
	ldr	r1, [r3]	@ _23, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:55:             RCC->CR |= RCC_CR_PLLON;
	orr	r1, r1, #16777216	@ _24, _23,
	str	r1, [r3]	@ _24, MEM[(struct RCC_TypeDef *)1073876992B].CR
.L3:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:56:             while (!(RCC->CR & RCC_CR_PLLRDY));
	ldr	r3, [r2]	@ _25, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:56:             while (!(RCC->CR & RCC_CR_PLLRDY));
	lsls	r3, r3, #6	@, _25,
	bpl	.L3		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:58:             RCC->CFGR |= RCC_CFGR_SW_PLL;
	ldr	r3, [r2, #8]	@ _27, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:59:             while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
	ldr	r1, .L12	@ tmp172,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:58:             RCC->CFGR |= RCC_CFGR_SW_PLL;
	orr	r3, r3, #3	@ _28, _27,
	str	r3, [r2, #8]	@ _28, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
.L4:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:59:             while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
	ldr	r3, [r1, #8]	@ _29, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:59:             while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
	tst	r3, #12	@ _29,
	beq	.L4		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:60:         }        
	bx	lr	@
.L13:
	.align	2
.L12:
	.word	1073876992
	.word	1073770496
	.size	STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>::init(), .-STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>::init()
	.section	.text.startup,"ax",%progbits
	.align	1
	.p2align 2,,3
	.syntax unified
	.thumb
	.thumb_func
	.type	main, %function
main:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}	@
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:67:             SysTick->LOAD = Clock::config::systick;
	mov	r4, #-536813568	@ tmp122,
	ldr	r3, .L21	@ tmp123,
	str	r3, [r4, #20]	@ tmp123, MEM[(struct SysTick_Type *)3758153744B].LOAD
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:68:             SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos);
	ldr	r3, [r4, #16]	@ _3, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:68:             SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos);
	orr	r3, r3, #1	@ _4, _3,
	str	r3, [r4, #16]	@ _4, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:111:         devs::clock::init();
	bl	STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>::init()		@
	ldr	r2, .L21+4	@ tmp133,
.L15:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:76:             if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
	ldr	r3, [r4, #16]	@ _5, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:76:             if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
	lsls	r3, r3, #15	@, _5,
	bpl	.L15		@,
.L20:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:77:                 ++mValue;
	ldr	r3, [r2]	@ mValue, mValue
	adds	r3, r3, #1	@ tmp130, mValue,
	str	r3, [r2]	@ tmp130, mValue
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:140:             r = devs::systemTimer::value;
	ldr	r3, [r2]	@ _9, MEM[(volatile long unsigned int &)&mValue]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:76:             if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
	ldr	r3, [r4, #16]	@ _5, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:76:             if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
	lsls	r3, r3, #15	@, _5,
	bpl	.L15		@,
	b	.L20		@
.L22:
	.align	2
.L21:
	.word	170000
	.word	STM32::SystemTimer<STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>, std::integral_constant<bool, false>, Stm32G431>::mValue
	.size	main, .-main
	.weak	STM32::SystemTimer<STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>, std::integral_constant<bool, false>, Stm32G431>::mValue
	.section	.bss._ZN5STM3211SystemTimerINS_5ClockINS_11ClockConfigILm170000000ENS_3HSIEEE9Stm32G431EESt17integral_constantIbLb0EES5_E6mValueE,"awG",%nobits,STM32::SystemTimer<STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>, std::integral_constant<bool, false>, Stm32G431>::mValue,comdat
	.align	2
	.type	STM32::SystemTimer<STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>, std::integral_constant<bool, false>, Stm32G431>::mValue, %object
	.size	STM32::SystemTimer<STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>, std::integral_constant<bool, false>, Stm32G431>::mValue, 4
STM32::SystemTimer<STM32::Clock<STM32::ClockConfig<170000000ul, STM32::HSI>, Stm32G431>, std::integral_constant<bool, false>, Stm32G431>::mValue:
	.space	4
	.bss
	.align	2
	.type	r, %object
	.size	r, 4
r:
	.space	4
