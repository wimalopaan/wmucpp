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
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:34:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	ldr	r3, .L12	@ tmp158,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:39:             FLASH->ACR |= FLASH_ACR_LATENCY_5WS 
	ldr	r0, .L12+4	@ tmp164,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:34:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	ldr	r2, [r3, #96]	@ _28, MEM[(struct RCC_TypeDef *)1073876992B].APB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:44:             PWR->CR5 &= PWR_CR5_R1MODE;
	ldr	r1, .L12+8	@ tmp167,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:34:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	orr	r2, r2, #1	@ _29, _28,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/devices.h:26:     static inline void init() {
	push	{r4}	@
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:34:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	str	r2, [r3, #96]	@ _29, MEM[(struct RCC_TypeDef *)1073876992B].APB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:35:             RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	ldr	r2, [r3, #88]	@ _30, MEM[(struct RCC_TypeDef *)1073876992B].APB1ENR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:35:             RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	orr	r2, r2, #268435456	@ _31, _30,
	str	r2, [r3, #88]	@ _31, MEM[(struct RCC_TypeDef *)1073876992B].APB1ENR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:37:             RCC->CFGR |= RCC_CFGR_HPRE_DIV2;            
	ldr	r2, [r3, #8]	@ _32, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:37:             RCC->CFGR |= RCC_CFGR_HPRE_DIV2;            
	orr	r2, r2, #128	@ _33, _32,
	str	r2, [r3, #8]	@ _33, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:39:             FLASH->ACR |= FLASH_ACR_LATENCY_5WS 
	ldr	r2, [r0]	@ _34, MEM[(struct FLASH_TypeDef *)1073881088B].ACR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:39:             FLASH->ACR |= FLASH_ACR_LATENCY_5WS 
	orr	r2, r2, #1792	@ _35, _34,
	orr	r2, r2, #5	@ _35, _35,
	str	r2, [r0]	@ _35, MEM[(struct FLASH_TypeDef *)1073881088B].ACR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:44:             PWR->CR5 &= PWR_CR5_R1MODE;
	ldr	r2, [r1, #128]	@ _36, MEM[(struct PWR_TypeDef *)1073770496B].CR5
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:44:             PWR->CR5 &= PWR_CR5_R1MODE;
	and	r2, r2, #256	@ _37, _36,
	str	r2, [r1, #128]	@ _37, MEM[(struct PWR_TypeDef *)1073770496B].CR5
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:45:             PWR->CR1 |= PWR_CR1_VOS_0;
	ldr	r2, [r1]	@ _38, MEM[(struct PWR_TypeDef *)1073770496B].CR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:45:             PWR->CR1 |= PWR_CR1_VOS_0;
	orr	r2, r2, #512	@ _39, _38,
	str	r2, [r1]	@ _39, MEM[(struct PWR_TypeDef *)1073770496B].CR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:47:             RCC->CR |= RCC_CR_HSION;
	ldr	r2, [r3]	@ _40, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:47:             RCC->CR |= RCC_CR_HSION;
	orr	r2, r2, #256	@ _41, _40,
	str	r2, [r3]	@ _41, MEM[(struct RCC_TypeDef *)1073876992B].CR
.L2:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:48:             while (!(RCC->CR & RCC_CR_HSIRDY));
	ldr	r2, [r3]	@ _42, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:48:             while (!(RCC->CR & RCC_CR_HSIRDY));
	lsls	r2, r2, #21	@, _42,
	bpl	.L2		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:50:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos) 
	ldr	r0, [r3, #12]	@ _44, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:50:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos) 
	ldr	r2, .L12+12	@ _45,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:58:             while (!(RCC->CR & RCC_CR_PLLRDY));
	ldr	r1, .L12	@ tmp180,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:50:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos) 
	orrs	r2, r2, r0	@, _45, _45, _44
	str	r2, [r3, #12]	@ _45, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:57:             RCC->CR |= RCC_CR_PLLON;
	ldr	r2, [r3]	@ _46, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:57:             RCC->CR |= RCC_CR_PLLON;
	orr	r2, r2, #16777216	@ _47, _46,
	str	r2, [r3]	@ _47, MEM[(struct RCC_TypeDef *)1073876992B].CR
.L3:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:58:             while (!(RCC->CR & RCC_CR_PLLRDY));
	ldr	r3, [r1]	@ _48, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:58:             while (!(RCC->CR & RCC_CR_PLLRDY));
	lsls	r3, r3, #6	@, _48,
	bpl	.L3		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:60:             RCC->CFGR |= RCC_CFGR_SW_PLL;
	ldr	r3, [r1, #8]	@ _50, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:62:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	ldr	r2, .L12	@ tmp184,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:60:             RCC->CFGR |= RCC_CFGR_SW_PLL;
	orr	r3, r3, #3	@ _51, _50,
	str	r3, [r1, #8]	@ _51, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
.L4:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:62:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	ldr	r3, [r2, #8]	@ _52, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:62:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	and	r3, r3, #12	@ tmp185, _52,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:62:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	cmp	r3, #12	@ tmp185,
	bne	.L4		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:64:             MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);            
	ldr	r3, [r2, #8]	@ _54, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/arm.h:9:             TPI->ACPR = Clock::config::f / f - 1;
	ldr	r0, .L12+16	@ tmp188,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:72:             SysTick->LOAD = Clock::config::systick;
	mov	r1, #-536813568	@ tmp190,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:64:             MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);            
	bic	r3, r3, #240	@ _55, _54,
	str	r3, [r2, #8]	@ _55, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/arm.h:9:             TPI->ACPR = Clock::config::f / f - 1;
	movs	r4, #84	@ tmp189,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:72:             SysTick->LOAD = Clock::config::systick;
	ldr	r3, .L12+20	@ tmp191,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/arm.h:9:             TPI->ACPR = Clock::config::f / f - 1;
	str	r4, [r0, #16]	@ tmp189, MEM[(struct TPI_Type *)3758358528B].ACPR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:72:             SysTick->LOAD = Clock::config::systick;
	str	r3, [r1, #20]	@ tmp191, MEM[(struct SysTick_Type *)3758153744B].LOAD
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:73:             SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos) 
	ldr	r0, [r1, #16]	@ _17, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:26:             auto temp = mcuGpio->MODER;
	ldr	r3, .L12+24	@ tmp196,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/devices.h:37:     }
	ldr	r4, [sp], #4	@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:73:             SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos) 
	orr	r0, r0, #5	@ _18, _17,
	str	r0, [r1, #16]	@ _18, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:10:             RCC->AHB2ENR |= Letter::ahb2Bit;
	ldr	r1, [r2, #76]	@ _15, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:22:             mcuDac->CR |= DAC_CR_EN1;
	ldr	r0, .L12+28	@ tmp204,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:10:             RCC->AHB2ENR |= Letter::ahb2Bit;
	orr	r1, r1, #2	@ _16, _15,
	str	r1, [r2, #76]	@ _16, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:26:             auto temp = mcuGpio->MODER;
	ldr	r1, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:27:             temp &= ~(GPIO_MODER_MODE0 << moderPos);
	bic	r1, r1, #196608	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:29:                 temp |= 0x01UL << moderPos;
	orr	r1, r1, #65536	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:31:             mcuGpio->MODER = temp;
	str	r1, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:26:             auto temp = mcuGpio->MODER;
	ldr	r1, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:27:             temp &= ~(GPIO_MODER_MODE0 << moderPos);
	bic	r1, r1, #768	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:29:                 temp |= 0x01UL << moderPos;
	orr	r1, r1, #256	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:31:             mcuGpio->MODER = temp;
	str	r1, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:22:             mcuGpio->OSPEEDR |= (0x11 << moderPos);
	ldr	r1, [r3, #8]	@ _7, MEM[(struct GPIO_TypeDef *)1207960576B].OSPEEDR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:22:             mcuGpio->OSPEEDR |= (0x11 << moderPos);
	orr	r1, r1, #4352	@ _8, _7,
	str	r1, [r3, #8]	@ _8, MEM[(struct GPIO_TypeDef *)1207960576B].OSPEEDR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:20:                 RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
	ldr	r3, [r2, #76]	@ _3, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:20:                 RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
	orr	r3, r3, #65536	@ _4, _3,
	str	r3, [r2, #76]	@ _4, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:22:             mcuDac->CR |= DAC_CR_EN1;
	ldr	r3, [r0]	@ _5, MEM[(struct DAC_TypeDef *)1342179328B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:22:             mcuDac->CR |= DAC_CR_EN1;
	orr	r3, r3, #1	@ _6, _5,
	str	r3, [r0]	@ _6, MEM[(struct DAC_TypeDef *)1342179328B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/devices.h:37:     }
	bx	lr	@
.L13:
	.align	2
.L12:
	.word	1073876992
	.word	1073881088
	.word	1073770496
	.word	289428786
	.word	-536608768
	.word	170000
	.word	1207960576
	.word	1342179328
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
	@ args = 0, pretend = 0, frame = 224
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}	@
	sub	sp, sp, #228	@,,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:10:         devs::init();
	bl	Devices<void, Mcu::Stm::Stm32G431>::init()		@
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:34:             mcuGpio->BSRR |= (0x01UL << N);
	ldr	r2, .L200	@ tmp243,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:83:                 ++mValue;
	ldr	fp, .L200+32	@ tmp471,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:34:             mcuGpio->BSRR |= (0x01UL << N);
	ldr	r3, [r2, #24]	@ _3, MEM[(struct GPIO_TypeDef *)1207960576B].BSRR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         ++a;
	ldr	r10, .L200+36	@ tmp473,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	ldr	r7, .L200+4	@ tmp482,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	ldr	r8, .L200+40	@ tmp483,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	ldr	r9, .L200+44	@ tmp484,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:34:             mcuGpio->BSRR |= (0x01UL << N);
	orr	r3, r3, #256	@ _4, _3,
	str	r3, [r2, #24]	@ _4, MEM[(struct GPIO_TypeDef *)1207960576B].BSRR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:20:         if (++c == 1000) {
	ldr	r3, .L200+8	@ tmp518,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:83:                 ++mValue;
	ldr	lr, [fp]	@ _8, mValue
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:20:         if (++c == 1000) {
	ldr	ip, [r3]	@ _22, c
	ldr	r3, .L200+12	@ tmp519,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         ++a;
	ldrh	r2, [r10]	@ _19, a
	ldr	r0, [r3]	@ ivtmp.210, r
	add	r5, sp, #20	@ tmp478,,
	add	r6, sp, #18	@ _464,,
	mov	r4, lr	@ _8, _8
.L85:
	adds	r3, r0, #1	@ ivtmp.210, ivtmp.210,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:82:             if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
	mov	lr, #-536813568	@ tmp249,
.L15:
	ldr	r1, [lr, #16]	@ _5, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:82:             if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
	lsls	r1, r1, #15	@, _5,
	mov	r0, r3	@ ivtmp.210, ivtmp.210
	add	r3, r3, #1	@ ivtmp.210, ivtmp.210,
	bpl	.L15		@,
	ldr	r3, .L200+12	@ tmp520,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         ++a;
	adds	r2, r2, #1	@ tmp253, _19,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:20:         if (++c == 1000) {
	add	ip, ip, #1	@ _22, _22,
	str	r0, [r3]	@ ivtmp.210, r
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:25:             mcuDac->DHR12R1 = v;
	ldr	r3, .L200+16	@ tmp259,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:18:         a &= 0x0fff;
	ubfx	r2, r2, #0, #12	@ _19, tmp253,,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:83:                 ++mValue;
	adds	r4, r4, #1	@ _8, _8,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:20:         if (++c == 1000) {
	cmp	ip, #1000	@ _22,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:83:                 ++mValue;
	str	r4, [fp]	@ _8, mValue
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:18:         a &= 0x0fff;
	strh	r2, [r10]	@ movhi	@ _19, a
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/dac.h:25:             mcuDac->DHR12R1 = v;
	str	r2, [r3, #8]	@ _19, MEM[(struct DAC_TypeDef *)1342179328B].DHR12R1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:20:         if (++c == 1000) {
	beq	.L187		@,
.L16:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:40:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	ldr	r1, .L200	@ tmp465,
	ldr	r3, [r1, #20]	@ _23, MEM[(struct GPIO_TypeDef *)1207960576B].ODR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:40:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	eor	r3, r3, #16	@ tmp466, _23,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:40:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	orr	r3, r3, #1048576	@ _25, tmp466,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:40:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	str	r3, [r1, #24]	@ _25, MEM[(struct GPIO_TypeDef *)1207960576B].BSRR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:26:     }
	b	.L85		@
.L187:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:21:             c = 0;
	ldr	r2, .L200+8	@ tmp521,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	r0, .L200+20	@ p,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:21:             c = 0;
	movs	r3, #0	@ tmp261,
	str	r3, [r2]	@ tmp261, c
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	movs	r1, #115	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r3, #-536870912	@ tmp262,
	b	.L17		@
.L19:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r1, [r0], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_46 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r1, .L18	@ c,
.L17:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r2, [r3, #3712]	@ _48, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r4, r2, #31	@, _48,
	bpl	.L19		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r2, [r3, #3584]	@ _51, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r2, r2, #31	@, _51,
	bpl	.L19		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r2, [r3]	@ _53,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r2, .L188	@ _53,
.L20:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r2, [r3]	@ _53,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r2, #0	@ _53
	beq	.L20	@
.L188:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r1, [r3]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r1, [r0], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_46 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r1, #0	@ c
	bne	.L17	@
.L18:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:38:             std::to_chars(std::begin(buffer), std::end(buffer), v);
	ldr	r4, [fp]	@ __val, MEM[(const volatile long unsigned int &)&mValue]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	strh	r1, [sp, #16]	@ movhi	@ c, MEM[(struct array *)_30]
	strd	r1, r1, [sp, #8]	@ c, c,,
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ __val
	bne	.L22	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r3, #48	@ tmp274,
	strb	r3, [sp, #8]	@ tmp274, MEM[(char *)_30]
.L23:
	add	r1, sp, #8	@ ivtmp.192,,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r3, #-536870912	@ tmp481,
	b	.L39		@
.L36:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r6, r1	@ _464, ivtmp.192
	beq	.L35		@,
.L39:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r0, [r1], #1	@ zero_extendqisi2	@ _57, MEM[(const value_type &)_301]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r0, .L35	@ _57,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r2, [r3, #3712]	@ _58, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r4, r2, #31	@, _58,
	bpl	.L36		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r2, [r3, #3584]	@ _99, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r2, r2, #31	@, _99,
	bpl	.L36		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r2, [r3]	@ _101,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r2, .L189	@ _101,
.L37:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r2, [r3]	@ _101,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r2, #0	@ _101
	beq	.L37	@
.L189:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r6, r1	@ _464, ivtmp.192
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r0, [r3]	@ _57, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	bne	.L39		@,
.L35:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	r0, .L200+24	@ p,
	movs	r1, #32	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r3, #-536870912	@ tmp323,
	b	.L40		@
.L42:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r1, [r0], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_39 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r1, .L41	@ c,
.L40:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r2, [r3, #3712]	@ _41, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r4, r2, #31	@, _41,
	bpl	.L42		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r2, [r3, #3584]	@ _102, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r2, r2, #31	@, _102,
	bpl	.L42		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r2, [r3]	@ _104,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r2, .L190	@ _104,
.L43:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r2, [r3]	@ _104,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r2, #0	@ _104
	beq	.L43	@
.L190:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r1, [r3]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r1, [r0], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_39 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r1, #0	@ c
	bne	.L40	@
.L41:
	ldr	r3, .L200+12	@ tmp523,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	strh	r1, [sp, #16]	@ movhi	@ c, MEM[(struct array *)_30]
	ldr	r4, [r3]	@ __val, r
	strd	r1, r1, [sp, #8]	@ c, c,,
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ __val
	bne	.L45	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r3, #48	@ tmp335,
	strb	r3, [sp, #8]	@ tmp335, MEM[(char *)_30]
.L46:
	add	r1, sp, #8	@ ivtmp.168,,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r3, #-536870912	@ tmp480,
	b	.L62		@
.L59:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r1, r6	@ ivtmp.168, _464
	beq	.L58		@,
.L62:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r0, [r1], #1	@ zero_extendqisi2	@ _108, MEM[(const value_type &)_456]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r0, .L58	@ _108,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r2, [r3, #3712]	@ _109, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r4, r2, #31	@, _109,
	bpl	.L59		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r2, [r3, #3584]	@ _150, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r2, r2, #31	@, _150,
	bpl	.L59		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r2, [r3]	@ _152,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r2, .L191	@ _152,
.L60:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r2, [r3]	@ _152,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r2, #0	@ _152
	beq	.L60	@
.L191:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r1, r6	@ ivtmp.168, _464
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r0, [r3]	@ _108, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	bne	.L62		@,
.L58:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	r0, .L200+28	@ p,
	movs	r1, #32	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r3, #-536870912	@ tmp384,
	b	.L63		@
.L65:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r1, [r0], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_32 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r1, .L64	@ c,
.L63:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r2, [r3, #3712]	@ _34, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r4, r2, #31	@, _34,
	bpl	.L65		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r2, [r3, #3584]	@ _153, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r2, r2, #31	@, _153,
	bpl	.L65		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r2, [r3]	@ _155,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r2, .L192	@ _155,
.L66:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r2, [r3]	@ _155,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r2, #0	@ _155
	beq	.L66	@
.L192:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r1, [r3]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r1, [r0], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_32 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r1, #0	@ c
	bne	.L63	@
.L64:
	ldrh	r4, [r10]	@ _29, a
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	str	r1, [sp, #8]	@ c, MEM[(struct array *)_30]
	strb	r1, [sp, #12]	@ c, MEM[(struct array *)_30]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ _29
	bne	.L68	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r3, #48	@ tmp396,
	strb	r3, [sp, #8]	@ tmp396, MEM[(char *)_30]
.L69:
	add	r2, sp, #8	@ ivtmp.147,,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r4, #-536870912	@ tmp479,
.L81:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r1, [r2], #1	@ zero_extendqisi2	@ _159, MEM[(const value_type &)_442]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r1, .L77	@ _159,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r3, [r4, #3712]	@ _160, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r3, r3, #31	@, _160,
	bpl	.L78		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r3, [r4, #3584]	@ _221, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r0, r3, #31	@, _221,
	bpl	.L78		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r3, [r4]	@ _61, MEM[(struct ITM_Type *)3758096384B].PORT[0].u32
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r3, .L80	@ _61,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	mov	r0, #-536870912	@ tmp452,
.L79:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r3, [r0]	@ _223, MEM[(struct ITM_Type *)3758096384B].PORT[0].u32
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r3, #0	@ _223
	beq	.L79	@
.L80:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	mov	r3, #-536870912	@ tmp450,
	strb	r1, [r3]	@ _159, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
.L78:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	add	r3, sp, #13	@ tmp526,,
	cmp	r2, r3	@ ivtmp.147, tmp526
	bne	.L81		@,
.L77:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r3, #-536870912	@ tmp453,
	ldr	r2, [r3, #3712]	@ _26, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r2, #31	@, _26,
	bpl	.L82		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r2, [r3, #3584]	@ _224, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r2, r2, #31	@, _224,
	bmi	.L186		@,
.L82:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:20:         if (++c == 1000) {
	ldr	r3, .L200+8	@ tmp527,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:83:                 ++mValue;
	ldr	r4, [fp]	@ _8, mValue
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         ++a;
	ldrh	r2, [r10]	@ _19, a
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:20:         if (++c == 1000) {
	ldr	ip, [r3]	@ _22, c
	movs	r0, #0	@ ivtmp.210,
	b	.L16		@
.L83:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
	.thumb
	.syntax unified
.L186:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r2, [r3]	@ _226,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r2, #0	@ _226
	beq	.L83	@
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	mov	r3, #-536870912	@ tmp458,
	movs	r2, #10	@ tmp459,
	strb	r2, [r3]	@ tmp459, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
	b	.L82		@
.L201:
	.align	2
.L200:
	.word	1207960576
	.word	1374389535
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::c
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::r
	.word	1342179328
	.word	.LC0+1
	.word	.LC1+1
	.word	.LC2+1
	.word	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::a
	.word	99999999
	.word	999999999
.L68:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ __unsigned_val,
	bls	.L70		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ __unsigned_val,
	bls	.L193		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	cmp	r4, #1000	@ __unsigned_val,
	bcc	.L96		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movw	r3, #9999	@ tmp403,
	cmp	r4, r3	@ __unsigned_val, tmp403
	ite	ls		@
	movls	r3, #3	@, prephitmp_213,
	movhi	r3, #4	@, prephitmp_213,
.L74:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	movs	r2, #201	@,
	ldr	r1, .L202	@,
	str	r3, [sp, #4]	@ prephitmp_213, %sfp
	mov	r0, r5	@, tmp478
	bl	memcpy		@
	ldr	r3, [sp, #4]	@ prephitmp_213, %sfp
	adds	r3, r3, #7	@ tmp531, prephitmp_213,
	add	r2, sp, r3	@ ivtmp.153,, tmp531
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	mov	lr, #100	@ tmp415,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	movw	ip, #9999	@ tmp437,
.L75:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r1, r3, r7, r4	@ tmp487, tmp412, tmp482, __unsigned_val
	lsrs	r3, r3, #5	@ tmp411, tmp412,
	mls	r0, lr, r3, r4	@ tmp416, tmp415, tmp411, __unsigned_val
	mov	r1, r4	@ __val, __unsigned_val
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r0, [r5, r0, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.136_428 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r0, [r2], #-2	@ unaligned	@ vect__212.137, MEM <vector(2) char> [(char *)vectp.139_425]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	cmp	r1, ip	@ __val, tmp437
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:97: 	  __val /= 100;
	mov	r4, r3	@ __unsigned_val, tmp411
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	bhi	.L75		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:102:       if (__val >= 10)
	cmp	r1, #1000	@ __val,
	bcc	.L70		@,
.L72:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r3, sp, #224	@ tmp525,,
	add	r4, r3, r4, lsl #1	@ tmp438, tmp525, __unsigned_val,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r3, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_344][_216], MEM[(char[201] *)_344][_216]
	strb	r3, [sp, #9]	@ MEM[(char[201] *)_344][_216], MEM[(char *)_30]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r3, [r4, #-204]	@ zero_extendqisi2	@ _218, MEM[(char[201] *)_344][__num_215]
.L76:
	strb	r3, [sp, #8]	@ _218, MEM[(char *)_30]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:235:       return __res;
	b	.L69		@
.L22:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ __val,
	bls	.L24		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ __val,
	bls	.L194		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	cmp	r4, #1000	@ __val,
	bcc	.L86		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movw	r3, #9999	@ tmp281,
	cmp	r4, r3	@ __val, tmp281
	bls	.L195		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	ldr	r3, .L202+4	@ tmp286,
	cmp	r4, r3	@ __val, tmp286
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	it	ls
	movls	r3, #5	@ _74,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	bls	.L27		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	ldr	r3, .L202+8	@ tmp282,
	cmp	r4, r3	@ __val, tmp282
	bls	.L196		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	ldr	r3, .L202+12	@ tmp283,
	cmp	r4, r3	@ __val, tmp283
	bcc	.L88		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	cmp	r4, r8	@ __val, tmp483
	bls	.L89		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, r9	@ __val, tmp484
	bls	.L90		@,
	movs	r3, #5	@ __n,
.L31:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	adds	r3, r3, #5	@ _74, __n,
.L27:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	movs	r2, #201	@,
	ldr	r1, .L202	@,
	str	r3, [sp, #4]	@ _74, %sfp
	mov	r0, r5	@, tmp478
	bl	memcpy		@
	ldr	r3, [sp, #4]	@ _74, %sfp
	adds	r3, r3, #6	@ tmp533, _74,
	add	r2, sp, r3	@ ivtmp.197,, tmp533
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	mov	lr, #100	@ tmp298,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	movw	ip, #9999	@ tmp307,
.L33:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r1, r3, r7, r4	@ tmp485, tmp295, tmp482, __val
	lsrs	r3, r3, #5	@ tmp294, tmp295,
	mls	r0, lr, r3, r4	@ tmp299, tmp298, tmp294, __val
	mov	r1, r4	@ __val, __val
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r0, [r5, r0, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.126_438 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r0, [r2], #-2	@ unaligned	@ vect__90.127, MEM <vector(2) char> [(char *)vectp.129_435]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	cmp	r1, ip	@ __val, tmp307
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:97: 	  __val /= 100;
	mov	r4, r3	@ __val, tmp294
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	bhi	.L33		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:102:       if (__val >= 10)
	cmp	r1, #1000	@ __val,
	bcc	.L24		@,
.L26:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r3, sp, #224	@ tmp522,,
	add	r4, r3, r4, lsl #1	@ tmp308, tmp522, __val,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r3, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_344][_94], MEM[(char[201] *)_344][_94]
	strb	r3, [sp, #9]	@ MEM[(char[201] *)_344][_94], MEM[(char *)_30]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r3, [r4, #-204]	@ zero_extendqisi2	@ _96, MEM[(char[201] *)_344][__num_93]
.L34:
	strb	r3, [sp, #8]	@ _96, MEM[(char *)_30]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:235:       return __res;
	b	.L23		@
.L45:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ __val,
	bls	.L47		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ __val,
	bls	.L197		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	cmp	r4, #1000	@ __val,
	bcc	.L91		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movw	r3, #9999	@ tmp342,
	cmp	r4, r3	@ __val, tmp342
	bls	.L198		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	ldr	r3, .L202+4	@ tmp347,
	cmp	r4, r3	@ __val, tmp347
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	it	ls
	movls	r3, #5	@ _125,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	bls	.L50		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	ldr	r3, .L202+8	@ tmp343,
	cmp	r4, r3	@ __val, tmp343
	bls	.L199		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	ldr	r3, .L202+12	@ tmp344,
	cmp	r4, r3	@ __val, tmp344
	bcc	.L93		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	cmp	r4, r8	@ __val, tmp483
	bls	.L94		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, r9	@ __val, tmp484
	bls	.L95		@,
	movs	r3, #5	@ __n,
.L54:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	adds	r3, r3, #5	@ _125, __n,
.L50:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	movs	r2, #201	@,
	ldr	r1, .L202	@,
	str	r3, [sp, #4]	@ _125, %sfp
	mov	r0, r5	@, tmp478
	bl	memcpy		@
	ldr	r3, [sp, #4]	@ _125, %sfp
	adds	r3, r3, #6	@ tmp532, _125,
	add	r2, sp, r3	@ ivtmp.173,, tmp532
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	mov	lr, #100	@ tmp359,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	movw	ip, #9999	@ tmp368,
.L56:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r1, r3, r7, r4	@ tmp486, tmp356, tmp482, __val
	lsrs	r3, r3, #5	@ tmp355, tmp356,
	mls	r0, lr, r3, r4	@ tmp360, tmp359, tmp355, __val
	mov	r1, r4	@ __val, __val
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r0, [r5, r0, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.131_433 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r0, [r2], #-2	@ unaligned	@ vect__141.132, MEM <vector(2) char> [(char *)vectp.134_430]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	cmp	r1, ip	@ __val, tmp368
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:97: 	  __val /= 100;
	mov	r4, r3	@ __val, tmp355
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	bhi	.L56		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:102:       if (__val >= 10)
	cmp	r1, #1000	@ __val,
	bcc	.L47		@,
.L49:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r3, sp, #224	@ tmp524,,
	add	r4, r3, r4, lsl #1	@ tmp369, tmp524, __val,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r3, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_344][_145], MEM[(char[201] *)_344][_145]
	strb	r3, [sp, #9]	@ MEM[(char[201] *)_344][_145], MEM[(char *)_30]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r3, [r4, #-204]	@ zero_extendqisi2	@ _147, MEM[(char[201] *)_344][__num_144]
.L57:
	strb	r3, [sp, #8]	@ _147, MEM[(char *)_30]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:235:       return __res;
	b	.L46		@
.L47:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:109: 	__first[0] = '0' + __val;
	add	r3, r4, #48	@ tmp374, __val,
	uxtb	r3, r3	@ _147, tmp374
	b	.L57		@
.L24:
	add	r3, r4, #48	@ tmp313, __val,
	uxtb	r3, r3	@ _96, tmp313
	b	.L34		@
.L70:
	add	r3, r4, #48	@ tmp443, __unsigned_val,
	uxtb	r3, r3	@ _218, tmp443
	b	.L76		@
.L88:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	movs	r3, #7	@ _74,
	b	.L27		@
.L93:
	movs	r3, #7	@ _125,
	b	.L50		@
.L89:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movs	r3, #8	@ _74,
	b	.L27		@
.L94:
	movs	r3, #8	@ _125,
	b	.L50		@
.L90:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	movs	r3, #9	@ _74,
	b	.L27		@
.L95:
	movs	r3, #9	@ _125,
	b	.L50		@
.L193:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L202	@,
	movs	r2, #201	@,
	mov	r0, r5	@, tmp478
	bl	memcpy		@
	b	.L72		@
.L198:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movs	r3, #4	@ _125,
	b	.L50		@
.L91:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	movs	r3, #3	@ _125,
	b	.L50		@
.L197:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L202	@,
	movs	r2, #201	@,
	mov	r0, r5	@, tmp478
	bl	memcpy		@
	b	.L49		@
.L195:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movs	r3, #4	@ _74,
	b	.L27		@
.L86:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	movs	r3, #3	@ _74,
	b	.L27		@
.L194:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L202	@,
	movs	r2, #201	@,
	mov	r0, r5	@, tmp478
	bl	memcpy		@
	b	.L26		@
.L96:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	movs	r3, #2	@ prephitmp_213,
	b	.L74		@
.L199:
	movs	r3, #1	@ __n,
	b	.L54		@
.L196:
	movs	r3, #1	@ __n,
	b	.L31		@
.L203:
	.align	2
.L202:
	.word	.LC3
	.word	99999
	.word	999999
	.word	10000000
	.size	main, .-main
	.weak	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue
	.section	.bss._ZN3Mcu3Stm11SystemTimerINS0_5ClockINS0_11ClockConfigIXtlN5Units9frequencyItSt5ratioILx1ELx1000000EEEELm170EEEXtlNS5_ImS6_ILx1ELx1EEEELm1000EEENS0_3HSIEEENS0_9Stm32G431EEESt17integral_constantIbLb0EESD_E6mValueE,"awG",%nobits,Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue,comdat
	.align	2
	.type	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue, %object
	.size	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue, 4
Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue:
	.space	4
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
