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
	ldr	r3, .L12	@ tmp154,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:39:             FLASH->ACR |= FLASH_ACR_LATENCY_5WS 
	ldr	r0, .L12+4	@ tmp160,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:34:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	ldr	r2, [r3, #96]	@ _22, MEM[(struct RCC_TypeDef *)1073876992B].APB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:44:             PWR->CR5 &= PWR_CR5_R1MODE;
	ldr	r1, .L12+8	@ tmp163,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:34:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	orr	r2, r2, #1	@ _23, _22,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/devices.h:22:     static inline void init() {
	push	{r4}	@
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:34:             RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;            
	str	r2, [r3, #96]	@ _23, MEM[(struct RCC_TypeDef *)1073876992B].APB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:35:             RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	ldr	r2, [r3, #88]	@ _24, MEM[(struct RCC_TypeDef *)1073876992B].APB1ENR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:35:             RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	orr	r2, r2, #268435456	@ _25, _24,
	str	r2, [r3, #88]	@ _25, MEM[(struct RCC_TypeDef *)1073876992B].APB1ENR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:37:             RCC->CFGR |= RCC_CFGR_HPRE_DIV2;            
	ldr	r2, [r3, #8]	@ _26, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:37:             RCC->CFGR |= RCC_CFGR_HPRE_DIV2;            
	orr	r2, r2, #128	@ _27, _26,
	str	r2, [r3, #8]	@ _27, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:39:             FLASH->ACR |= FLASH_ACR_LATENCY_5WS 
	ldr	r2, [r0]	@ _28, MEM[(struct FLASH_TypeDef *)1073881088B].ACR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:39:             FLASH->ACR |= FLASH_ACR_LATENCY_5WS 
	orr	r2, r2, #1792	@ _29, _28,
	orr	r2, r2, #5	@ _29, _29,
	str	r2, [r0]	@ _29, MEM[(struct FLASH_TypeDef *)1073881088B].ACR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:44:             PWR->CR5 &= PWR_CR5_R1MODE;
	ldr	r2, [r1, #128]	@ _30, MEM[(struct PWR_TypeDef *)1073770496B].CR5
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:44:             PWR->CR5 &= PWR_CR5_R1MODE;
	and	r2, r2, #256	@ _31, _30,
	str	r2, [r1, #128]	@ _31, MEM[(struct PWR_TypeDef *)1073770496B].CR5
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:45:             PWR->CR1 |= PWR_CR1_VOS_0;
	ldr	r2, [r1]	@ _32, MEM[(struct PWR_TypeDef *)1073770496B].CR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:45:             PWR->CR1 |= PWR_CR1_VOS_0;
	orr	r2, r2, #512	@ _33, _32,
	str	r2, [r1]	@ _33, MEM[(struct PWR_TypeDef *)1073770496B].CR1
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:47:             RCC->CR |= RCC_CR_HSION;
	ldr	r2, [r3]	@ _34, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:47:             RCC->CR |= RCC_CR_HSION;
	orr	r2, r2, #256	@ _35, _34,
	str	r2, [r3]	@ _35, MEM[(struct RCC_TypeDef *)1073876992B].CR
.L2:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:48:             while (!(RCC->CR & RCC_CR_HSIRDY));
	ldr	r2, [r3]	@ _36, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:48:             while (!(RCC->CR & RCC_CR_HSIRDY));
	lsls	r2, r2, #21	@, _36,
	bpl	.L2		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:50:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos) 
	ldr	r0, [r3, #12]	@ _38, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:50:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos) 
	ldr	r2, .L12+12	@ _39,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:58:             while (!(RCC->CR & RCC_CR_PLLRDY));
	ldr	r1, .L12	@ tmp176,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:50:             RCC->PLLCFGR |= (Config::pllM << RCC_PLLCFGR_PLLM_Pos) 
	orrs	r2, r2, r0	@, _39, _39, _38
	str	r2, [r3, #12]	@ _39, MEM[(struct RCC_TypeDef *)1073876992B].PLLCFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:57:             RCC->CR |= RCC_CR_PLLON;
	ldr	r2, [r3]	@ _40, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:57:             RCC->CR |= RCC_CR_PLLON;
	orr	r2, r2, #16777216	@ _41, _40,
	str	r2, [r3]	@ _41, MEM[(struct RCC_TypeDef *)1073876992B].CR
.L3:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:58:             while (!(RCC->CR & RCC_CR_PLLRDY));
	ldr	r3, [r1]	@ _42, MEM[(struct RCC_TypeDef *)1073876992B].CR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:58:             while (!(RCC->CR & RCC_CR_PLLRDY));
	lsls	r3, r3, #6	@, _42,
	bpl	.L3		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:60:             RCC->CFGR |= RCC_CFGR_SW_PLL;
	ldr	r3, [r1, #8]	@ _44, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:62:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	ldr	r2, .L12	@ tmp180,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:60:             RCC->CFGR |= RCC_CFGR_SW_PLL;
	orr	r3, r3, #3	@ _45, _44,
	str	r3, [r1, #8]	@ _45, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
.L4:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:62:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	ldr	r3, [r2, #8]	@ _46, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:62:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	and	r3, r3, #12	@ tmp181, _46,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:62:             while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
	cmp	r3, #12	@ tmp181,
	bne	.L4		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:64:             MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);            
	ldr	r3, [r2, #8]	@ _48, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/arm.h:9:             TPI->ACPR = Clock::config::f / f - 1;
	ldr	r0, .L12+16	@ tmp184,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:72:             SysTick->LOAD = Clock::config::systick;
	mov	r1, #-536813568	@ tmp186,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:64:             MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);            
	bic	r3, r3, #240	@ _49, _48,
	str	r3, [r2, #8]	@ _49, MEM[(struct RCC_TypeDef *)1073876992B].CFGR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/arm.h:9:             TPI->ACPR = Clock::config::f / f - 1;
	movs	r4, #84	@ tmp185,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:72:             SysTick->LOAD = Clock::config::systick;
	ldr	r3, .L12+20	@ tmp187,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/arm.h:9:             TPI->ACPR = Clock::config::f / f - 1;
	str	r4, [r0, #16]	@ tmp185, MEM[(struct TPI_Type *)3758358528B].ACPR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:72:             SysTick->LOAD = Clock::config::systick;
	str	r3, [r1, #20]	@ tmp187, MEM[(struct SysTick_Type *)3758153744B].LOAD
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:73:             SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos) 
	ldr	r0, [r1, #16]	@ _13, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:26:             auto temp = mcuGpio->MODER;
	ldr	r3, .L12+24	@ tmp192,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/devices.h:32:     }
	ldr	r4, [sp], #4	@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:73:             SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos) 
	orr	r0, r0, #5	@ _14, _13,
	str	r0, [r1, #16]	@ _14, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:10:             RCC->AHB2ENR |= Letter::ahb2Bit;
	ldr	r1, [r2, #76]	@ _11, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:10:             RCC->AHB2ENR |= Letter::ahb2Bit;
	orr	r1, r1, #2	@ _12, _11,
	str	r1, [r2, #76]	@ _12, MEM[(struct RCC_TypeDef *)1073876992B].AHB2ENR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:26:             auto temp = mcuGpio->MODER;
	ldr	r2, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:27:             temp &= ~(GPIO_MODER_MODE0 << moderPos);
	bic	r2, r2, #196608	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:29:                 temp |= 0x01UL << moderPos;
	orr	r2, r2, #65536	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:31:             mcuGpio->MODER = temp;
	str	r2, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:26:             auto temp = mcuGpio->MODER;
	ldr	r2, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:27:             temp &= ~(GPIO_MODER_MODE0 << moderPos);
	bic	r2, r2, #768	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:29:                 temp |= 0x01UL << moderPos;
	orr	r2, r2, #256	@ temp, temp,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:31:             mcuGpio->MODER = temp;
	str	r2, [r3]	@ temp, MEM[(struct GPIO_TypeDef *)1207960576B].MODER
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:22:             mcuGpio->OSPEEDR |= (0x11 << moderPos);
	ldr	r2, [r3, #8]	@ _3, MEM[(struct GPIO_TypeDef *)1207960576B].OSPEEDR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:22:             mcuGpio->OSPEEDR |= (0x11 << moderPos);
	orr	r2, r2, #4352	@ _4, _3,
	str	r2, [r3, #8]	@ _4, MEM[(struct GPIO_TypeDef *)1207960576B].OSPEEDR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/devices.h:32:     }
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
	.size	Devices<void, Mcu::Stm::Stm32G431>::init(), .-Devices<void, Mcu::Stm::Stm32G431>::init()
	.section	.rodata.main.str1.4,"aMS",%progbits,1
	.align	2
.LC0:
	.ascii	"bla: \000"
	.align	2
.LC1:
	.ascii	" r: \000"
	.align	2
.LC2:
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
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:10:         devs::init();
	bl	Devices<void, Mcu::Stm::Stm32G431>::init()		@
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:34:             mcuGpio->BSRR |= (0x01UL << N);
	ldr	r2, .L153	@ tmp204,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:83:                 ++mValue;
	ldr	r9, .L153+40	@ tmp357,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:34:             mcuGpio->BSRR |= (0x01UL << N);
	ldr	r3, [r2, #24]	@ _3, MEM[(struct GPIO_TypeDef *)1207960576B].BSRR
	ldr	r10, .L153+44	@ tmp356,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	ldr	r8, .L153+48	@ tmp361,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	ldr	r6, .L153+4	@ tmp362,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	ldr	r7, .L153+8	@ tmp363,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:34:             mcuGpio->BSRR |= (0x01UL << N);
	orr	r3, r3, #256	@ _4, _3,
	str	r3, [r2, #24]	@ _4, MEM[(struct GPIO_TypeDef *)1207960576B].BSRR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         if (++c == 1000) {
	ldr	r3, .L153+12	@ tmp387,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:83:                 ++mValue;
	ldr	ip, [r9]	@ _8, mValue
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         if (++c == 1000) {
	ldr	r4, [r3]	@ _18, c
	ldr	r0, [r10]	@ ivtmp.158, r
	mov	r2, r4	@ _18, _18
	add	r3, sp, #12	@ tmp355,,
	mov	r4, ip	@ _8, _8
	add	r5, sp, #10	@ _278,,
	mov	ip, r2	@ _18, _18
.L66:
	adds	r2, r0, #1	@ ivtmp.158, ivtmp.158,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:82:             if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
	mov	lr, #-536813568	@ tmp209,
.L15:
	ldr	r1, [lr, #16]	@ _5, MEM[(struct SysTick_Type *)3758153744B].CTRL
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:82:             if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
	lsls	r1, r1, #15	@, _5,
	mov	r0, r2	@ ivtmp.158, ivtmp.158
	add	r2, r2, #1	@ ivtmp.158, ivtmp.158,
	bpl	.L15		@,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         if (++c == 1000) {
	add	ip, ip, #1	@ _18, _18,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:83:                 ++mValue;
	adds	r4, r4, #1	@ _8, _8,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         if (++c == 1000) {
	cmp	ip, #1000	@ _18,
	str	r0, [r10]	@ ivtmp.158, r
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:83:                 ++mValue;
	str	r4, [r9]	@ _8, mValue
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         if (++c == 1000) {
	beq	.L142		@,
.L16:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:40:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	ldr	r1, .L153	@ tmp348,
	ldr	r2, [r1, #20]	@ _19, MEM[(struct GPIO_TypeDef *)1207960576B].ODR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:40:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	eor	r2, r2, #16	@ tmp349, _19,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:40:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	orr	r2, r2, #1048576	@ _21, tmp349,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/gpio.h:40:             mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
	str	r2, [r1, #24]	@ _21, MEM[(struct GPIO_TypeDef *)1207960576B].BSRR
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:23:     }
	b	.L66		@
.L142:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:18:             c = 0;
	ldr	r1, .L153+12	@ tmp388,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	r4, .L153+16	@ p,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:18:             c = 0;
	movs	r2, #0	@ tmp214,
	str	r2, [r1]	@ tmp214, c
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	movs	r0, #98	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp215,
	b	.L17		@
.L19:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_34 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r0, .L18	@ c,
.L17:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _36, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _36,
	bpl	.L19		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _39, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _39,
	bpl	.L19		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _41,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L143	@ _41,
.L20:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _41,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _41
	beq	.L20	@
.L143:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r0, [r2]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_34 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r0, #0	@ c
	bne	.L17	@
.L18:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:38:             std::to_chars(std::begin(buffer), std::end(buffer), v);
	ldr	r4, [r9]	@ __val, MEM[(const volatile long unsigned int &)&mValue]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	strh	r0, [sp, #8]	@ movhi	@ c, MEM[(struct array *)_232]
	strd	r0, r0, [sp]	@ c, c,
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ __val
	bne	.L22	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r2, #48	@ tmp227,
	strb	r2, [sp]	@ tmp227, MEM[(char *)_232]
.L23:
	mov	r0, sp	@ ivtmp.140,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp360,
	b	.L39		@
.L36:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r5, r0	@ _278, ivtmp.140
	beq	.L35		@,
.L39:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r4, [r0], #1	@ zero_extendqisi2	@ _45, MEM[(const value_type &)_290]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r4, .L35	@ _45,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _46, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _46,
	bpl	.L36		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _87, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _87,
	bpl	.L36		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _89,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L144	@ _89,
.L37:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _89,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _89
	beq	.L37	@
.L144:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r5, r0	@ _278, ivtmp.140
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r4, [r2]	@ _45, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	bne	.L39		@,
.L35:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldr	r4, .L153+20	@ p,
	movs	r0, #32	@ c,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp276,
	b	.L40		@
.L42:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_27 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cbz	r0, .L41	@ c,
.L40:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _29, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _29,
	bpl	.L42		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _90, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _90,
	bpl	.L42		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _92,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L145	@ _92,
.L43:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _92,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _92
	beq	.L43	@
.L145:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r0, [r2]	@ c, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	ldrb	r0, [r4], #1	@ zero_extendqisi2	@ c, MEM[(const char *)p_27 + 4294967295B]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:18:             while(const char c = *p++) {
	cmp	r0, #0	@ c
	bne	.L40	@
.L41:
	ldr	r4, [r10]	@ __val, r
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:37:             std::array<char, Etl::numberOfDigits<std::remove_volatile_t<T>>()> buffer{};
	strh	r0, [sp, #8]	@ movhi	@ c, MEM[(struct array *)_232]
	strd	r0, r0, [sp]	@ c, c,
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:339:       if (__value == 0)
	cmp	r4, #0	@ __val
	bne	.L45	@
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:341: 	  *__first = '0';
	movs	r2, #48	@ tmp288,
	strb	r2, [sp]	@ tmp288, MEM[(char *)_232]
.L46:
	mov	r0, sp	@ ivtmp.115,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp359,
	b	.L62		@
.L59:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r0, r5	@ ivtmp.115, _278
	beq	.L58		@,
.L62:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	ldrb	r4, [r0], #1	@ zero_extendqisi2	@ _96, MEM[(const value_type &)_276]
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:27:                 if (c == typename C::value_type{'\0'}) {
	cbz	r4, .L58	@ _96,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	ldr	r1, [r2, #3712]	@ _97, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _97,
	bpl	.L59		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _138, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _138,
	bpl	.L59		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _140,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cbnz	r1, .L146	@ _140,
.L60:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	.thumb
	.syntax unified
	ldr	r1, [r2]	@ _140,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _140
	beq	.L60	@
.L146:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	cmp	r0, r5	@ ivtmp.115, _278
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	strb	r4, [r2]	@ _96, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/output.h:26:             for(const typename C::value_type& c : a) {
	bne	.L62		@,
.L58:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	mov	r2, #-536870912	@ tmp337,
	ldr	r1, [r2, #3712]	@ _22, MEM[(struct ITM_Type *)3758096384B].TCR
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r0, r1, #31	@, _22,
	bpl	.L63		@,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2062:       ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
	ldr	r1, [r2, #3584]	@ _141, MEM[(struct ITM_Type *)3758096384B].TER
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2061:   if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
	lsls	r1, r1, #31	@, _141,
	bmi	.L141		@,
.L63:
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         if (++c == 1000) {
	ldr	r2, .L153+12	@ tmp391,
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/clock.h:83:                 ++mValue;
	ldr	r4, [r9]	@ _8, mValue
@ /home/lmeier/Projekte/wmucpp/boards/nucleo_g431/test00.cc:17:         if (++c == 1000) {
	ldr	ip, [r2]	@ _18, c
	movs	r0, #0	@ ivtmp.158,
	b	.L16		@
.L64:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2066:       __NOP();
	.syntax unified
@ 2066 "../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h" 1
	nop
@ 0 "" 2
	.thumb
	.syntax unified
.L141:
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	ldr	r1, [r2]	@ _143,
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2064:     while (ITM->PORT[0U].u32 == 0UL)
	cmp	r1, #0	@ _143
	beq	.L64	@
@ ../../../STM32CubeG4/Drivers/CMSIS/Core/Include/core_cm4.h:2068:     ITM->PORT[0U].u8 = (uint8_t)ch;
	mov	r2, #-536870912	@ tmp342,
	movs	r1, #10	@ tmp343,
	strb	r1, [r2]	@ tmp343, MEM[(struct ITM_Type *)3758096384B].PORT[0].u8
	b	.L63		@
.L22:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ __val,
	bls	.L24		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ __val,
	bls	.L147		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	cmp	r4, #1000	@ __val,
	bcc	.L67		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movw	r2, #9999	@ tmp234,
	cmp	r4, r2	@ __val, tmp234
	bls	.L148		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	ldr	r2, .L153+24	@ tmp239,
	cmp	r4, r2	@ __val, tmp239
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	it	ls
	movls	fp, #5	@ _62,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	bls	.L27		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	ldr	r2, .L153+28	@ tmp235,
	cmp	r4, r2	@ __val, tmp235
	bls	.L149		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	ldr	r2, .L153+32	@ tmp236,
	cmp	r4, r2	@ __val, tmp236
	bcc	.L69		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	cmp	r4, r6	@ __val, tmp362
	bls	.L70		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, r7	@ __val, tmp363
	bls	.L71		@,
	mov	fp, #5	@ __n,
.L31:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	add	fp, fp, #5	@ _62, __n,
.L27:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L153+36	@,
	mov	r0, r3	@, tmp355
	movs	r2, #201	@,
	bl	memcpy		@
	sub	r1, fp, #2	@ tmp246, _62,
	mov	r3, r0	@ tmp355,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	mov	lr, #100	@ tmp251,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	movw	ip, #9999	@ tmp260,
	add	r1, r1, sp	@ ivtmp.145,
.L33:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r0, r2, r8, r4	@ tmp364, tmp248, tmp361, __val
	lsrs	r2, r2, #5	@ tmp247, tmp248,
	mls	fp, lr, r2, r4	@ tmp252, tmp251, tmp247, __val
	mov	r0, r4	@ __val, __val
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	cmp	r0, ip	@ __val, tmp260
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r4, [r3, fp, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.99_272 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r4, [r1], #-2	@ unaligned	@ vect__78.100, MEM <vector(2) char> [(char *)vectp.102_269]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:97: 	  __val /= 100;
	mov	r4, r2	@ __val, tmp247
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	bhi	.L33		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:102:       if (__val >= 10)
	cmp	r0, #1000	@ __val,
	bcc	.L24		@,
.L26:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r2, sp, #216	@ tmp389,,
	add	r4, r2, r4, lsl #1	@ tmp261, tmp389, __val,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r1, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_225][_82], MEM[(char[201] *)_225][_82]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r2, [r4, #-204]	@ zero_extendqisi2	@ _84, MEM[(char[201] *)_225][__num_81]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	strb	r1, [sp, #1]	@ MEM[(char[201] *)_225][_82], MEM[(char *)_232]
.L34:
	strb	r2, [sp]	@ _84, MEM[(char *)_232]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:235:       return __res;
	b	.L23		@
.L154:
	.align	2
.L153:
	.word	1207960576
	.word	99999999
	.word	999999999
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::c
	.word	.LC0+1
	.word	.LC1+1
	.word	99999
	.word	999999
	.word	10000000
	.word	.LC2
	.word	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue
	.word	GFSM<Devices<void, Mcu::Stm::Stm32G431> >::r
	.word	1374389535
.L45:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, #9	@ __val,
	bls	.L47		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	cmp	r4, #99	@ __val,
	bls	.L150		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	cmp	r4, #1000	@ __val,
	bcc	.L72		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	movw	r2, #9999	@ tmp295,
	cmp	r4, r2	@ __val, tmp295
	bls	.L151		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	ldr	r2, .L155	@ tmp300,
	cmp	r4, r2	@ __val, tmp300
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	it	ls
	movls	fp, #5	@ _44,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	bls	.L50		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	ldr	r2, .L155+4	@ tmp296,
	cmp	r4, r2	@ __val, tmp296
	bls	.L152		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	ldr	r2, .L155+8	@ tmp297,
	cmp	r4, r2	@ __val, tmp297
	bcc	.L74		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	cmp	r4, r6	@ __val, tmp362
	bls	.L75		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	cmp	r4, r7	@ __val, tmp363
	bls	.L76		@,
	mov	fp, #5	@ __n,
.L54:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:68: 	  if (__value < __b2) return __n + 1;
	add	fp, fp, #5	@ _44, __n,
.L50:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	ldr	r1, .L155+12	@,
	mov	r0, r3	@, tmp355
	movs	r2, #201	@,
	bl	memcpy		@
	sub	r1, fp, #2	@ tmp307, _44,
	mov	r3, r0	@ tmp355,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	mov	lr, #100	@ tmp312,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	movw	ip, #9999	@ tmp321,
	add	r1, r1, sp	@ ivtmp.121,
.L56:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:96: 	  auto const __num = (__val % 100) * 2;
	umull	r0, r2, r8, r4	@ tmp365, tmp309, tmp361, __val
	lsrs	r2, r2, #5	@ tmp308, tmp309,
	mls	fp, lr, r2, r4	@ tmp313, tmp312, tmp308, __val
	mov	r0, r4	@ __val, __val
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	cmp	r0, ip	@ __val, tmp321
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	ldrh	r4, [r3, fp, lsl #1]	@, MEM <const vector(2) char> [(char *)vectp.104_267 + 4294967295B]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:99: 	  __first[__pos - 1] = __digits[__num];
	strh	r4, [r1], #-2	@ unaligned	@ vect__129.105, MEM <vector(2) char> [(char *)vectp.107_264]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:97: 	  __val /= 100;
	mov	r4, r2	@ __val, tmp308
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:94:       while (__val >= 100)
	bhi	.L56		@,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:102:       if (__val >= 10)
	cmp	r0, #1000	@ __val,
	bcc	.L47		@,
.L49:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	add	r2, sp, #216	@ tmp390,,
	add	r4, r2, r4, lsl #1	@ tmp322, tmp390, __val,
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	ldrb	r1, [r4, #-203]	@ zero_extendqisi2	@ MEM[(char[201] *)_225][_133], MEM[(char[201] *)_225][_133]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:106: 	  __first[0] = __digits[__num];
	ldrb	r2, [r4, #-204]	@ zero_extendqisi2	@ _135, MEM[(char[201] *)_225][__num_132]
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:105: 	  __first[1] = __digits[__num + 1];
	strb	r1, [sp, #1]	@ MEM[(char[201] *)_225][_133], MEM[(char *)_232]
.L57:
	strb	r2, [sp]	@ _135, MEM[(char *)_232]
@ /usr/arm-none-eabi/include/c++/13.1.0/charconv:235:       return __res;
	b	.L46		@
.L47:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:109: 	__first[0] = '0' + __val;
	add	r2, r4, #48	@ tmp327, __val,
	uxtb	r2, r2	@ _135, tmp327
	b	.L57		@
.L24:
	add	r2, r4, #48	@ tmp266, __val,
	uxtb	r2, r2	@ _84, tmp266
	b	.L34		@
.L74:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	mov	fp, #7	@ _44,
	b	.L50		@
.L69:
	mov	fp, #7	@ _62,
	b	.L27		@
.L75:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	mov	fp, #8	@ _44,
	b	.L50		@
.L70:
	mov	fp, #8	@ _62,
	b	.L27		@
.L76:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:67: 	  if (__value < (unsigned)__base) return __n;
	mov	fp, #9	@ _44,
	b	.L50		@
.L71:
	mov	fp, #9	@ _62,
	b	.L27		@
.L147:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	mov	r0, r3	@, tmp355
	ldr	r1, .L155+12	@,
	movs	r2, #201	@,
	bl	memcpy		@
	mov	r3, r0	@ tmp355,
	b	.L26		@
.L151:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	mov	fp, #4	@ _44,
	b	.L50		@
.L72:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	mov	fp, #3	@ _44,
	b	.L50		@
.L150:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:87:       constexpr char __digits[201] =
	mov	r0, r3	@, tmp355
	ldr	r1, .L155+12	@,
	movs	r2, #201	@,
	bl	memcpy		@
	mov	r3, r0	@ tmp355,
	b	.L49		@
.L148:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:70: 	  if (__value < __b4) return __n + 3;
	mov	fp, #4	@ _62,
	b	.L27		@
.L67:
@ /usr/arm-none-eabi/include/c++/13.1.0/bits/charconv.h:69: 	  if (__value < __b3) return __n + 2;
	mov	fp, #3	@ _62,
	b	.L27		@
.L152:
	mov	fp, #1	@ __n,
	b	.L54		@
.L149:
	mov	fp, #1	@ __n,
	b	.L31		@
.L156:
	.align	2
.L155:
	.word	99999
	.word	999999
	.word	10000000
	.word	.LC2
	.size	main, .-main
	.weak	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue
	.section	.bss._ZN3Mcu3Stm11SystemTimerINS0_5ClockINS0_11ClockConfigIXtlN5Units9frequencyItSt5ratioILx1ELx1000000EEEELm170EEEXtlNS5_ImS6_ILx1ELx1EEEELm1000EEENS0_3HSIEEENS0_9Stm32G431EEESt17integral_constantIbLb0EESD_E6mValueE,"awG",%nobits,Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue,comdat
	.align	2
	.type	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue, %object
	.size	Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue, 4
Mcu::Stm::SystemTimer<Mcu::Stm::Clock<Mcu::Stm::ClockConfig<Units::frequency<unsigned short, std::ratio<1ll, 1000000ll> >{170ul}, Units::frequency<unsigned long, std::ratio<1ll, 1ll> >{1000ul}, Mcu::Stm::HSI>, Mcu::Stm::Stm32G431>, std::integral_constant<bool, false>, Mcu::Stm::Stm32G431>::mValue:
	.space	4
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
