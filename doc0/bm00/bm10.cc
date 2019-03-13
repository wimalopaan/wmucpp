#include <cstdint>
#include <limits>
#ifndef __AVR__
#include<type_traits>
#include <limits>
#include <cmath>
#include <chrono>
#include <ratio>
#endif
#ifdef A4963
#include "../../src/CustomDataTypes/Volt.h"
#include "../../src/CustomDataTypes/Hertz.h"
#include "../../src/CustomDataTypes/Percentage.h"
#endif


using mem_width = uint8_t;
using ptr_t = uintptr_t;

#define NoConstructors(x) x() = delete; x(const x&) = delete; x(x&&) = delete
namespace mega4809 {
namespace utils {


	template<bool condition, typename T1, typename T2>
	struct conditional;

	template<typename T, T X>
	struct integralConstant;

	template<typename T, typename O>
	struct isEqual;

	template<auto x>
	struct	minRequired;

	template<auto x>
	struct minRequiredUnsigned;
	

	template<typename T>
	struct isUnsigned;

	namespace details {
		template<bool condition, typename T1, typename T2>
		struct conditional
		{
			using type = T2;
		};

		template<typename T1, typename T2>
		struct conditional<true, T1, T2>
		{
			using type = T1;
		};


		template<typename T, typename O>
		struct isEqual
		{
			static constexpr bool value = false;
		};

		template<typename T>
		struct isEqual<T, T>
		{
			static constexpr bool value = true;
		};


//		template<long long x>
//		struct	minRequired {
//			using min = 							
//				typename conditional<x <= SCHAR_MAX && x >= SCHAR_MIN , char,
//					typename conditional<x <= SHRT_MAX && x >= SHRT_MIN, short,
//						typename conditional<x <= INT_MAX && x >= INT_MIN, int,
//							typename conditional<x <= LONG_MAX && x >= LONG_MIN, long, long long
//							>::type
//						>::type
//					>::type
//				>::type;
//		};

//		template<unsigned long long x>
//		struct	minRequiredUnsigned {
//			using min = 
//				typename conditional<x <= UCHAR_MAX, unsigned char,
//					typename conditional<x <= USHRT_MAX, unsigned short,
//						typename conditional<x <= UINT_MAX, unsigned int,
//							typename conditional<x <= ULONG_MAX, unsigned long, unsigned long long
//							>::type
//						>::type
//					>::type
//				>::type;
//		};

		template<typename T>
		struct isUnsigned {
			static constexpr bool value = false;
		};

		template<>
		struct isUnsigned<unsigned char>
		{
			static constexpr bool value = true;
		};

		template<>
		struct isUnsigned<unsigned short>
		{
			static constexpr bool value = true;
		};

		template<>
		struct isUnsigned<unsigned int>
		{
			static constexpr bool value = true;
		};

		template<>
		struct isUnsigned<unsigned long>
		{
			static constexpr bool value = true;
		};

		template<>
		struct isUnsigned<unsigned long long>
		{
			static constexpr bool value = true;
		};

		template<typename integral, integral val>
		struct isPositive {
			static constexpr bool value = val > 0 ? true : false;
		};
	}

	template<bool condition, typename T1, typename T2>
	struct conditional {
		using type = typename details::conditional<condition, T1, T2>::type;
	};

	template<typename T, T X>
	struct integralConstant {
		static constexpr T value = X;
		using value_type = T;
		using type = integralConstant;
	};

	template<typename T, typename O>
	struct isEqual {
		static constexpr bool value = details::isEqual<T, O>::value;
	};

	//#define minReqS(value) typename utils::minRequired<decltype(value),value>::type
//	template<auto x>
//	struct minRequired {
//		using type = typename conditional<x <= INT_MAX,typename details::minRequired<x>::min,void>::type;
//		static_assert(!isEqual<type, void>::value, "signed value is too big to fit to an unsigned type");
//	};

	//#define minReqU(value) typename utils::minRequiredUnsigned<decltype(value),value>::type
//	template<auto x>
//	struct minRequiredUnsigned {
//		static_assert(x >= 0, " tried to use unsigned type for negative value");
//		using type = typename details::minRequiredUnsigned<x>::min;
//	};

	template<typename T>
	struct isUnsigned {
		static constexpr bool value = details::isUnsigned<T>::value;
	};


	template<typename ...T>
	struct list {
		static constexpr auto size = sizeof...(T)+1;
	};
	 //to test if this change will cause bugs or other problems
	template<typename first, typename ...T>
	struct list<first, T...> {
		static constexpr auto size = sizeof...(T)+1;
	};


	template<typename T>
	struct front { using type = T; };

	template<template<typename,typename...> typename list, typename F, typename ...T>
	struct front<list<F,T...>> {
		using type = F;
	};

	template<typename Push, typename L>
	struct push_front {};

	template<typename Push,template<typename...> typename List, typename... T>
	struct push_front<Push,List<T...>>
	{
		using type = List<Push,T...>;
	};

	template<typename T>
	struct pop_front { using type = list<T>; };

	template<typename F, typename ...T>
	struct pop_front<list<F,T...>> {
		using type = list<F,T...>;
	};

	template<unsigned long long F, typename T, typename... P>
	struct getType;

	template<typename T, typename... Ts>
	struct getType<0, T, Ts...>;

	template<unsigned long long F, template<typename, typename...> typename List, typename T, typename... P>
	struct getType<F, List<T, P...>>;


	template<unsigned long long F, typename T, typename... P>
	struct getType
	{
		static_assert(F >= 0, "no negative values allowed");
		static_assert(F < (sizeof...(P)+1), "index out of bounds");
		using type = typename getType<F - 1, P...>::type;
	};

	template<unsigned long long F, template<typename,typename...> typename List,typename T, typename... P>
	struct getType<F,List<T,P...>>
	{
		static_assert(F >= 0, "no negative values allowed");
		using type = 
			typename getType<F, T, P...>::type;
	};

	//necessary for tl variant (ambiguous instantiation)
	template<template<typename, typename...> typename List, typename T, typename... P>
	struct getType<0, List<T, P...>>
	{
		using type = T;
	};

	template<typename T, typename... Ts>
	struct getType<0,T,Ts...>
	{
		using type = T;
	};

	template<typename first,typename ...T>
	[[nodiscard]] constexpr bool sameTypes() noexcept {
		if constexpr(sizeof...(T) == 0) 
			return true;
		else {
			if constexpr (!isEqual<first, typename front<list<T...>>::type>::value)
				return false;
			else
				return sameTypes<T...>();
		}
	
	}
	
			
	template<typename T, auto val>
	struct Pair {
		static inline constexpr auto value = val;
		using type = T;
	};
	
	template<typename searched, typename first, typename ... pack>
	struct contains {
		static inline constexpr bool value = isEqual<searched,first>::value ? true : contains<searched,pack...>::value;
	};

	//last in recursion
	template<typename searched, typename first>
	struct contains<searched,first> {
		static inline constexpr bool value = isEqual<searched,first>::value;
	};
}

/*
* Pin.h
*
* Created: 27.01.2019 19:29:16
*  Author: Keven
*/

struct Pin {
	mem_width pinValue;
	
	constexpr explicit Pin(mem_width number) : pinValue( 1 << number ) {}
	constexpr Pin(const Pin& other) : pinValue(other.pinValue) {}
	constexpr Pin(Pin&& other) = delete;
	constexpr void operator=(mem_width number) { pinValue = 1 << number; }
	constexpr void operator=(const Pin& other) { pinValue = other.pinValue; }
	constexpr void operator=(Pin&& other) = delete;
	constexpr explicit operator mem_width() const{
		return pinValue;
	}
};

namespace reg {
	
	namespace accessmode {
		struct RW {}; struct ReadOnly {};
	}
	
	namespace specialization {
		//data and flag equal
		struct Data{}; struct Control {}; struct Toggle{};
	}
	
	template<typename Access = accessmode::RW, typename Specialization = specialization::Data, typename Bits = void, typename size = mem_width>
	class Register;
	
	template<typename size>
	class Register<accessmode::RW, specialization::Data, void, size>{

		volatile size reg;
		
		public:
		NoConstructors(Register);
		using regSize = size;
		template<typename... ARGS>
		inline void on(const ARGS... bits) volatile {
			if constexpr(sizeof...(ARGS) == 0){
				reg = static_cast<size>(-1);
			} else
			reg |= (static_cast<size>(bits) | ...);
		}
		
		template<typename... ARGS>
		inline void off(const ARGS... bits) volatile {
			if constexpr(sizeof...(ARGS) == 0){
				reg = static_cast<size>(0);
			} else
			reg &= ~(static_cast<size>(bits) | ...);
		}

		template<typename... ARGS>
		inline void invert(const ARGS... bits) volatile {
			if constexpr(sizeof...(ARGS) == 0){
				reg ^= static_cast<size>(-1);
			} else
			reg ^= (static_cast<size>(bits) | ...);
		}

		//SFINAE
		[[nodiscard]] bool areSet() volatile;

		template<typename... ARGS>
		[[nodiscard]] inline bool areSet(const ARGS...bits) const volatile {
			return ((static_cast<size>(bits) | ...) & reg) == (static_cast<size>(bits) | ...);
		}
		
		[[nodiscard]] volatile size& raw() volatile {
			return reg;
		}

		[[nodiscard]] volatile size raw() const volatile {
			return reg;
		}
		/*
		function to cast the structs to Register
		*/
		[[nodiscard]] static inline volatile Register& getRegister(volatile size& reg) {
			return reinterpret_cast<volatile Register&>(reg);
		}
	}__attribute__((packed));

	template<typename size>
	class Register<accessmode::ReadOnly,specialization::Data,void,size> {
		const volatile size reg;
		
		public:
		NoConstructors(Register);
		using regSize = size;
		[[nodiscard]] volatile mem_width raw() const volatile {
			return reg;
		}
		
		//SFINAE
		[[nodiscard]] bool areSet() volatile;

		template<typename... ARGS>
		[[nodiscard]] inline bool areSet(const ARGS...bits) const volatile {
			return ((static_cast<size>(bits) | ...) & reg) == (static_cast<size>(bits) | ...);
		}
		/*
		function to cast the structs to Register
		*/
		[[nodiscard]] static inline volatile Register& getRegister(volatile size& reg) {
			return reinterpret_cast<volatile Register&>(reg);
		}
	}__attribute__((packed));
	
	template<typename Bits, typename size>
	class Register<accessmode::RW,specialization::Control,Bits,size> {
		static_assert(! utils::isEqual<Bits,void>::value, "enum type expected");
		volatile size reg;
		
		public:
		NoConstructors(Register);
		using regSize = size;
		using special_bit = Bits;
		
		template<typename... ARGS>
		inline void on(const ARGS... bits) volatile {
			static_assert(utils::isEqual<special_bit,typename utils::front<ARGS...>::type>::value && utils::sameTypes<ARGS...>(),"only the special bits are allowed");
			if constexpr(sizeof...(ARGS) == 0){
				reg = static_cast<size>(-1);
			} else
			reg |= (static_cast<size>(bits) | ...);
		}
		
		template<typename... ARGS>
		inline void off(const ARGS... bits) volatile {
			static_assert(utils::isEqual<special_bit,typename utils::front<ARGS...>::type>::value && utils::sameTypes<ARGS...>(),"only the special bits are allowed");
			if constexpr(sizeof...(ARGS) == 0){
				reg = static_cast<size>(0);
			} else
			reg &= ~(static_cast<size>(bits) | ...);
		}

		template<typename... ARGS>
		inline void invert(const ARGS... bits) volatile {
			static_assert(utils::isEqual<special_bit,typename utils::front<ARGS...>::type>::value && utils::sameTypes<ARGS...>(),"only the special bits are allowed");
			if constexpr(sizeof...(ARGS) == 0){
				reg ^= static_cast<size>(-1);
			} else
			reg ^= (static_cast<size>(bits) | ...);
		}

		//SFINAE
		[[nodiscard]] bool areSet() volatile;

		template<typename... ARGS>
		[[nodiscard]] inline bool areSet(const ARGS...bits) const volatile {
			static_assert(utils::isEqual<special_bit,typename utils::front<ARGS...>::type>::value && utils::sameTypes<ARGS...>(),"only the special bits are allowed");
			return ((static_cast<size>(bits) | ...) & reg) == (static_cast<size>(bits) | ...);
		}
		
		[[nodiscard]] volatile size& raw() volatile {
			return reg;
		}

		[[nodiscard]] volatile size raw() const volatile {
			return reg;
		}
		/*
		function to cast the structs to Register
		*/
		[[nodiscard]] static inline volatile Register& getRegister(volatile size& reg) {
			return reinterpret_cast<volatile Register&>(reg);
		}
	}__attribute__((packed));
	
	template<typename size>
	class Register<accessmode::RW,specialization::Toggle,void,size> {

		volatile size reg;
		
		public:
		NoConstructors(Register);
		using regSize = size;
		template<typename... ARGS>
		inline void invert(const ARGS... bits) volatile {
			if constexpr(sizeof...(ARGS) == 0){
				reg = static_cast<size>(-1);
			} else
			reg = (static_cast<size>(bits) | ...);
		}

		//SFINAE
		[[nodiscard]] bool areSet() volatile;

		template<typename... ARGS>
		[[nodiscard]] inline bool areSet(const ARGS...bits) const volatile {
			return ((static_cast<size>(bits) | ...) & reg) == (static_cast<size>(bits) | ...);
		}
		
		[[nodiscard]] volatile size& raw() volatile {
			return reg;
		}

		[[nodiscard]] volatile size raw() const volatile {
			return reg;
		}
		/*
		function to cast the structs to Register
		*/
		[[nodiscard]] static inline volatile Register& getRegister(volatile size& reg) {
			return reinterpret_cast<volatile Register&>(reg);
		}
	}__attribute__((packed));
}
namespace port {
	
	namespace {
		
		template<typename P, mem_width number>
		class PortPin;
		
		template<typename P, typename regs>
		class Port {
			
			template<typename p, mem_width number>
			friend class PortPin;
			
			using _port = P;
            static inline auto& port = P::value;
			public:
			using pins = typename P::pins;
			using portPins = typename P::template portPins<Port>;
			using registers = regs;
			
			NoConstructors(Port);
			
			template<typename reg>
			static inline auto& get(){
				using reg_t = typename reg::type;
				return reg_t::getRegister(*((typename reg_t::regSize*)&P::value + reg::value));
			}

			[[nodiscard]] static inline auto& getDir() {
				return reg::Register<>::getRegister(P::value.DIR);
			};

			[[nodiscard]] static inline auto& getInput() {
				return reg::Register<reg::accessmode::ReadOnly>::getRegister(P::value.IN);
			};

			[[nodiscard]] static inline auto& getOutput() {
				return reg::Register<>::getRegister(P::value.OUT);
			};
			
			template<typename... PINS>
			static inline void setInput(PINS... args) {
				if constexpr(sizeof...(PINS) == 0){
					getDir().raw() = static_cast<mem_width>(0x00);
					} else {
					getDir().raw() &= ~(static_cast<mem_width>(args) | ...);
				}
			}
			
			template<typename... PINS>
			static inline void setOutput(PINS... args) {
				if constexpr(sizeof...(PINS) == 0){
					getDir().raw() = static_cast<mem_width>(-1);
					} else {
					getDir().raw() |= (static_cast<mem_width>(args) | ...);
				}
			}
		};

		template<typename P, mem_width number>
		class PortPin{
			
			#define port P::_port::value
			
			public:
			
			NoConstructors(PortPin);
			
			static inline constexpr mem_width pinValue = 1 << number;
			
			static inline void off(){
				port.OUT &= ~pinValue;
			}
			
			static inline void on(){
				port.OUT |= pinValue;
			}
			
			static inline void invert(){
				port.OUT ^= pinValue;
			}
			
			static inline void setOutput(){ //no option to write as template -> parse error in template argument list, bug?
				port.DIR |= pinValue;
			}
			
			static inline void setInput() {
				port.DIR &= ~pinValue;
			}
			
			#undef port
		};
	}
}
using register8_t = uint8_t;

typedef struct PORT_struct
{
    register8_t DIR;  /* Data Direction */
    register8_t DIRSET;  /* Data Direction Set */
    register8_t DIRCLR;  /* Data Direction Clear */
    register8_t DIRTGL;  /* Data Direction Toggle */
    register8_t OUT;  /* Output Value */
    register8_t OUTSET;  /* Output Value Set */
    register8_t OUTCLR;  /* Output Value Clear */
    register8_t OUTTGL;  /* Output Value Toggle */
    register8_t IN;  /* Input Value */
    register8_t INTFLAGS;  /* Interrupt Flags */
    register8_t PORTCTRL;  /* Port Control */
    register8_t reserved_0x0B;
    register8_t reserved_0x0C;
    register8_t reserved_0x0D;
    register8_t reserved_0x0E;
    register8_t reserved_0x0F;
    register8_t PIN0CTRL;  /* Pin 0 Control */
    register8_t PIN1CTRL;  /* Pin 1 Control */
    register8_t PIN2CTRL;  /* Pin 2 Control */
    register8_t PIN3CTRL;  /* Pin 3 Control */
    register8_t PIN4CTRL;  /* Pin 4 Control */
    register8_t PIN5CTRL;  /* Pin 5 Control */
    register8_t PIN6CTRL;  /* Pin 6 Control */
    register8_t PIN7CTRL;  /* Pin 7 Control */
    register8_t reserved_0x18;
    register8_t reserved_0x19;
    register8_t reserved_0x1A;
    register8_t reserved_0x1B;
    register8_t reserved_0x1C;
    register8_t reserved_0x1D;
    register8_t reserved_0x1E;
    register8_t reserved_0x1F;
} PORT_t;

#define PORTA                (*(PORT_t *) 0x0400) /* I/O Ports */
#define PORTB                (*(PORT_t *) 0x0420) /* I/O Ports */
#define PORTC                (*(PORT_t *) 0x0440) /* I/O Ports */
#define PORTD                (*(PORT_t *) 0x0460) /* I/O Ports */
#define PORTE                (*(PORT_t *) 0x0480) /* I/O Ports */
#define PORTF                (*(PORT_t *) 0x04A0) /* I/O Ports */

	
		
		struct portpairs {
			
			enum intFlagMasks : mem_width { pinInterrupt = 0xff};
			enum portCtrlMasks : mem_width { slewRateEnable = 0x1};
			enum pinMasks : mem_width {
				InvertedIOEnable = 0,
				InputSenseConfiguration = 0,
				ISCFallingEdge = 0,
				ISCRisingEdge = 0,
				ISCBothEdges = 0,
				ISCInputDisable = 0,
				ISCInterruptDisable = 0,
				Pullupenable = 0
			};
			
			using dir = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Data>,0x00>;
			using dirset = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Data>,0x01>;
			using dirclear = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Data>,0x02> ;
			using dirtoggle = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Toggle>,0x03>;
			using out = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Data>,0x04>;
			using outset = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Data>,0x05>;
			using outclear = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Data>,0x06>;
			using outtoggle = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Toggle>,0x07>;
			//Input port declared as RW ?
			using in = utils::Pair<reg::Register<reg::accessmode::ReadOnly,reg::specialization::Data>,0x08>;
			using intflags = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Control,intFlagMasks>,0x09> ;
			using portctrl = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Control, portCtrlMasks>,0x0A>;
			
			template<auto num>
			requires(num < 8 && num >= 0)
			using pinctrl = utils::Pair<reg::Register<reg::accessmode::RW,reg::specialization::Control, pinMasks>,0x10+num>;
		};
		
		//legacy
		enum PortRegisters : mem_width {
			DIR = 0,  /* Data Direction */
			DIRSET = 1,  /* Data Direction Set */
			DIRCLR = 2,  /* Data Direction Clear */
			DIRTGL = 3,  /* Data Direction Toggle */
			OUT = 4,  /* Output Value */
			OUTSET = 5,  /* Output Value Set */
			OUTCLR = 6,  /* Output Value Clear */
			OUTTGL = 7,  /* Output Value Toggle */
			IN = 8,  /* Input Value */
			INTFLAGS = 9,  /* Interrupt Flags */
			PORTCTRL = 10,  /* Port Control */
			PIN0CTRL = 16,  /* Pin 0 Control */
			PIN1CTRL = 17,  /* Pin 1 Control */
			PIN2CTRL = 18,  /* Pin 2 Control */
			PIN3CTRL = 19,  /* Pin 3 Control */
			PIN4CTRL = 20,  /* Pin 4 Control */
			PIN5CTRL = 21,  /* Pin 5 Control */
			PIN6CTRL = 22,  /* Pin 6 Control */
			PIN7CTRL = 23  /* Pin 7 Control */
		};
		
		#define pp(number) using pin ## number = port::PortPin<P,number>
		
		struct ports{
			NoConstructors(ports);
			struct A{
				NoConstructors(A);
				static inline auto& value = PORTA;
				struct pins {
					static inline constexpr Pin pin0{0}, pin1{1}, pin2{2}, pin3{3}, pin4{4}, pin5{5},pin6{6}, pin7{7};
				};
				template<typename P>
				struct portPins{
					pp(0); pp(1); pp(2); pp(3); pp(4); pp(5); pp(6); pp(7);
				};
			};
			
			struct C{
				NoConstructors(C);
				static inline auto& value = PORTC;
				struct pins {
					static inline constexpr Pin pin0{0}, pin1{1}, pin2{2}, pin3{3};
				};
				template<typename P>
				struct portPins{
					pp(0); pp(1); pp(2); pp(3);
				};
			};
			
			struct D{
				NoConstructors(D);
				static inline auto& value = PORTD;
				struct pins {
					static inline constexpr Pin pin0{0}, pin1{1}, pin2{2}, pin3{3}, pin4{4}, pin5{5},pin6{6}, pin7{7};
				};
				template<typename P>
				struct portPins{
					pp(0); pp(1); pp(2); pp(3); pp(4); pp(5); pp(6); pp(7);
				};
			};
			
			struct F{
				NoConstructors(F);
				static inline auto& value = PORTF;
				struct pins {
					static inline constexpr Pin pin0{0}, pin1{1}, pin2{2}, pin3{3}, pin4{4}, pin5{5},pin6{6};
				};
				template<typename P>
				struct portPins{
					pp(0); pp(1); pp(2); pp(3); pp(4); pp(5); pp(6);
				};
			};
			
			#undef pp
		};
		
		template<typename P>
		using _port = port::Port<P,portpairs>;

	struct Atmega4809 {
		
		struct Ports {

			Ports() = delete;
			Ports(const Ports&) = delete;
			Ports(Ports&&) = delete;
			
			using porta = _port<ports::A>;

			using portc = _port<ports::C>;
			using portd = _port<ports::D>;

			using portf = _port<ports::F>;
		};
		

	};
}

using namespace mega4809;
using portf = Atmega4809::Ports::portf;
int main(){
    portf::get<portf::registers::out>().on();
    //PORTF.OUT = 0xff;
}
