/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);	/* end of "RAM" Ram type memory */

/* _Min_Heap_Size = 0x200;	/* required amount of heap  */
 _Min_Heap_Size = 0x000;	/* required amount of heap  */

/* _Min_Stack_Size = 0x400; */
_Min_Stack_Size = 0x100;	/* required amount of stack */

/* Memories definition */
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 8K
  FLASH    (rx)   : ORIGIN = 0x8000000,   LENGTH = 32K
}

_flash_start = ORIGIN(FLASH);

/* Sections */
SECTIONS
{
  /* The startup code into "FLASH" Rom type memory */
  .isr_vector :
  {
    . = ALIGN(4);
	KEEP(*(.isr_vector)) /* Startup code */
	. = ALIGN(4);
  } >FLASH

  /* The program code and other data into "FLASH" Rom type memory */
  .text :
  {
    . = ALIGN(4);
	*(.text)           /* .text sections (code) */
	*(.text*)          /* .text* sections (code) */
	*(.glue_7)         /* glue arm to thumb code */
	*(.glue_7t)        /* glue thumb to arm code */
	*(.eh_frame)

    KEEP (*(.init))
	KEEP (*(.fini))

    . = ALIGN(4);
	_etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data into "FLASH" Rom type memory */
  .rodata :
  {
    . = ALIGN(4);
	*(.rodata)         /* .rodata sections (constants, strings, etc.) */
	*(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
	. = ALIGN(4);
  } >FLASH

  .ARM.extab   : {
    . = ALIGN(4);
	*(.ARM.extab* .gnu.linkonce.armextab.*)
	. = ALIGN(4);
  } >FLASH

  .ARM : {
    . = ALIGN(4);
	__exidx_start = .;
	*(.ARM.exidx*)
	__exidx_end = .;
	. = ALIGN(4);
  } >FLASH

  .preinit_array     :
  {
    . = ALIGN(4);
	PROVIDE_HIDDEN (__preinit_array_start = .);
	KEEP (*(.preinit_array*))
	PROVIDE_HIDDEN (__preinit_array_end = .);
	. = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
	PROVIDE_HIDDEN (__init_array_start = .);
	KEEP (*(SORT(.init_array.*)))
	KEEP (*(.init_array*))
	PROVIDE_HIDDEN (__init_array_end = .);
	. = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
	PROVIDE_HIDDEN (__fini_array_start = .);
	KEEP (*(SORT(.fini_array.*)))
	KEEP (*(.fini_array*))
	PROVIDE_HIDDEN (__fini_array_end = .);
	. = ALIGN(4);
  } >FLASH


.eeprom ALIGN(2048) :
{
  _eeprom_start = .;
  *(.eeprom)
  *(.eeprom*)
  . = ALIGN(2048);
  _eeprom_end = .;
} >FLASH

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    . = ALIGN(4);
	_sdata = .;        /* create a global symbol at data start */
	*(.data)           /* .data sections */
	*(.data*)          /* .data* sections */

    . = ALIGN(4);
	_edata = .;        /* define a global symbol at data end */

  } >RAM AT> FLASH

  /* Uninitialized data section into "RAM" Ram type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
	_sbss = .;         /* define a global symbol at bss start */
	__bss_start__ = _sbss;
	*(.bss)
	*(.bss*)
	*(COMMON)

    . = ALIGN(4);
	_ebss = .;         /* define a global symbol at bss end */
	__bss_end__ = _ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough "RAM" Ram  type memory left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
	PROVIDE ( end = . );
	PROVIDE ( _end = . );
	. = . + _Min_Heap_Size;
	. = . + _Min_Stack_Size;
	. = ALIGN(8);
  } >RAM

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
	libm.a ( * )
	libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
