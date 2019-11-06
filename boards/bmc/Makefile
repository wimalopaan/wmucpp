# Fuse
# sudo avrdude -p atmega328pb -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0xff:m -U hfuse:w:0xd0:m -U efuse:w:0xf7:m

# Bootloader
# sudo avrdude -p atmega328pb -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U optiboot_flash_atmega328pb_UART1_9600_16000000L.hex

# Firmware
# sudo avrdude -p atmega328pb -c arduino -b 9600 -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P /dev/ttyUSB0 -U flash:w:test00.elf


MCU = atmega328pb
F_OSC = 16000000

#subdirs += robbenavy540
subdirs += small01

targets += test00.elf
targets += test10.elf
targets += blink.elf
targets += blink01.elf

-include ../../Makefile.avr0.include
