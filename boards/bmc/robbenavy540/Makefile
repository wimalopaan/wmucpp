
##### noch unklar

# Fuse
# 8MHz RC-Oszi
# sudo avrdude -p atmega328p -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
# preserve EEPROM
# sudo avrdude -p atmega328p -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0xe2:m -U hfuse:w:0xd1:m -U efuse:w:0xff:m

# Bootloader
# OWL mit Pin PB0
# ...

# optiboot geht nicht wegen PinBelegung
# sudo avrdude -p atmega328p -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U optiboot_flash_atmega328pb_UART1_9600_16000000L.hex

# Firmware
# sudo avrdude -p atmega328p -c arduino -b 9600 -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P /dev/ttyUSB0 -U flash:w:test00.elf

#####


MCU = atmega328p
F_OSC = 8000000

subdirs =

#targets += blink.elf
#targets += test00.elf
#targets += test01.elf
#targets += test02.elf
#targets += test03.elf
targets += test04.elf

-include ../../../Makefile.avr0.include
