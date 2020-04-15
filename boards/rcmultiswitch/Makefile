
# ~/Projekte/updi/pyupdi/pyupdi.py -d tiny1614 -c /dev/ttyUSB1  -f test00.hex

MCU = attiny1614
F_OSC = 20000000

subdirs =

# digital (OpenTx)
# multi8d (ibus)

# Graupner (schnell) Multikanal MK4
# * multi8mk4  (ppm)
# multi8mk8l (ppm) (high 8)
# multi8mk8h (ppm) (low 8)

# multi8mk4hott  (sumd)
# multi8mk8lhott (sumd)
# multi8mk8hhott (sumd)

# Langsamer Modus - wie Momo-Sauerland / Engel-Modelltechnik
# * multi8man     (ppm)
# multi8manhott (sumd)


targets += test00.elf test00.hex test00.S
targets += test01.elf test01.hex
targets += test10.elf test10.hex
targets += multi8d.hex
targets += multi8mk4.hex
targets += multi8man.hex

-include ../../Makefile.avr0.include
