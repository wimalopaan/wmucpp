* test01: 
	- Timer Test
	- ohne Interrupt
	- 100Hz Pin toggle LCD PWM Beleuchtung

* test02:
	- SPI Test
	- ohne Interrupt
	- toggle LCD-PWM-Pin wenn SPI-Ready
	
* test03: wie test02
	- mit Interrupt
	- mehr flash Verbrauch
	
* test04: 
	- I2C Test
	- ohne Interrupt
	- I2C: toggle LCD-PWM-Pin wenn Empfang

* test05: 
	- I2C Test
	- mit Interrupt
	- I2C: toggle LCD-PWM-Pin wenn Empfang

* test06:
	- ohne Interrupt
	- Usart

* test07:
	- ohne Interrupt
	- Usart
    - EEProm (FlagsRegister)

* test08:
    - ohne Interrupt
    - Usart
    - EEProm
    - I2C Test (FlagRegister)
    - ohne Interrupt
    - I2C: toggle LCD-PWM-Pin wenn Empfang

* test09:
	- SPI mit FlagRegister
	- ohne Interrupt
	- toggle LCD-PWM-Pin wenn SPI-Ready

* test10:
	- SPI mit FlagRegister
	- mit Interrupt 
	- toggle LCD-PWM-Pin wenn SPI-Ready

* test11:
	- SPI mit FlagRegister
	- ohne Interrupt
	- UART
	
* test12:
	- ohne Interrupt
	- Usart
	- EEProm (FlagRegister)
	- I2C (FlagRegister)
	- SPI (FlagRegister)

* test13:
	- ohne Interrupt
	- Usart
	- EEProm (FlagRegister)
	- I2C (FlagRegister)
	- SPI (FlagRegister)
	- LCD PWM

* test14:
	- ohne Interrupt
	- Usart
	- EEProm (FlagRegister)
	- I2C (FlagRegister)
	- SPI (FlagRegister)
	- LCD PWM
	- LCD Ausgabe

-----
	- Led (WS2812)
	- Drehencoder (über I2C abfragbar) + Taster
	- Menusystem
