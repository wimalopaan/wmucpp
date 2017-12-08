test01:
	- ohne Interrupt
	- Timer Test
	- PD7 toggle (SWUsart-TX)

test02:
	- ohne Interrupt
	- Timer Test
	- Alarmtimer (ohne Eventsystem)
	- PD7 toggle (SWUsart-TX)

test03:
	- ohne Interrupt
	- Timer Test
	- Alarmtimer (mit Eventsystem)
	- PD7 toggle (SWUsart-TX)

test04:
	- mit Interrupt
	- Usart Test

test05:
	- ohne Interrupt
	- Usart Test





test03:
	- ohne Interrupt
	- SPI Test
	- Ausgabe auf HW-Spi
	
test04:
	- ohne Interrupt
	- Soft-SPI Test
	- Ausgabe auf Soft-SPI -> mit SPI-Usart-Brücke (AtMega328) auch ausgeben
	
test05:
	- ohne Interrupt
	- I2C-Master (nur schreibend)
	- MCP 23008 Ansteuerung
	
test06:
	- ohne Interrupt
	- I2C-Master /schreiben udn lesend)
	- DS1307 Ansteuerung
	
test07:
	