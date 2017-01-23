/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#pragma once

// twi / i2c slave für atmega 
// http://rn-wissen.de/wiki/index.php?title=TWI_Slave_mit_avr-gcc

#include "compat/twi.h"
#include "mcu/avr8.h"
#include "mcu/avr/twiaddress.h"
#include "container/fifo.h"

namespace TWI {
class Slave final {
    Slave() = delete;
public:
    static void init() {
        
    }

private:
};

}

//volatile uint8_t buffer_adr; //"Adressregister" für den Buffer

///*Initaliserung des TWI-Inteface. Muss zu Beginn aufgerufen werden, sowie bei einem Wechsel der Slave Adresse
//Parameter adr: gewünschte Slave-Adresse
//*/
//void init_twi_slave(uint8_t adr)
//{
//    TWAR= adr; //Adresse setzen
//	TWCR &= ~(1<<TWSTA)|(1<<TWSTO);
//	TWCR|= (1<<TWEA) | (1<<TWEN)|(1<<TWIE); 	
//	buffer_adr=0xFF;  
//}


////Je nach Statuscode in TWSR müssen verschiedene Bitmuster in TWCR geschreiben werden(siehe Tabellen im Datenblatt!). 
////Makros für die verwendeten Bitmuster:

////ACK nach empfangenen Daten senden/ ACK nach gesendeten Daten erwarten
//#define TWCR_ACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);  

////NACK nach empfangenen Daten senden/ NACK nach gesendeten Daten erwarten     
//#define TWCR_NACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);

////switch to the non adressed slave mode...
//#define TWCR_RESET TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC);  


///*ISR, die bei einem Ereignis auf dem Bus ausgelöst wird. Im Register TWSR befindet sich dann 
//ein Statuscode, anhand dessen die Situation festgestellt werden kann.
//*/
//ISR (TWI_vect)  
//{
//uint8_t data=0;

//switch (TW_STATUS) //TWI-Statusregister prüfen und nötige Aktion bestimmen 
//{

//// Slave Receiver 

//case TW_SR_SLA_ACK: // 0x60 Slave Receiver, Slave wurde adressiert	
//	TWCR_ACK; // nächstes Datenbyte empfangen, ACK danach senden
//	buffer_adr=0xFF; //Bufferposition ist undefiniert
//break;
	
//case TW_SR_DATA_ACK: // 0x80 Slave Receiver, ein Datenbyte wurde empfangen
//	data=TWDR; //Empfangene Daten auslesen
//	if (buffer_adr == 0xFF) //erster Zugriff, Bufferposition setzen
//		{
//			//Kontrolle ob gewünschte Adresse im erlaubten bereich
//			if(data<i2c_buffer_size+1)
//				{
//					buffer_adr= data; //Bufferposition wie adressiert setzen
//				}
//			else
//				{
//					buffer_adr=0; //Adresse auf Null setzen. Ist das sinnvoll? TO DO!
//				}				
//			TWCR_ACK;	// nächstes Datenbyte empfangen, ACK danach, um nächstes Byte anzufordern
//		}
//	else //weiterer Zugriff, nachdem die Position im Buffer gesetzt wurde. NUn die Daten empfangen und speichern
//		{
		
//			if(buffer_adr<i2c_buffer_size+1)
//				{
//						i2cdata[buffer_adr]=data; //Daten in Buffer schreibe	
//				}
//			buffer_adr++; //Buffer-Adresse weiterzählen für nächsten Schreibzugriff
//			TWCR_ACK;	
//		}
//break;


////Slave transmitter

//case TW_ST_SLA_ACK: //0xA8 Slave wurde im Lesemodus adressiert und hat ein ACK zurückgegeben.
//	//Hier steht kein break! Es wird also der folgende Code ebenfalls ausgeführt!
	
//case TW_ST_DATA_ACK: //0xB8 Slave Transmitter, Daten wurden angefordert

//	if (buffer_adr == 0xFF) //zuvor keine Leseadresse angegeben! 
//		{
//			buffer_adr=0;
//		}	
		
//	if(buffer_adr<i2c_buffer_size+1)	
//		{
//			TWDR = i2cdata[buffer_adr]; //Datenbyte senden
//			buffer_adr++; //bufferadresse für nächstes Byte weiterzählen
//		}
//	else
//		{
//			TWDR=0; //Kein Daten mehr im Buffer
//		}
//	TWCR_ACK;
//break;
//case TW_SR_STOP:
//            TWCR_ACK;
//        break;
//case TW_ST_DATA_NACK: // 0xC0 Keine Daten mehr gefordert 
//case TW_SR_DATA_NACK: // 0x88 
//case TW_ST_LAST_DATA: // 0xC8  Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
//default: 	
//    TWCR_RESET;
//break;
	
//} //end.switch (TW_STATUS)
//} //end.ISR(TWI_vect)

////Ende von twislave.c////