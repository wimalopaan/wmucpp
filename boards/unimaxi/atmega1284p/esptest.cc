/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

// 20MHz extern
// sudo avrdude -p atmega1284P -P usb -c avrisp2 -U lfuse:w:0xf7:m -U hfuse:w:0xd1:m -U efuse:w:0xfc:m

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/usart.h"
#include "hal/softspimaster.h"
#include "hal/bufferedstream.h"
#include "hal/alarmtimer.h"
#include "hal/constantrate.h"
#include "external/ws2812.h"
#include "external/esp8266/esp8266.h"
#include "external/esp8266/esptimeclient.h"

#include "console.h"

#include <util/eu_dst.h>

auto date = PGMSTRING(__DATE__);

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using SoftSPIData = AVR::Pin<PortB, 1>;
using SoftSPIClock = AVR::Pin<PortB, 0>;
using SoftSPISS = AVR::Pin<PortC, 3>;
using SSpi0 = SoftSpiMaster<SoftSPIData, SoftSPIClock, SoftSPISS>;
//using terminal = SSpi0;
using terminalDevice = AVR::Usart<1>;
using terminal = std::basic_ostream<terminalDevice>;
//using bufferedTerminal = BufferedStream<SSpi0, 512>;

// Timer0
using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer = AlarmTimer<systemTimer>;

// Timer1

// Timer2

// Timer3

// Uart 0
//constexpr uint8_t ESPUartNumber = 0;
//using espUart = AVR::Usart<ESPUartNumber, Esp8266::ATProtocollAdapter<ESPUartNumber>>;
//using espInterface = Esp8266::Interface<espUart>;

using espInterface = Esp8266::Interface<0>;

using espTimeClient = ESP8266::TimeClient<espInterface>;

using statusLed = WS2812<1, AVR::Pin<PortC, 6>>;
typedef statusLed::color_type StatusColor;

using select1 = AVR::Pin<PortC, 5>;

using systemConstantRateAdapter = ConstantRateAdapter<void, AVR::ISR::Timer<0>::CompareA, alarmTimer>;

using isrRegistrar = IsrRegistrar<systemConstantRateAdapter, 
                                    espInterface::usart::RxHandler, espInterface::usart::TxHandler,
//                                  espUart::RxHandler, espUart::TxHandler,
                                  terminalDevice::RxHandler, terminalDevice::TxHandler
>;

namespace Constant {
}

const auto periodicTimer = alarmTimer::create(1000_ms, AlarmFlags::Periodic);

struct TimerHandler : public EventHandler<EventType::Timer> {
    static bool process(uint8_t timer) {
        static uint8_t counter = 0;
        if (timer == *periodicTimer) {
            ++counter;
            if (counter % 2) {
                statusLed::off();
            }
            else {
                StatusColor c = {Red{128}};
                statusLed::set(c);
            }
        }
        return true;
    }
};

struct UsartFeHandler: public EventHandler<EventType::UsartFe> {
    static bool process(uint8_t n) {
        std::outl<terminal>("Usart Fe  "_pgm,  n);
        return true;
    }
};
struct UsartDorHandler: public EventHandler<EventType::UsartDor> {
    static bool process(uint8_t n) {
        std::outl<terminal>("Usart Dor "_pgm, n);
        return true;
    }
};
struct UsartUpeHandler: public EventHandler<EventType::UsartUpe> {
    static bool process(uint8_t n) {
        std::outl<terminal>("Usart Upe "_pgm, n);
        return true;
    }
};
struct UsartRx0: public EventHandler<EventType::UsartRecv0> {
    static bool process(uint8_t c) {
        std::outl<terminal>("Usart rx0 "_pgm, c);
        return true;
    }
};
struct UsartRx1: public EventHandler<EventType::UsartRecv1> {
    static bool process(uint8_t c) {
        if (c == 'r') {
            std::outl<terminal>("reset"_pgm);
//            esp::send(Esp8266::Command::Reset);
        }
        else if (c == 'o') {
            std::outl<terminal>("ok"_pgm);
//            esp::send(Esp8266::Command::Ok);
        }
        else if (c == 'v') {
            std::outl<terminal>("version"_pgm);
//            esp::send(Esp8266::Command::Version);
        }
        else if (c == 'm') {
            std::outl<terminal>("mode"_pgm);
//            esp::send(Esp8266::Command::StationMode);
        }
        else if (c == 'l') {
            std::outl<terminal>("list"_pgm);
//            esp::send(Esp8266::Command::ListAP);
        }
        else if (c == 'k') {
            std::outl<terminal>("listopt"_pgm);
//            esp::send(Esp8266::Command::ListAPOpt);
        }
        else if (c == 'a') {
            std::outl<terminal>("adresses"_pgm);
//            esp::send(Esp8266::Command::AddressInfo);
        }
        else if (c == 'j') {
            std::outl<terminal>("join1"_pgm);
//            esp::send(Esp8266::Command::JoinAP1);
        }
        else if (c == 't') {
            std::outl<terminal>("nist time"_pgm);
//            esp::send(Esp8266::Command::NistTimeS);
        }
        else if (c == 'n') {
            std::outl<terminal>("mux"_pgm);
//            esp::send(Esp8266::Command::Mux);
        }
        else if (c == 'b') {
            std::outl<terminal>("single"_pgm);
//            esp::send(Esp8266::Command::Single);
        }
        else if (c == '?') {
            std::outl<terminal>("status"_pgm);
//            esp::send(Esp8266::Command::Status);
        }
        else if (c == 's') {
//            std::cout << "state: "_pgm << (uint8_t)Esp8266::ATProtocollAdapter<ESPUartNumber>::state() << std::endl;
        }
        return true;
    }
};
//struct EspCDataHandler: public EventHandler<EventType::Esp_CData> {
//    static bool process(uint8_t c) {
//        std::cout << "ESP CData size: "_pgm << esp::data().size() << " : ";
//        for(char c: esp::data()) {
//            std::cout << c;
//        }
//        std::cout << std::endl;
//        return true;
//    }
//};

int main() {
    set_zone(ONE_HOUR); // europe central time
    set_dst(eu_dst);

    isrRegistrar::init();
    systemConstantRateAdapter::init(); 
    terminalDevice::init<19200>();
    statusLed::off();
    
    espTimeClient::init();

    {
        Scoped<EnableInterrupt> interruptEnabler;
        
        using allEventHandler = EventHandlerGroup<TimerHandler,
        UsartDorHandler, UsartFeHandler, UsartUpeHandler,
        UsartRx0, UsartRx1>;
        
        std::outl<terminal>("ESP Test"_pgm);

        EventManager::run3<allEventHandler, espTimeClient::HandlerGroup>([](){
            systemConstantRateAdapter::periodic();
            if (EventManager::unprocessedEvent()) {
                StatusColor c{255};
                statusLed::set(c);
            }
            if (EventManager::leakedEvent()) {
                StatusColor c = {Red{255}, Green{0}, Blue{255}};
                statusLed::set(c);
            }
        });
    }
}

ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(USART0_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART0_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

ISR(USART1_RX_vect) {
    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
}
ISR(USART1_UDRE_vect){
    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, ',', file, ',', line);
    while(true) {}
}
#endif


//SoftwareSerial esp8266(11, 12); // RX, TX

//void setup() {
//  Serial.begin(19200);
//  esp8266.begin(19200);

//  if (!espConfig()) serialDebug();
//  else digitalWrite(LED_WLAN, HIGH);

//  getTime("chronic.herokuapp.com", "/utc/in+one+hours"); //Gets Time from Page and Sets it
//}

//void loop() {
//  String shour, sminute, ssecond;

//  delay(1000);

//  if (hour() <= 9) shour = "0" + String(hour()); else shour = String(hour()); // adjust for 0-9
//  if (minute() <= 9) sminute = "0" + String(minute());  else sminute = String(minute());
//  if (second() <= 9) ssecond = "0" + String(second());  else ssecond = String(second());

//  debug(shour + ":" + sminute + ":" + ssecond);
//}


//String getTCP(String Host, String Subpage)
//{
//  boolean succes = true;

//  succes &= sendCom("AT+CIPSTART=\"TCP\",\"" + Host + "\",80", "OK");
//  String getRequest = "GET " + Subpage + " HTTP/1.1\r\nHost:" + Host + "\r\n\r\n";
//  succes &= sendCom("AT+CIPSEND=" + String(getRequest.length() + 2), ">");

//  return sendCom(getRequest);
//}

//boolean getTime(String Host, String Subpage)
//{
//  boolean succes = true;
//  int xyear, xmonth, xday, xhour, xminute, xsecond;  //lokal variables

//  succes &= sendCom("AT+CIPSTART=\"TCP\",\"" + Host + "\",80", "OK");
//  String getRequest = "GET " + Subpage + " HTTP/1.1\r\nHost:" + Host + "\r\n";
//  succes &= sendCom("AT+CIPSEND=" + String(getRequest.length() + 2), ">");

//  esp8266.println(getRequest);

//  if (esp8266.find("+IPD"))
//  {
//    if (esp8266.find("\r\n\r\n"))
//    {
//      xyear = esp8266.parseInt();
//      xmonth = esp8266.parseInt();
//      xday = esp8266.parseInt();
//      xhour = esp8266.parseInt();
//      xminute = esp8266.parseInt();
//      xsecond = esp8266.parseInt();

//      if (xday < 0) xday *= -1;          //Because of date seperator - parseInt detects negativ integer
//      if (xmonth < 0) xmonth *= -1;    //Because of date seperator - parseInt detects negativ integer


//      setTime(xhour, xminute, xsecond, xday, xmonth, xyear);
//      sendCom("AT+CIPCLOSE", "OK");
//      return true;
//    }
//    else return false;
//  }
//  else return false;
//}

////-----------------------------------------Config ESP8266------------------------------------

//boolean espConfig()
//{
//  boolean succes = true;
//  esp8266.setTimeout(5000);
//  succes &= sendCom("AT+RST", "ready");
//  esp8266.setTimeout(1000);
//  if (configStation(SSID, PASSWORD)) {
//    succes &= true;
//    debug("WLAN Connected");
//    debug("My IP is:");
//    debug(sendCom("AT+CIFSR"));
//  }
//  else
//  {
//    succes &= false;
//  }
//  //shorter Timeout for faster wrong UPD-Comands handling
//  succes &= sendCom("AT+CIPMODE=0", "OK");  //So rum scheit wichtig!
//  succes &= sendCom("AT+CIPMUX=0", "OK");

//  return succes;
//}

//boolean configTCPServer()
//{
//  boolean succes = true;

//  succes &= (sendCom("AT+CIPMUX=1", "OK"));
//  succes &= (sendCom("AT+CIPSERVER=1,80", "OK"));

//  return succes;

//}

//boolean configTCPClient()
//{
//  boolean succes = true;

//  succes &= (sendCom("AT+CIPMUX=0", "OK"));
//  //succes &= (sendCom("AT+CIPSERVER=1,80", "OK"));

//  return succes;

//}


//boolean configStation(String vSSID, String vPASSWORT)
//{
//  boolean succes = true;
//  succes &= (sendCom("AT+CWMODE=1", "OK"));
//  esp8266.setTimeout(20000);
//  succes &= (sendCom("AT+CWJAP=\"" + String(vSSID) + "\",\"" + String(vPASSWORT) + "\"", "OK"));
//  esp8266.setTimeout(1000);
//  return succes;
//}

//boolean configAP()
//{
//  boolean succes = true;

//  succes &= (sendCom("AT+CWMODE=2", "OK"));
//  succes &= (sendCom("AT+CWSAP=\"NanoESP\",\"\",5,0", "OK"));

//  return succes;
//}

//boolean configUDP()
//{
//  boolean succes = true;

//  succes &= (sendCom("AT+CIPMODE=0", "OK"));
//  succes &= (sendCom("AT+CIPMUX=0", "OK"));
//  succes &= sendCom("AT+CIPSTART=\"UDP\",\"192.168.255.255\",90,91,2", "OK"); //Importand Boradcast...Reconnect IP
//  return succes;
//}

////-----------------------------------------------Controll ESP-----------------------------------------------------

//boolean sendUDP(String Msg)
//{
//  boolean succes = true;

//  succes &= sendCom("AT+CIPSEND=" + String(Msg.length() + 2), ">");    //+",\"192.168.4.2\",90", ">");
//  if (succes)
//  {
//    succes &= sendCom(Msg, "OK");
//  }
//  return succes;
//}


//boolean sendCom(String command, char respond[])
//{
//  esp8266.println(command);
//  if (esp8266.findUntil(respond, "ERROR"))
//  {
//    return true;
//  }
//  else
//  {
//    debug("ESP SEND ERROR: " + command);
//    return false;
//  }
//}

//String sendCom(String command)
//{
//  esp8266.println(command);
//  return esp8266.readString();
//}



////-------------------------------------------------Debug Functions------------------------------------------------------
//void serialDebug() {
//  while (true)
//  {
//    if (esp8266.available())
//      Serial.write(esp8266.read());
//    if (Serial.available())
//      esp8266.write(Serial.read());
//  }
//}

//void debug(String Msg)
//{
//  if (DEBUG)
//  {
//    Serial.println(Msg);
//  }
//}

