/*
  Devil-Elec
  Programmed and tested with: Arduino IDE 1.8.19 & avr-gcc 11.2.0
  24.01.2022
*/

#pragma once

#include <avr/io.h>

namespace
{      
  /* *** im Headerfile definiert ***
    typedef volatile uint8_t  register8_t;
    typedef volatile uint16_t register16_t;
    typedef volatile uint32_t register32_t;
  */

    
  #if defined(__AVR_AVR32DB32__) || defined(__AVR_AVR64DB32__) || defined(__AVR_AVR128DB32__)
    #include "include\AVRxDB32_PinRegister.h" 
    #include "include\AVRxDB32_PinNamen.h"
  #elif defined(__AVR_AVR128DA28__) 
    #include "include/AVRxDB32_PinRegister.h" 
    #include "include/AVRxDB32_PinNamen.h"
  #elif defined(__AVR_AVR32DB48__) || defined(__AVR_AVR64DB48__) || defined(__AVR_AVR128DB48__)
    #include "include\AVRxDB48_PinRegister.h"  
    #include "include\AVRxDB48_PinNamen.h"
  #elif defined(__AVR_AVR64DB64__) || defined(__AVR_AVR128DB64__)
    #include "include\AVRxDB64_PinRegister.h"  
    #include "include\AVRxDB64_PinNamen.h"  
  #else
    #error "Your board has no AVRxDB controller!".
  #endif
 
  #include <assert.h>
  
  #define INLINE inline __attribute__((always_inline))

  // ------------------ VPORTs ------------------ //    
  // Direction
  constexpr register8_t* regVPORTdir(const uint8_t pin)       { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].vport + addrOffset.vDir); }
  
  // Output Level
  constexpr register8_t* regVPORTout(const uint8_t pin)       { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].vport + addrOffset.vOut); }
  
  // Input Level Status
  constexpr register8_t* regVPORTin(const uint8_t pin)        { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].vport + addrOffset.vIn); }
  
  // Interrupt Flag
  constexpr register8_t* regVPORTflag(const uint8_t pin)      { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].vport + addrOffset.vFlag); }

  // ------------------ NORMAL PORTs ------------------ //
  // Pin Mode is Input
  constexpr register8_t* regPORTdirClear(const uint8_t pin)   { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.dirClear); }
  
  // Input Level Status
  constexpr register8_t* regPORTin(const uint8_t pin)         { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.in); }
  
  // Pin Mode is Output
  constexpr register8_t* regPORTdirSet(const uint8_t pin)     { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.dirSet); }
  
  // Pin Level High
  constexpr register8_t* regPORToutSet(const uint8_t pin)     { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.outSet); }
  
  // Pin Level Low
  constexpr register8_t* regPORToutClear(const uint8_t pin)   { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.outClear); }
  
  // Pin Level Toggle
  constexpr register8_t* regPORToutToggle(const uint8_t pin)  { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.outToggle); }
  
  // Interrupt Flag
  constexpr register8_t* regPORTflag(const uint8_t pin)       { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.flag); }
  
  // ------------------ SPECIALs ------------------ //
  // SlewRate
  constexpr register8_t* regPORTctrl(const uint8_t pin)       { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.portCtrl); }
  
  // Invert, Input-Level, Pullup, Interrupt
  constexpr register8_t* regPINctrl(const uint8_t pin)        { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + baseAddr[pin].pinCtrl); }
  
  // Pin Bitmaske
  constexpr uint8_t getMask(const uint8_t pin)                { using namespace Pins::Addr; return (baseAddr[pin].mask); }
  
  /*
  // alternative >> Invert, Input-Level, Pullup, Interrupt
  constexpr register8_t* regPORTconfig(const uint8_t pin)     { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.portConfig);    }
  constexpr register8_t* regPINctrlUpdate(const uint8_t pin)  { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.pinCtrlUpdate); }
  constexpr register8_t* regPINctrlSet(const uint8_t pin)     { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.pinCtrlSet);    }
  constexpr register8_t* regPINctrlClear(const uint8_t pin)   { using namespace Pins::Addr; return (register8_t*) (baseAddr[pin].port + addrOffset.pinCtrlClear);  }
  */   
}
      
template<uint8_t pin>
class Pin
{              
  private:
    static_assert(pin < Pins::Addr::ANZAHLPINS, "pin number not available for this controller");
    bool state {false};
    bool oldRead {false};
    
  protected:
    
    // Inputs
    void INLINE initInput()     { *regVPORTdir(pin) = *regVPORTdir(pin) & ~getMask(pin); }
    
    bool INLINE isOn() const    { return *regVPORTin(pin) & getMask(pin); }

    void INLINE enablePullup()  { *regPINctrl(pin) = *regPINctrl(pin) | PORT_PULLUPEN_bm; }  // Bit 3 Pullup
    
    void INLINE disablePullup() { *regPINctrl(pin) = *regPINctrl(pin) & ~PORT_PULLUPEN_bm; }
               
    // Outputs
    void inline __attribute__((always_inline)) initOutput() { *regVPORTdir(pin) = *regVPORTdir(pin) | getMask(pin); }
    
    void INLINE setOn()      { *regVPORTout(pin) = *regVPORTout(pin) | getMask(pin); }
    
    void INLINE setOff()     { *regVPORTout(pin) = *regVPORTout(pin) & ~getMask(pin); }
    
    void inline __attribute__((always_inline)) toggle()     { *regVPORTin(pin) = getMask(pin); }
    
    // Specials
    void INLINE enableSlewRate()  { *regPORTctrl(pin) = *regPORTctrl(pin) | PORT_SRL_bm; }     // Bit 0 SlewRate, valid for the complete port
    
    void INLINE disableSlewRate() { *regPORTctrl(pin) = *regPORTctrl(pin) & ~PORT_SRL_bm; }
    
    void INLINE enableInvertIO()  { *regPINctrl(pin) = *regPINctrl(pin) | PORT_INVEN_bm; }     // Bit 7 Inverted I/O
    
    void INLINE disableInvertIO() { *regPINctrl(pin) = *regPINctrl(pin) & ~PORT_INVEN_bm; }
    
//    void INLINE enableTTL()       { *regPINctrl(pin) = *regPINctrl(pin) | PORT_INLVL_bm; }
    
//    void INLINE disableTTL()      { *regPINctrl(pin) = *regPINctrl(pin) & ~PORT_INLVL_bm; }
          
    // Interrupts
    void INLINE reset() { *regPINctrl(pin) = *regPINctrl(pin) & ~PORT_ISC_gm; }               // all Interrupts disabled but input buffer enabled
    
    void INLINE senseBothEdges() { 
      reset();      
      *regPINctrl(pin) = *regPINctrl(pin) | 0x01;               // Interrupt enabled with sense on both edges
    }
    
    void INLINE senseRising() { 
      reset();       
      *regPINctrl(pin) = *regPINctrl(pin) | 0x02;               // Interrupt enabled with sense on rising edge
    }
    
    void INLINE senseFalling() { 
      reset();      
      *regPINctrl(pin) = *regPINctrl(pin) | 0x03;               // Interrupt enabled with sense on falling edge
    }
    
    void INLINE senseDisable() { 
      reset();        
      *regPINctrl(pin) = *regPINctrl(pin) | 0x04;               // Interrupt and digital input buffer disabled, the input can not read
    }
        
    void INLINE senseLow() { 
      reset();        
      *regPINctrl(pin) = *regPINctrl(pin) | 0x05;               // Interrupt enabled with sense on low level
    }     
    
    bool INLINE isActiv() const {      
      return *regVPORTflag(pin) & getMask(pin);                 // read the Flag Register and filter
    }

    void INLINE deleteFlag() {       
      *regVPORTflag(pin) = getMask(pin);                        // delete Interrupt Flag
    }
    
    bool INLINE detectRising()                                  // detect rising without retrigger, signal change is required
    { 
      state = false;
      bool read = isOn(); 
      
      if (read && (!oldRead)) {	
        state = true;
      }	
      oldRead = read;
      return state;
    }
    
    void INLINE detectReset() {                                 // or delete detect signal manually
      oldRead = false;
    }
};

/* Test
template<const uint8_t pin>
class DetectRissingEdge : protected Pin<pin>
{
  private:
    bool state {false};
    bool oldRead {false};
       
    void rissing()
    { 
      state = false;
        
      bool read = Pin<pin>::isOn(); 
      
      if (read && (oldRead == LOW) )
      {	
        state = true;
      }	
      oldRead = read;
    } 
    
  public:
    DetectRissingEdge()
    {}
    
    bool detectEdge (void)
    {
      rissing();
      return state;
    }
     
};

template<const uint8_t pin>
class DetectFallingEdge : protected Pin<pin>
{
  private:
    bool state {false};
    bool oldRead {true};
    
    void falling()
    { 
      state = false;
        
      bool read = Pin<pin>::isOn(); 
      
      if (read && (oldRead == HIGH) )
      {	
        state = true;
      }	
      oldRead = read;
    }  
    
  public:
    DetectFallingEdge()
    {}  
};
*/

template<uint8_t pin>
class InputPin : protected Pin<pin> 
{
  public:      
    using Pin<pin>::enablePullup;
    using Pin<pin>::disablePullup;
    using Pin<pin>::isOn;
    using Pin<pin>::enableInvertIO;
    using Pin<pin>::disableInvertIO;
    
    void  INLINE init() {
      Pin<pin>::initInput();
    }
  
    INLINE operator bool() const {
      return  Pin<pin>::isOn();
    }
};

template<uint8_t pin>
class InterruptPin : protected InputPin<pin>
{        
  public:        
    //using Pin<pin>::reset;            // prepared when required
    using Pin<pin>::enablePullup;
    using Pin<pin>::disablePullup;
    //using Pin<pin>::enableInvertIO;   // prepared when required
    //using Pin<pin>::disableInvertIO;  // prepared when required
    using Pin<pin>::isActiv;
    using Pin<pin>::senseBothEdges;
    using Pin<pin>::senseRising;
    using Pin<pin>::senseFalling;
    using Pin<pin>::senseLow;
    using Pin<pin>::senseDisable;
    using Pin<pin>::deleteFlag;
    using Pin<pin>::isOn;
    using Pin<pin>::detectRising;
    using Pin<pin>::detectReset;
          
    void INLINE init()
    {
      Pin<pin>::initInput();
      Pin<pin>::reset();
      Pin<pin>::deleteFlag();
    }           
};

template<uint8_t pin>
class TasterGND : protected InputPin<pin>
{
  public:
    //using InputPin<pin>::init;        // prepared when required
    using Pin<pin>::enablePullup;
    //using Pin<pin>::disablePullup;    // prepared when required
    //using Pin<pin>::enableInvertIO;   // prepared when required
    //using Pin<pin>::disableInvertIO;  // prepared when required
    
    void  INLINE init() {
      Pin<pin>::enableInvertIO();
      Pin<pin>::initInput();
    }
    
    bool INLINE pressed() const     { return InputPin<pin>::isOn(); }
    
    bool INLINE released() const    { return !InputPin<pin>::isOn(); }
    
    INLINE operator bool() const    { return pressed(); }
    
    bool INLINE operator ()() const { return pressed(); }
};
    
template<uint8_t pin>
class OutputPin : protected Pin<pin>
{
  public:       
    using Pin<pin>::setOn;
    using Pin<pin>::setOff;
    using Pin<pin>::toggle;
    using Pin<pin>::isOn;
    using Pin<pin>::enableSlewRate;
    using Pin<pin>::disableSlewRate;
    using Pin<pin>::enableInvertIO;
    using Pin<pin>::disableInvertIO;
  
    void INLINE init() { Pin<pin>::initOutput(); }
   
    void INLINE set(const bool value) {
      if(value) setOn();
      else setOff();
    }
    
    bool INLINE operator = (const bool value) {
      set(value);
      return value;
    }
    
    bool INLINE operator () (const bool value) {
      set(value);
      return value;
    }
    
    INLINE operator bool() const { return Pin<pin>::isOn(); }
};   

template<uint8_t pin>   // for example for low active relay
class OutputPinInvert : public OutputPin<pin>
{
  public:
    void INLINE init() { 
      Pin<pin>::initOutput();
      Pin<pin>::enableInvertIO();
    }

    INLINE operator bool() const { return !Pin<pin>::isOn(); }
};

template<uint8_t pin>
class AnalogInPin : protected Pin<pin>
{
  public:         
    constexpr void init(void) { 
      Pin<pin>::disableInvertIO();
      Pin<pin>::disableSlewRate();
      Pin<pin>::disablePullup();
      Pin<pin>::senseDisable();
      Pin<pin>::deleteFlag();
      Pin<pin>::initInput();
    }
};
