/*
  Devil-Elec
  https://www.mikrocontroller.net/articles/Drehgeber
  new encode Algorithmus from Uwe Kolz, https://www.mikrocontroller.net/topic/112603#7084956
  05.06.2022 
*/
  
#pragma once

//#include <assert.h>
//#include <NanoEveryPin.h>

template <uint8_t a, uint8_t b, uint8_t raster = 2, int32_t relMin = -2147483648, int32_t relMax = 2147483647>  
class Encoder                    
{
  private:           
    static_assert( ((raster == 1) | (raster == 2) | (raster == 4)), "Raster may be only 1, 2 or 4");
    static_assert( (relMin <= relMax), "relMin may be > relMax");
    InputPin <a> phaseA;
    InputPin <b> phaseB;
    int32_t relCounter;       // relative counter, changeable
    int32_t absCounter;       // absolute counter, not changeable
    int8_t encDelta;          // +/- 1 count per detent and direction indicator 
    uint8_t last {0};               
    int32_t absOld {0};
    
    void updateRelativCounter (void)
    {
      if ( absCounter != absOld)
      {
        if (absCounter > absOld) { relCounter++; }
        else {                     relCounter--; }
        
        absOld = absCounter;
        
        if (relCounter < relMin)      { relCounter = relMax; }
        else if (relCounter > relMax) { relCounter = relMin; }
      }
    }

  public:                  
    Encoder() = default; 
    
    void init (void)
    {
      phaseA.init();
      phaseB.init();
      encode();               // power on state
      relCounter = 0;         // power on state
      absCounter = 0;         // power on state
    }

    void enablePullup (void)
    {
      phaseA.enablePullup();
      phaseB.enablePullup();
      encode();               // power on state
    }
    
    void encode (void)
    {
      uint8_t newPhase {0};                 
                            
      if (phaseA.isOn()) { newPhase |= 0b01; }
      if (phaseB.isOn()) { newPhase |= 0b10; }       
         
      encDelta = 0;
      
      if ( (last & 0b11) != newPhase) // return; // no change --> return
      { 
        last = last << 2;
        last = (last | newPhase) & 0b00111111;

        //----------------------------------------------------------------//
        // Uncomment one of the switch statement to select the resolution //
        //----------------------------------------------------------------//
       
//         if constexpr (1 == raster) // ab C++17
        if (1 == raster)
        { // 1:1 scanning
          switch (last) 
          {
            case 0b111000: [[fallthrough]];
            case 0b100001: [[fallthrough]];
            case 0b000111: [[fallthrough]];
            case 0b011110: encDelta =  1; break;
            case 0b110100: [[fallthrough]];
            case 0b010010: [[fallthrough]];
            case 0b001011: [[fallthrough]];
            case 0b101101: encDelta = -1; break;
            default: break;
          }
        }

        // if constexpr (2 == raster) ab C++17
        if (2 == raster)
        { // 1:2 scanning
          switch (last) 
          {
            case 0b000111: [[fallthrough]];
            case 0b111000: encDelta =  1; break;
            case 0b001011: [[fallthrough]];
            case 0b110100: encDelta = -1; break;
            default: break;
          }
        }

        // if constexpr (4 == raster) ab C++17
        if (4 == raster)
        { // 1:4 scanning
          switch (last)  
          {
            case 0b100001: encDelta =  1; break;
            case 0b010010: encDelta = -1; break;
            default: break;
          }
        }
        
        absCounter += encDelta;            
        updateRelativCounter();
      }
    }
    
    int8_t getDirection() { return encDelta; }                    // Richtung abfragen
    void setRelCounter(const int32_t var) { relCounter = var; }   // relativen Zählwert ändern
    int32_t getRelCounter() { return relCounter;}                 // relativen Zählerwert abfragen 
    int32_t getAbsCounter() { return (absCounter); }              // absoluten Zählerwert abfragen
    bool getA() { return phaseA.isOn(); }                         // Phase A abfragen
    bool getB() { return phaseB.isOn(); }                         // Phase B abfragen
};

