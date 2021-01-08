#include <stdint.h>
#include <avr/io.h>

#define arraySize 20

typedef int MeinArray[arraySize];

static MeinArray a = {1,2,3};
static MeinArray b = {4,5,6};

typedef struct
{
  MeinArray array;
} MeinContainer;

static MeinContainer fillContainer()
{
  MeinContainer c = {0};
  for(unsigned i = 0; i < arraySize; i++)
  {
    c.array[i] = a[i] + b[i] + i;
  }
  return c;
}

void setup(void) 
{
  MeinContainer c = fillContainer();

  for(unsigned i = 0; i < arraySize; i++)
  {
    PORTB = c.array[i];
  }
}

//int main() {
    
//}


#include <stdlib.h>
// Anzahl an Kanaelen
#define MAX_CH 4
// Aufzaehlung fuer die verfuegbaren Kanaele
enum nutzbare_kanaele {CH1=1, CH2=2, CH3=3, CH4=4};
// structure fuer Infos zum Anschluss der HW
struct kanal_info {
  enum nutzbare_kanaele nummer;
  uint8_t on_off_pin;
  uint8_t flowcontrol_pin;
  uint8_t pwm_pin;
  uint8_t pwm_timer;
};

typedef struct kanal_info aaa;

// Array mit Infos/Daten fuer die Kanaele erstellen
aaa kanal_daten[MAX_CH] = {{.nummer = CH1}};

int main(void) {
}
