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

int main() {
    
}
