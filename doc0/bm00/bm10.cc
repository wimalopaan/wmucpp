#include <cstdint>
#include <utility>
#include <tuple>


struct Interface
{
  virtual uint8_t getData() const = 0;
};
template<uint8_t n>
struct Foo : public Interface
{
    Foo() = default;           
    uint8_t getData() const { return n; }
};

namespace {
    Foo<42> f0;  
    Foo<43> f1;  
}

Interface* foo[2] = {&f1, &f1};

void setup (void)
{
//  Serial.begin(250000);
//  Serial.println(F("\nµC Reset ### ### ###")); 
  for (Interface *i : foo)
  {
//    Serial.println(i->getData() );
  }
}
void loop (void)
{ }

int main()
{
    setup();
}
