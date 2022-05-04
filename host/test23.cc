#include <stdio.h>

extern bool a();
extern bool b();

static void test1() 
{
  bool x = a();
  bool y = b();
  
  if (x || y)
    puts("test1");
}
static void test2()
{
  if (a() || b())
    puts("test2");
}

int main() {
    test1();
//    test2();    
}
