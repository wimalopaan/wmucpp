struct S{
  char a;
};

struct T {
  char a;
};

// aliasing
int test1(S& val1, S& val2){
  val1.a = 10;
  val2.a = 2;
  return val1.a+val2.a;
}

int test2(S& val1, T& val2){
  val1.a = 10;
  val2.a = 2;
  return val1.a+val2.a; 
}

int test3(char& val1, char& val2){
  val1 = 10;
  val2 = 2;
  return val1 + val2; 
}

int test10(S* const val1, S* const val2){
  val1->a = 10;
  val2->a = 2;
  return val1->a+val2->a;
}

int test20(S* const val1, T* const val2){
  val1->a = 10;
  val2->a = 2;
  return val1->a+val2->a; 
}

int test30(char* const val1, char* const val2){
  *val1 = 10;
  *val2 = 2;
  return *val1 + *val2; 
}

template<typename T>
[[gnu::noinline]] int test40(T& val1, T& val2){
    val1.a = 10;
    val2.a = 2;
    return val1.a+val2.a; 
}
template<typename T, typename U>
[[gnu::noinline]] int test40(T& val1, U& val2){
    val1.a = 10;
    val2.a = 2;
    return val1.a+val2.a; 
}

int main() {
    S a;
    T b;
    return test40(a, b);
}
