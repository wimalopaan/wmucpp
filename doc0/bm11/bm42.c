
unsigned char check(float x)
{
   return (0.0 < x);
}

int main() {
     
} 

#if 0

typedef __UINT8_TYPE__ uint8_t;
typedef __INT8_TYPE__ int8_t;


__attribute__((__const__)) extern long lrintf (float __x);
__attribute__((__const__)) extern float floorf (float __x);

static void uart_init (void);
static void uart_putc (const char);

static unsigned mod_mul (unsigned a, unsigned b, unsigned n)
{
  unsigned ab = 0;

  while (1)
    {
      if (a % 2 == 1)
        {
          ab += b;
          if (ab >= n)
            ab -= n;
        }

      a = a / 2;
      if (a == 0)
        return ab;

      b = b * 2;
      if (b >= n)
        b -= n;
    }
}

static unsigned mod_pow16 (unsigned k, unsigned n)
{
  unsigned p = 1;
  unsigned _16 = 16;

  if (n == 1)
    return 0;

  while (_16 >= n)
    _16 -= n;

  while (1)
    {
      if (k % 2 == 1)
        p = mod_mul (_16, p, n);

      k = k / 2;
      if (k == 0)
        break;

      _16 = mod_mul (_16, _16, n);
    }

  return p;
}



static float tame (float s)
{
  int8_t si = (int8_t) lrintf (s);

  if (si <= -2 || si >= 2)
    s -= si;

  return s;
}


static float sigma_a (unsigned n, uint8_t j)
{
  float s = 0.0f;

  for (unsigned k = n-1; k+1 != 0; k--)
    {
      unsigned j_8k = j + 8*k;

      s += mod_pow16 (n-k, j_8k) / (float) j_8k;
      s = tame (s);

      __asm__ __volatile__ ("wdr");
    }

  return s;
}



static float sigma_b (unsigned n, uint8_t j)
{
  float s = 0;
  float _16 = 1.0f;

  for (unsigned k = n; k <= n + 10; k++)
    {
      s += _16 / (8*k + j);
      _16 /= 16;

      __asm__ __volatile__ ("wdr");
    }

  return s;
}


float sigma (unsigned n, uint8_t j)
{
  return sigma_a (n, j) + sigma_b (n, j);
}


float pi_n (unsigned n)
{
  float s = 0.0;

  for (uint8_t i = 0; i < 4; i++)
    {
      uint8_t j = i ? i + 3 : 1;

      int8_t c = -1;
      if (i == 0) c = 4;
      if (i == 1) c = -2;

      s += c * sigma (n, j);
    }

  return s;
}


static uint8_t pi_dig16 (unsigned n)
{
  return 15 & lrintf (floorf (16 * pi_n (n)));
}


static uint8_t hexdigit (uint8_t n)
{
  n += '0';
  return n > '9' ? n + 'A'-'0'-10 : n;
}

int main (void)
{
  uart_init();

  uart_putc ('\n');
  uart_putc ('3');
  uart_putc ('.');

  for (unsigned n = 0; n < 4000; n++)
    {
      if (n % 64 == 0)
        uart_putc ('\n');

      uart_putc (hexdigit (pi_dig16 (n)));
    }

  uart_putc ('\n');

  return 0;
}

void uart_init (void)
{
  unsigned ubrr = -.6 + 22118400 / (8L * 115200);

  (*(volatile uint8_t*)((0x02) + 0x20)) = ubrr >> 8;
  (*(volatile uint8_t*)((0x09) + 0x20)) = ubrr;
  (*(volatile uint8_t*)((0x0B) + 0x20)) = (1 << 1) | (1 << 6);
  (*(volatile uint8_t*)((0x0A) + 0x20)) = (1 << 3);
  (*(volatile uint8_t*)((0x03) + 0x20)) = (1 << 2) | (1 << 1);
}


void uart_putc (char c)
{
  while (!(
           (*(volatile uint8_t*)((0x0B) + 0x20)) 
           & (1 << 5)))
    __asm__ __volatile__ ("wdr");

  (*(volatile uint8_t*)((0x0C) + 0x20)) = c;
}

void exit (int x)
{
  (void) x;

  while (1)
    __asm__ __volatile__ ("wdr");
}
#endif
