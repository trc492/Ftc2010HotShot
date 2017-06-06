typedef struct
{
  int a;
  int b;
  int c;
} S;

typedef struct
{
  S &st[2];
} INTARRAY;

S s1;
S s2;
INTARRAY g_Array;

void ArrayPrint(INTARRAY &intArray)
{
  for (int i = 0; i < 2; ++i)
  {
    debugPrintLine("a[%d]: %d", i, intArray.st[i].a);
    debugPrintLine("b[%d]: %d", i, intArray.st[i].b);
    debugPrintLine("c[%d]: %d", i, intArray.st[i].c);
  }
}

task main()
{
  s1.a = 1;
  s1.b = 2;
  s1.c = 3;
  s2.a = 4;
  s2.b = 5;
  s2.c = 6;
  g_Array.st[0] = s1;
  g_Array.st[1] = s2;
  ArrayPrint(g_Array);
}
