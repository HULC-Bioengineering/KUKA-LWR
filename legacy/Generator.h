#ifndef __generator_h__
#define __generator_h__

// generator/continuation for C++
// author: Andrew Fedoniouk @ terrainformatica.com
// idea borrowed from: "coroutines in C" Simon Tatham,
//   http://www.chiark.greenend.org.uk/~sgtatham/coroutines.html
#include <vector>

struct _generator
{
  int _line;
  // bottom, top, step
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;
  int size;
  _generator() :_line(0) {}
};

#define $generator(NAME) struct NAME : public _generator

#define $emit(T) bool operator()(T& _rv) { \
                    switch(_line) { case 0:;

#define $stop  } _line = 0; return false; }

#define $yield(V)     \
        do {\
            _line=__LINE__;\
            _rv = (V); return true; case __LINE__:;\
                } while (0)
#endif