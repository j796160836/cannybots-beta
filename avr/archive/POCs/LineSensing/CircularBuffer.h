#ifndef CIRCULARBUFFER_h
#define CIRCULARBUFFER_h
#include <inttypes.h>
 
template <typename T, uint16_t Size>
class CircularBuffer {
public:

  CircularBuffer() :
    writePtr(buf), 
    tail(buf+Size) {
  }

  ~CircularBuffer() {
  }

  void push(T value) {
    *writePtr++ = value;
    if (writePtr == tail) 
      writePtr = buf;
  }
  
  int average() {
    int total = 0;
    for (int i=0; i<Size; i++) {
       total += buf[i]; 
    }
    return ( total / Size );
  }
 
private:
  T buf[Size];
  T *writePtr;
  T *tail;
};
 
#endif
