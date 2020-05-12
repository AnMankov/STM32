#ifndef __MEDIAN_FILTER_H
#define __MEDIAN_FILTER_H

#include <stdint.h>

class TMedianFilter
{
public:
  struct TPair
  {
    struct TPair *Ptr;
    int16_t       Val;
  };

public:
  TMedianFilter( int16_t _Stopper );
  ~TMedianFilter();
  
  int16_t process( int16_t Val );
protected:
private:
  static constexpr uint8_t Size = 5;

  TPair Buf[ Size ];
  TPair *DataPtr;
  TPair  Small;
  TPair  Big;
  TPair *Successor;
  TPair *Scan;
  TPair *Scanold;
  TPair *Median;
  
  int16_t Stopper;
};

#endif //__MEDIAN_FILTER_H
