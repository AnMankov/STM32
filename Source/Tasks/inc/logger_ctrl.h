#ifndef __LOGGER_CTRL_H
#define __LOGGER_CTRL_H

#include <cstdint>

__packed struct TBoardData
{
  uint8_t CodeSw;
  int16_t Angle;
  uint8_t SampleValidSign;
};

__packed struct TRec
{
  TBoardData Sens;
  TBoardData Base;

  int16_t OpenAngle;
  uint8_t State;
};

enum TLoggerState : uint8_t
{
  __OFF = 0U,
  __ON  = 1U,
};

constexpr uint8_t __REC_SIZE = sizeof (TRec);


#endif //__LOGGER_CTRL_H
