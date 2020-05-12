#ifndef __DISCRETE_OUT_H
#define __DISCRETE_OUT_H

#include "lib.h"
#include "main.h"
#include "hard_config.h"
#include "model.h"

class TDiscreteOut
{
public:
  TDiscreteOut( const TPin & );
  ~TDiscreteOut();
  
  void init();        //аппаратная инициализация
  void closed();      //замкнуть
  void open();        //разомкнуть
  void toggle();      //переключить
  void uncalib_acc(); //формирование спец сигнала при некалиброванном акселерометре
  
  constexpr static uint16_t DLY_TIME_MS   = 700U;
  constexpr static uint8_t  PULSE_TIME_MS = 100U;
protected:
private:
  const TPin &Pin;
};

extern TDiscreteOut Do;

#endif //__DISCRETE_OUT_H
