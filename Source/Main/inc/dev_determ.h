#ifndef __DEV_DETERM_H
#define __DEV_DETERM_H

#include "lib.h"
#include "main.h"
#include "hard_config.h"
#include "model.h"

class TDevDeterm final
{
public:

  TDevDeterm(
             const TPin &,
             TIM_TypeDef *             
            );
  ~TDevDeterm();

  inline TModel::TDevType get_dev_type() const { return DevType; }
  
  void init_tmr();
  TModel::TDevType is_dev();          //время определения типа устройства = 200мс
  
  static bool TmrTrig;
protected:

private:  
  TModel::TDevType DevType;
  const TPin &HW;
  TIM_TypeDef *Tmr;
  
  constexpr static uint32_t F_TIMER = 100U; //частота работы таймера [Гц]
};

extern TDevDeterm DevDeterm;

#endif //__DEV_DETERM_H
