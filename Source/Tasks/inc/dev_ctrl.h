
#ifndef __DEV_CTRL_H
#define __DEV_CTRL_H

#include "lib.h"

class TDevCtrl
{
public:
  struct TSensDevCtr
  {
    uint8_t Val;
    const uint8_t Max;
  };

public:
  TDevCtrl();
  ~TDevCtrl();
  
  constexpr static uint32_t __WAITING_MEMS_DEV_RDY_MS  = 100U;
  constexpr static uint32_t __WAITING_SENS_DEV_OK_MS   = 300U; //прикидочный расчет для самой низкой скорости обмена через usart по modbus (1200 бит/с)
  constexpr static uint32_t __WAITING_MEMS_DEV_DONE_MS = 500U;
  constexpr static uint32_t __WAITING_SENS_DEV_RES_MS  = 300U; //прикидочный расчет для самой низкой скорости обмена через usart по modbus (1200 бит/с)
  
  constexpr static uint32_t __SENS_DEV_OK_MS_MAX = 2000U;
  constexpr static uint32_t __SENS_DEV_OK_QTY    = __SENS_DEV_OK_MS_MAX / __WAITING_SENS_DEV_OK_MS;
  
  TSensDevCtr SensDevCtr;
protected:
private:
};

#endif //__DEV_CTRL_H
