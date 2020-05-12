#ifndef __CODING_SWITCH_H
#define __CODING_SWITCH_H

#include "hard_config.h"
#include "rtos_headers.h"

class TCodingSwitch
{
public:
  enum TTmrType : uint8_t
  {
    __SAMPLE = 0,
    __DLY    = 1,
    
    __MAX    = __DLY,
  };
  
  struct TTmrCtrl
  {
    const uint16_t TIME_MS;
    const uint8_t  MAX_QTY;
    uint8_t        Qty;
  };

public:
  TCodingSwitch( const TDevAddr_HWr &_DevAddr_HW );
  ~TCodingSwitch();
  
  void init();       //аппаратная инициализация
  
  uint8_t rd_addr(); //считывается одна выборка с кодового переключателя(ERD716BM06Z)\
                       и преобразуется в десятичный код\
                       валидные положения для ERD716BM06Z: 0..15
  constexpr static uint8_t POLL_TIME_MS   = 100U;
  constexpr static uint16_t DLY_TIME_MS   = 900U;
  constexpr static uint8_t SAMPLE_TIME_MS =   5U;
   
  constexpr static uint8_t BUF_SIZE = POLL_TIME_MS / SAMPLE_TIME_MS;
  uint8_t SampleBuf[BUF_SIZE];
  
  TTmrCtrl SampleTmr;
  TTmrCtrl DlyTmr;  
  TTmrType State;
  
  void sample_ctrl( TimerHandle_t & );
  void dly_ctrl( TimerHandle_t & );
protected:
private:
  const TDevAddr_HWr &DevAddr_HW;
};

#endif //__CODING_SWITCH_H
