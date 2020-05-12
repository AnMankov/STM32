#ifndef __LED_H 
#define __LED_H

#include "hard_config.h"
#include "rtos_headers.h"

class TLed
{
public:

  TLed( 
       const TPin &_Pin,
       SemaphoreHandle_t *Sem
      );
  ~TLed();

  void init(); //аппаратная инициализация

  void on();
  void off();
  void toggle();
  void free_toggle();

  static constexpr uint8_t WORK_DLY = 2U;
protected:
private:
  const TPin &Pin;

  static constexpr uint16_t BLINK_HALF_PERIOD_MS = 500U;
  SemaphoreHandle_t *Sem;
  uint64_t ToggleCtr;
};

extern TLed Led;
extern TLed UCG1;

#endif //__LED_H
