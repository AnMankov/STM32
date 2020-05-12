#ifndef __BUTTON_H 
#define __BUTTON_H

#include "hard_config.h"

class TButton
{
public:
  enum TState
  {
    __PRESSED     = 0,
    __NOT_PRESSED = 1,
  };

  TButton( const TPin & );
  ~TButton();

  void init();         //аппаратная инициализация
  TState read() const; //считывание состояния кнопки

  uint64_t SampleCtr;

protected:
private:
  const TPin &Pin;
};

extern TButton Btn;

#endif //__BUTTON_H
