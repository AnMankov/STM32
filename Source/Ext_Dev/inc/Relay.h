#ifndef _RELAY_H 
#define _RELAY_H

#include "lib.h"

namespace relay
{
  class TRelay
  {
  public:
    typedef GPIO_TypeDef * TPort;
    typedef std::uint32_t TClkMask;
    typedef std::uint32_t TPinMask;
    typedef char * TSign;
	 
    TRelay(const TPort, const TClkMask, const TPinMask, TSign Sign);
	~TRelay();
	    
    void toggle() const;
    void on() const;
    void off() const;
  protected:
  private:
    TPort GPIOx;        //порт
    TClkMask ClkMask;   //маска для разрешения тактирования
    TPinMask PinMask;   //вывод
    TSign Sign;         //подпись
  };
}

#endif
