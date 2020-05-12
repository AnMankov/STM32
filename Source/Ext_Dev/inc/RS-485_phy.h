
#ifndef __RS_485_PHY_H
#define __RS_485_PHY_H

#include <stdint.h>
#include "MAX13430EEUB.h"

enum class TConfigurable : uint8_t
{
  _HALF_DUPLEX,
  _FULL_DUPLEX
};

enum class TDePolarity : uint8_t
{
  _DIRECT,
  _INVERSE
};

class T_RS_485_phy
{
  public:
    T_RS_485_phy();
    ~T_RS_485_phy();

	  TConfigurable get_config();    //получить аппаратную конфигурацию интерфейса RS-485
	  TDePolarity get_de_polarity(); //получить полярность входа включения драйвера
  protected:
  private:
    TConfigurable Config;   //аппаратная конфигурация интерфейса RS-485
	  TDePolarity DePolarity; //полярность сигнала DE \
	                            полрность сигнала RE - инверсия DE
};

#endif //__RS_485_PHY_H
