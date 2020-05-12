//--------------------------------------------------------
// Файл описания классов драйвера микросхемы ADM2587E
//--------------------------------------------------------
#ifndef __ADM2587E_H
#define __ADM2587E_H

#include <cstdint>

using std::uint32_t;
using std::uint8_t;

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

class T_ADM2587E
{
  public:
    T_ADM2587E();
    ~T_ADM2587E();

  	const float DRIVER_EN_TIME_us        = {2.5}; //данные из Data Sheet Rev. F 
    const float DRIVER_RISE_FALL_TIME_us = {1.1};
    const float DRIVER_DISABLE_TIME_us   = {0.2};
    const uint32_t MAX_DATA_RATE_kbps    = {500};

	  TConfigurable get_config();    //получить аппаратную конфигурацию интерфейса RS-485
	  TDePolarity get_de_polarity(); //получить полярность входа включения драйвера
  protected:
  private:
    TConfigurable Config;   //аппаратная конфигурация интерфейса RS-485
	  TDePolarity DePolarity; //полярность сигнала DE \
	                            полрность сигнала RE - инверсия DE
};

#endif

