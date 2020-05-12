#ifndef __SN65HVD233D_H
#define __SN65HVD233D_H

#include <cstdint>

namespace SN65HVD233D
{
  enum class TMode : std::uint8_t
  {
    _NORMAL,         // закольцовывание не используется => либо подтянуть к земле, либо установить Z - состояние на входе LBK
	  _LOOPBACK        // закольцовывание - все, что передаем на вход драйвера - D, оказывается на выходе драйвера - R. У
  };
  
  enum class TRate : std::uint8_t
  {
    _HIGH_SPEED,     // Rs подтянут к GND
	  _SLOPE_CONTROL,  // между Rs и GND подключен резистор 10кОм..100кОм
	  _STANDBY         // Rs подтянут к Vcc - режим прослушки
  };
  
  enum class TPin : std::uint8_t
  {
    _INPUT,
	  _OUTPUT
  };
  
  class TCANPhy
  {
  public:
    TCANPhy();
	  ~TCANPhy();
	 
  protected:
	  void set_mode(TMode);
	  TMode get_mode();
	  
	  void set_rate(TRate);
	  TRate get_rate();
	  
	  TPin get_rs_type();
	  TPin get_lbk_type();
  private:
    TMode Mode;   //определяется аппаратно выводом Rs 
	  TRate Rate;   //определяется аппаратно выводом LBK
	  
	  TPin RsType;  //тип вывода Rs микросхемы
	  TPin LbkType; //тип вывода LBK микросхемы
  };
}

#endif
