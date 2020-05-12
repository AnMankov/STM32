#include "SN65HVD233D.h"

namespace SN65HVD233D
{
  TCANPhy::TCANPhy()
  : Mode(TMode::_NORMAL), //_LOOPBACK для отладки
    Rate(TRate::_SLOPE_CONTROL),
	  RsType(TPin::_INPUT),
	  LbkType(TPin::_INPUT)	 
  {

  }

  TCANPhy::~TCANPhy()
  {

  }

  void TCANPhy::set_mode(TMode _Mode)
  {
    Mode = _Mode;
  }

  TMode TCANPhy::get_mode()
  {
    return Mode;
  }

  void TCANPhy::set_rate(TRate _Rate)
  {
    Rate = _Rate;
  }

  TRate TCANPhy::get_rate()
  {
    return Rate;
  }

  TPin TCANPhy::get_rs_type()
  {
    return RsType;
  }

  TPin TCANPhy::get_lbk_type()
  {
    return LbkType;
  }

}

