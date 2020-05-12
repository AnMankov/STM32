#include "RS-485_phy.h"

T_RS_485_phy::T_RS_485_phy() 
: 
Config( TConfigurable::_HALF_DUPLEX ),
DePolarity( TDePolarity::_DIRECT )
{

}

T_RS_485_phy::~T_RS_485_phy()
{

}

TConfigurable T_RS_485_phy::get_config()
{
  return Config;
}

TDePolarity T_RS_485_phy::get_de_polarity()
{
  return DePolarity;
}

