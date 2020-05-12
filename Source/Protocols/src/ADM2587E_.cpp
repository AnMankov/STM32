#include "ADM2587E_.h"

T_ADM2587E::T_ADM2587E() : Config(TConfigurable::_HALF_DUPLEX),
                           DePolarity(TDePolarity::_DIRECT)
{

}

T_ADM2587E::~T_ADM2587E()
{

}

TConfigurable T_ADM2587E::get_config()
{
  return Config;
}

TDePolarity T_ADM2587E::get_de_polarity()
{
  return DePolarity;
}
