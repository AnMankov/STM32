#include "ADM2587E.h"

namespace ADM2587E
{

  TADM2587E::TADM2587E() : Config(TConfigurable::_HALF_DUPLEX),
                           DePolarity(TDePolarity::_DIRECT)
  {

  }

  TADM2587E::~TADM2587E()
  {

  }

  TConfigurable TADM2587E::get_config()
  {
    return Config;
  }

  TDePolarity TADM2587E::get_de_polarity()
  {
    return DePolarity;
  }
}	

	