
#ifndef __MAX13430EEUB_H
#define __MAX13430EEUB_H

#include <stdint.h>

namespace MAX13430EEUB
{
  constexpr float DRIVER_EN_TIME_us        = { 2.5f }; //данные из Data Sheet Rev. F 
  constexpr float DRIVER_RISE_FALL_TIME_us = { 1.1f };
  constexpr float DRIVER_DISABLE_TIME_us   = { 0.2f };
  constexpr uint32_t MAX_DATA_RATE_kbps    = { 500U };
}

#endif //__MAX13430EEUB_H
