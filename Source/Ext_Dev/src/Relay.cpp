#include "Relay.h"
#include "main.h"

namespace relay
{
  TRelay::TRelay(const TPort Port, const TClkMask ClkMask, const TPinMask PinMask, TSign Sign)
  : GPIOx(Port),
    ClkMask(ClkMask),
    PinMask(PinMask),
    Sign(Sign)
  {
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    if (!LL_AHB2_GRP1_IsEnabledClock(ClkMask)) //включение тактирования GPIO, если не включено
    {
      LL_AHB2_GRP1_EnableClock(ClkMask);
    }
    LL_GPIO_ResetOutputPin(Port, PinMask);
    
	 GPIO_InitStruct.Pin        = PinMask;
	 GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
	 GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
	 GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	 GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
	 LL_GPIO_Init(GPIOx, &GPIO_InitStruct);
  }
	
  TRelay::~TRelay()
  {
  
  }
  
  void TRelay::toggle() const
  {
    LL_GPIO_TogglePin(GPIOx, PinMask);
  }
  
  void TRelay::on()     const
  {
    LL_GPIO_SetOutputPin(GPIOx, PinMask);
  }
  
  void TRelay::off()    const
  {
    LL_GPIO_ResetOutputPin(GPIOx, PinMask);
  }

}

