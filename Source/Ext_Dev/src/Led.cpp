#include "Led.h"

TLed Led = { 
            Led_HW,
            &LedTimer_TrigSem
           }; 

TLed UCG1 = { 
             UCG1_HW,
             nullptr
            }; 

TLed::TLed( 
           const TPin &_Pin,
           SemaphoreHandle_t *_Sem
          )
: 
Pin( _Pin ),
Sem( _Sem ),
ToggleCtr( 0U )
{

}

TLed::~TLed()
{

}

void TLed::init()                                       //аппаратная инициализация
{                                                       
  Pin.en_clk( Pin.ClkPortMask );                        //включение тактирования GPIO, если не включено
  LL_GPIO_ResetOutputPin( Pin.Gpio, Pin.Nbr );
  
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Pin        = Pin.Nbr;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
  LL_GPIO_Init( Pin.Gpio, &GPIO_InitStruct );
}

void TLed::on()
{
  LL_GPIO_SetOutputPin( Pin.Gpio, Pin.Nbr );
}

void TLed::off()
{
  LL_GPIO_ResetOutputPin( Pin.Gpio, Pin.Nbr );
}

void TLed::toggle()
{
//  xSemaphoreTake( *Sem, portMAX_DELAY );
  if (
      ++ToggleCtr
      >=
      ( TZ_Const::__START_HOLD_CTR )
     )
  {
    LL_GPIO_TogglePin( Pin.Gpio, Pin.Nbr );
    ToggleCtr = 0U;
  }
  else
  {

  }
}

void TLed::free_toggle()
{
  LL_GPIO_TogglePin( Pin.Gpio, Pin.Nbr );
}
