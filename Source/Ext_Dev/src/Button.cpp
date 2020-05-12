#include "Button.h"

TButton Btn = { Button_HW }; 

TButton::TButton( const TPin &_Pin )
:
Pin( _Pin )
{

}

TButton::~TButton()
{

}

void TButton::init()
{
  if ( !LL_AHB1_GRP1_IsEnabledClock( Pin.ClkPortMask ) )
  {
    LL_AHB1_GRP1_EnableClock( Pin.ClkPortMask );
  }

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin        = Pin.Nbr;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;

  LL_GPIO_Init( Pin.Gpio, &GPIO_InitStruct );
}

TButton::TState TButton::read() const
{
  return TState( LL_GPIO_IsInputPinSet( Pin.Gpio, Pin.Nbr ) );
}
