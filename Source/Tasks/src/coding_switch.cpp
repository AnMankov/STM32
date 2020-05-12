
#include <algorithm>

#include "coding_switch.h"
#include "model.h"
#include "Relay.h"
#include "main.h"

TCodingSwitch CodeSw = { DevAddr_HW };

TCodingSwitch::TCodingSwitch( const TDevAddr_HWr &_DevAddr_HW )
:
SampleTmr( { SAMPLE_TIME_MS, POLL_TIME_MS / SAMPLE_TIME_MS, 0U } ),
DlyTmr( { DLY_TIME_MS, 1U, 0U } ),
State( __SAMPLE ),
DevAddr_HW( _DevAddr_HW )
{

}

TCodingSwitch::~TCodingSwitch()
{

}

void TCodingSwitch::init()
{
  auto beg = &DevAddr_HW[0];
  auto end = &DevAddr_HW[ sizeof DevAddr_HW / sizeof DevAddr_HW[0] ];

  std::for_each( beg, end, []( TDevAddr_HW item ){
    item.Pin.en_clk( item.Pin.ClkPortMask );

    LL_GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin   = item.Pin.Nbr;
    GPIO_InitStruct.Mode  = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull  = LL_GPIO_PULL_UP;

    LL_GPIO_Init( item.Pin.Gpio, &GPIO_InitStruct );

  } );
}

uint8_t TCodingSwitch::rd_addr()
{
  uint8_t Addr = 0;
  
  auto *beg = &DevAddr_HW[0];
  auto *end = &DevAddr_HW[ sizeof DevAddr_HW / sizeof DevAddr_HW[0] ];
  
  std::for_each( beg, end, [&Addr](TDevAddr_HW item){
    if ( LL_GPIO_IsInputPinSet( item.Pin.Gpio, item.Pin.Nbr ) == item.Pull )
    {
      Addr += item.WeightCoeff;
    }
  } );
  
  return Addr;
}

void TCodingSwitch::sample_ctrl( TimerHandle_t &CodeSwTmr )
{
//  RelFour.toggle();
  SampleBuf[SampleTmr.Qty] = rd_addr();

  if ( ++SampleTmr.Qty == SampleTmr.MAX_QTY )
  {
    State = __DLY;
    SampleTmr.Qty = 0U;
    
    auto beg = SampleBuf;
    auto end = &SampleBuf[BUF_SIZE];
    
    if ( xTimerIsTimerActive( CodeSwTmr ) != pdFALSE ) //если таймер по неопределенной причине не закончил считать
    {
      xTimerStop( CodeSwTmr, 0U );
    }
    xTimerChangePeriod( CodeSwTmr, pdMS_TO_TICKS( DlyTmr.TIME_MS ), 0U );
    
    if ( std::equal( beg + 1U, end, beg ) ) //если все элементы массива равны
    {
      Model.set_code_sw( SampleBuf[0] );
    }
    
    std::fill( beg, end, 0U );
    
//    RelFour.on();
  }
  else
  {
  
  }
  
  xTimerStart( CodeSwTmr, 0U );
}

void TCodingSwitch::dly_ctrl( TimerHandle_t &CodeSwTmr )
{
//  RelFour.off();
  
  State = __SAMPLE;   
  if ( xTimerIsTimerActive( CodeSwTmr ) != pdFALSE ) //если таймер по неопределенной причине не закончил считать
  {
    xTimerStop( CodeSwTmr, 0U );
  } 
  xTimerChangePeriod( CodeSwTmr, pdMS_TO_TICKS( SampleTmr.TIME_MS ), 0U );
  
  xTimerStart( CodeSwTmr, 0U );
  
}

//----- Задача RTOS -------------------------------------------------------------------------------------------------
void coding_switch( void *Params ) //обработка кодового переключателя
{
  //В течение 100мс 1 раз в секунду происходит опрос кодового переключателя
  CodeSw.init();

  typedef void (TCodingSwitch::*TFnct)( TimerHandle_t & );

  TFnct Fnct[] =
  {
    &TCodingSwitch::sample_ctrl,
    &TCodingSwitch::dly_ctrl,
  };

  xTimerStart( CodeSwTmr, 0U );
  for (;;)
  {
    xSemaphoreTake( CodeSwTmr_TrigSem, portMAX_DELAY ); //ожидание срабатывания таймера
      ( CodeSw.*Fnct[CodeSw.State] )( CodeSwTmr );
  }
}
//\---- Задача RTOS -------------------------------------------------------------------------------------------------


//----- Таймер RTOS -------------------------------------------------------------------------------------------------
void code_sw_tmr( TimerHandle_t xTimer ) //обработка таймера кодового переключателя
{
  xSemaphoreGive(CodeSwTmr_TrigSem);
}
//\---- Таймер RTOS -------------------------------------------------------------------------------------------------
