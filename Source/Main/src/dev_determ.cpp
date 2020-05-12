
#include <algorithm>
#include "dev_determ.h"
#include "discrete_out.h"

TDevDeterm DevDeterm{
                     DevDetect_HW,
                     TIM7
                    };
                    
bool TDevDeterm::TmrTrig = false;

TDevDeterm::TDevDeterm(
                       const TPin &_HW,
                       TIM_TypeDef *_Tmr
                      )
:
DevType( TModel::_BASE ),
HW( _HW ),
Tmr( _Tmr )
{
  HW.en_clk( HW.ClkPortMask ); //включение тактирования порта, к которому подключен вывод определения типа устройства

  //инициализация вывода
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin        = HW.Nbr;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;

  LL_GPIO_Init( HW.Gpio, &GPIO_InitStruct );
}

TDevDeterm::~TDevDeterm()
{

}

void TDevDeterm::init_tmr()
{
  if ( Tmr == TIM7 )
  {
    LL_TIM_InitTypeDef TIM_InitStruct;

//  ----- Включить тактирование таймера ---------------------------------
    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM7 ); //F3_RM, Rev 8, c.126: If(APB1 prescaler = 1) x1 else x2 => \
                                                            => TIM7 тактируется частотой 72МГц

    LL_RCC_ClocksTypeDef RCC_Clocks;
    LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
//    RCC_Clocks.PCLK1_Frequency

    constexpr uint32_t F_CNT             = 2000U; //частота работы счетчика таймера
    constexpr uint32_t F_TIMER           = 100U;  //частота работы таймера
    uint16_t PRESCALER_VALUE             = (RCC_Clocks.PCLK1_Frequency + (F_CNT / 2U)) / F_CNT - 1U;
    constexpr uint32_t AUTORELOAD_VALUE  = (F_CNT + (F_TIMER / 2U)) / F_TIMER - 1U;
    constexpr uint32_t START_TIMER_VALUE = 0x00;

//  ----- Инициализация таймера -----------------------------------------
    TIM_InitStruct.Prescaler     = PRESCALER_VALUE;
    TIM_InitStruct.CounterMode   = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload    = AUTORELOAD_VALUE;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
//    TIM_InitStruct.RepetitionCounter = ;

    LL_TIM_Init(Tmr, &TIM_InitStruct);                            // Configure the TIMx time base unit
    LL_TIM_ClearFlag_UPDATE(Tmr);

//    LL_TIM_DisableUpdateEvent(MAIN_TIMER);                        // Enable update event generation 
    LL_TIM_SetUpdateSource(Tmr, LL_TIM_UPDATESOURCE_COUNTER);     // Set update event source  
    LL_TIM_SetOnePulseMode(Tmr, LL_TIM_ONEPULSEMODE_REPETITIVE);  // Set one pulse mode
    LL_TIM_SetCounterMode(Tmr, LL_TIM_COUNTERMODE_UP);            // Set the timer counter counting mode
    LL_TIM_DisableARRPreload(Tmr);                                // Enable auto-reload (ARR) preload
    LL_TIM_SetCounter(Tmr, START_TIMER_VALUE);                    // Set the counter value

    //настройка NVIC
    NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));  // 5 - максимальный уровень приоритета для прерывания из которого можно вызывать API функции FreeRTOS 
    NVIC_EnableIRQ(TIM7_IRQn);
  }
}

TModel::TDevType TDevDeterm::is_dev()
{
  //разрешение прерывания по переполнению таймера
  LL_TIM_EnableIT_UPDATE( Tmr );                                  // Enable update interrupt
  LL_TIM_EnableCounter( Tmr );                                    // Enable timer counter

  constexpr uint8_t DETECT_TIME = 200U; //время на определение типа устройства [мс]

  struct TDev
  {
    uint8_t  Ctr;
    TModel::TDevType DevType;
    TPull Pull;
  };

  TDev Base = { 0U, TModel::_BASE, TPull::_UP   };
  TDev HC   = { 0U, TModel::_HC,   TPull::_DOWN };

  TDev *Dev[] =
  {
    &Base, &HC
  };

  uint8_t TmrCtr  = 0U;

  constexpr uint8_t PERIOD  = ( 1000U + ( F_TIMER >> 1U ) ) / F_TIMER;     //[мс]
  constexpr uint8_t MAX_CTR = ( DETECT_TIME + ( PERIOD >> 1U ) ) / PERIOD; 

  while ( TmrCtr != MAX_CTR )
  {
    if ( TDevDeterm::TmrTrig == true )
    {
      TDevDeterm::TmrTrig = false;
      ++TmrCtr;

      //БАЗА    => вывод подтянут вверх      
      //НЕ БАЗА => вывод подтянут вниз
//      if ( LL_GPIO_IsInputPinSet( Pin.Gpio, Pin.Nbr ) == 1U )
//      {
//        ++Base.Ctr;
//      }
//      else
//      {
//        ++HC.Ctr;
//      }
      for ( auto &item : Dev )
      {
        if ( LL_GPIO_IsInputPinSet( HW.Gpio, HW.Nbr ) == item->Pull )
        {
          ++item->Ctr;
        }
      }
    }
  }

  DevType = ( Base.Ctr == MAX_CTR )
          ? TModel::_BASE
          :	( HC.Ctr == TmrCtr )
		      ? TModel::_HC
		      : TModel::_UNDEFINED;

  Model.Main.DevType = DevType;

  LL_TIM_DisableIT_UPDATE( Tmr );                                  // Disable update interrupt
  LL_TIM_DisableCounter( Tmr );

  return DevType;
}

extern "C" void TIM7_IRQHandler(void)
{
  if ( LL_TIM_IsActiveFlag_UPDATE( TIM7 ) )
  {
    TDevDeterm::TmrTrig = true;

    LL_TIM_ClearFlag_UPDATE( TIM7 );                                     // Clear the update interrupt flag (UIF)
  }
}
