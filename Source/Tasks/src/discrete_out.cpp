#include "discrete_out.h"
#include "rtos_headers.h"

TDiscreteOut Do = { DO_HW };  

TDiscreteOut::TDiscreteOut( const TPin &_Pin )
:
Pin( _Pin )
{

}

TDiscreteOut::~TDiscreteOut()
{

}

void TDiscreteOut::init()
{
  if ( !LL_AHB2_GRP1_IsEnabledClock( Pin.ClkPortMask ) )
  {
    LL_AHB2_GRP1_EnableClock( Pin.ClkPortMask );
  }

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin        = Pin.Nbr;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;

  LL_GPIO_Init( Pin.Gpio, &GPIO_InitStruct );
}

void TDiscreteOut::closed() //замкнуть
{
  LL_GPIO_SetOutputPin( Pin.Gpio, Pin.Nbr );
}

void TDiscreteOut::open() //разомкнуть
{
  LL_GPIO_ResetOutputPin( Pin.Gpio, Pin.Nbr );
}

void TDiscreteOut::toggle() //переключить
{
  LL_GPIO_TogglePin( Pin.Gpio, Pin.Nbr );
}

void TDiscreteOut::uncalib_acc() //формирование спец сигнала при некалиброванном акселерометре
{
//  LL_GPIO_ResetOutputPin( Pin.Gpio, Pin.Nbr );
  constexpr uint8_t MAX_PULSE_NBR = 3;

  LL_GPIO_SetOutputPin( Pin.Gpio, Pin.Nbr );
  for ( uint8_t Ctr = 0; Ctr < MAX_PULSE_NBR ; ++Ctr)
  {
    xTimerStart( DoUncalibTmr, 0U );
    xSemaphoreTake( DoUncalibTmr_TrigSem, portMAX_DELAY ); //ожидание срабатывания таймера
  }
  xTimerStop( DoUncalibTmr, 0U ); //если таймер по неопределенной причине не закончил считать
  xTimerChangePeriod( DoUncalibTmr, pdMS_TO_TICKS( DLY_TIME_MS ), 0U );
  xTimerStart( DoUncalibTmr, 0U );
  xSemaphoreTake( DoUncalibTmr_TrigSem, portMAX_DELAY ); //ожидание срабатывания таймера
  xTimerChangePeriod( DoUncalibTmr, pdMS_TO_TICKS( PULSE_TIME_MS ), 0U );
}

void test_calib( uint8_t );

//----- Задача RTOS -------------------------------------------------------------------------------------------------
void discrete_out( void *Params ) //управление дискретным выходом
{
  constexpr uint8_t DLY = 10U; //[мс]
  
  for ( ;; )
  {
    vTaskDelay( pdMS_TO_TICKS( DLY ) );
  }

  Do.init();

  typedef void (TDiscreteOut::*TFnct)();

  TFnct Fnct[  ][ TSettings::_HC_MAX ] =
  {
    &TDiscreteOut::open,   //[ _N_OPENED ][ _CLOSED ]
    &TDiscreteOut::closed, //[ _N_OPENED ][ _OPEN   ]
    &TDiscreteOut::closed, //[ _N_CLOSED ][ _CLOSED ]
    &TDiscreteOut::open,   //[ _N_CLOSED ][ _OPEN   ]
  };

//  while ( Model.get_accel_calib_sign() == TModel::_ACC_UNCALIBRATED )
//  {
//    Do.uncalib_acc(); //при некалиброванном акселерометре на дискретном выходе формируется спец сигнал
//  }

  for (;;)
  {
    TModel::TDevType DevType = Model.get_dev_type();

    uint8_t CodeSw = Model.get_code_sw();

    TContact Set;
    Set = Model.get_d_o_sets();

    TSettings::THC HCState = Model.get_hc_state();

    if ( 
        DevType == TModel::_HC //для датчика первичным является состояние кодового переключателя
        &&
        CodeSw == 0U           //"0" => датчик ждет синхронизацию с базой и ...
       )
    {
      Do.open();               //отключает (размыкает) свой дискретный выход
    }
    else
    {
      ( Do.*Fnct[Set][HCState] )();
    }

    vTaskDelay( pdMS_TO_TICKS( DLY ) );
    
    static uint8_t Nbr = 10U;
    
    test_calib( Nbr );    
    
  }
}
//\---- Задача RTOS -------------------------------------------------------------------------------------------------


//----- Таймер RTOS -------------------------------------------------------------------------------------------------
void do_uncalib_tmr( TimerHandle_t xTimer ) //обработка таймера кодового переключателя
{
  xSemaphoreGive(DoUncalibTmr_TrigSem);
}
//\---- Таймер RTOS -------------------------------------------------------------------------------------------------


//----- Тестовая функция --------------------------------------------------------------------------------------------
void test_calib( uint8_t Nbr )
{
  SemaphoreHandle_t *Sem[] =
  {
    &AccCalib_X_UP_Sem, 
    &AccCalib_X_DOWN_Sem,
    &AccCalib_Y_UP_Sem, 
    &AccCalib_Y_DOWN_Sem,
    &AccCalib_Z_UP_Sem, 
    &AccCalib_Z_DOWN_Sem,
  };

  if ( Nbr != 10U )
  {
    xSemaphoreGive( *Sem[Nbr] );
  }
}
//\---- Таймер RTOS -------------------------------------------------------------------------------------------------
