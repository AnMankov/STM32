#include <algorithm>

#include "rtos_headers.h"
#include "button.h"
#include "model.h"
#include "led.h"

typedef void (*TFnct)();


//----- Задача RTOS -------------------------------------------------------------------------------------------------
void vPushButton( void *pvParameters )
{
  Btn.init();
//  Test.init();

  xTimerStart( BtnTimer, 0U );
  
	struct TBtnHandler
	{
	  uint16_t          Ctr;
		TSettings::TPress HoldType;
		TSettings::TPress PressType;
	};
  
	constexpr TBtnHandler BtnHandler[] =
	{
	  { TZ_Const::__ANY_PRESS_CTR,  TSettings::TPress::__START_HOLD             , TSettings::TPress::__RELEASE_LESS_1_SEC },   //прошел антидребезг, но нажатие+отпускание < 1 сек
//	  { TZ_Const::__1_PRESS_CTR,    TSettings::TPress::__HOLD_EQ_OR_MORE_1_SEC  , TSettings::TPress::__RELEASE_LESS_3_5_SEC }, //прошел антидребезг, но нажатие+отпускание < 1 сек
//	  { TZ_Const::__3_5_PRESS_CTR,  TSettings::TPress::__HOLD_EQ_OR_MORE_3_5_SEC, TSettings::TPress::__RELEASE_MORE_3_5_SEC },
	};

	auto beg = &BtnHandler[0];
	auto end = &BtnHandler[ sizeof BtnHandler / sizeof BtnHandler[0] ];

//  Test.off();

	for ( ;; )
	{
    xSemaphoreTake( BtnTimer_TrigSem, portMAX_DELAY );
		  if ( Btn.read() == TButton::TState::__PRESSED )
			{
			  ++Btn.SampleCtr;
        std::for_each( beg, end, []( TBtnHandler item ){
				  if ( 
					    Btn.SampleCtr > item.Ctr
						 )
					{
            Model.set_btn_mode( item.HoldType );
					}
				} );
			}
			else
			{        
        std::for_each( beg, end, []( TBtnHandler item ){
				  if ( Model.get_btn_mode() == item.HoldType )
					{					  
            Model.set_btn_mode( item.PressType );
            return ;
					}
				} );
        
			  Btn.SampleCtr = 0U;
			}
	}
}
//-------------------------------------------------------------------------------------------------------------------

void vTimerCallback(TimerHandle_t xTimer)
{
  xSemaphoreGive( BtnTimer_TrigSem ); //отправить семафор - таймер сработал
  xSemaphoreGive( LedTimer_TrigSem ); //отправить семафор - таймер сработал
}

