#include <algorithm>

#include "rtos_headers.h"
#include "led.h"
#include "model.h"


//----- Задача RTOS -------------------------------------------------------------------------------------------------
void vLedCtrl( void *pvParameters )
{
  Led.init();

  typedef void (TLed::*TFnct)();

  TFnct Fnct[] =
  {
    &TLed::off,
    &TLed::on,
    &TLed::toggle,
  };

  for (;;)
	{
//    vTaskDelay( pdMS_TO_TICKS( 5U ) );
    xSemaphoreTake( LedTimer_TrigSem, portMAX_DELAY );
      ( Led.*Fnct[ Model.get_led_mode() ] )();
	}
}
//-------------------------------------------------------------------------------------------------------------------
