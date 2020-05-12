#include "sens_to_base.h"
#include "rtos_headers.h"
#include "modbus_app.h"
#include "dev_determ.h"

//TModbusLink BaseToSens = { UsartInt_HW, TModbusLink::TProcType::__MASTER }; //на датчике не работает

//----- Задача RTOS -------------------------------------------------------------------------------------------------
void sens_to_base( void *Params ) //обмен датчика с базой; только для датчика
{
  constexpr uint16_t DLY_MS = 1000U;

  if ( DevDeterm.get_dev_type() != TModel::TDevType::_HC )
  {
    for ( ;; )
    {
      vTaskDelay( pdMS_TO_TICKS( DLY_MS ) );
    }
  }
  
//  BaseToSens.pin_clk_config();
  for ( ;; )
  {
    vTaskDelay( pdMS_TO_TICKS( DLY_MS ) );
  }
}
//\---- Задача RTOS -------------------------------------------------------------------------------------------------
