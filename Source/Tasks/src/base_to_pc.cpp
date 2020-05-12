#include "base_to_pc.h"
#include "rtos_headers.h"
#include "modbus_app.h"
#include "dev_determ.h"
#include "model.h"
#include "discrete_out.h"

TModbusApp BaseToPc = {
                       UsartExt_HW,
                       TModbusLink::TProcType::__SLAVE,
											 &BaseToPcPduHandler,
                       &SlaveRtoTrigSem,
                       &SlaveCommErrSem
                      };

static void addr_determ();

//----- Задача RTOS -------------------------------------------------------------------------------------------------
void base_to_pc( void *Params ) //обмен базы с ПК; только для базы
{
  constexpr uint16_t PLACEBO_DLY_MS = 1000U;
  constexpr uint16_t START_DLY_MS   =  250U;

  if ( DevDeterm.get_dev_type() != TModel::TDevType::_BASE )
  {
    for ( ;; )
    {
      vTaskDelay( pdMS_TO_TICKS( PLACEBO_DLY_MS ) );
    }
  }
  
  vTaskDelay( pdMS_TO_TICKS( START_DLY_MS ) ); //задержка чтобы успела хотя бы 2 цикла отработать задача считывания данных с кодового переключателя
  
  BaseToPc.StrIx = static_cast<uint8_t>( Model.get_dev_type() );
  BaseToPc.pin_clk_config();

  BaseToPc.hw_init( TModbusLink::_1_5_CH_BITS_NBR );
  BaseToPc.init_dma();

  for ( ;; )
  {
//    Do.closed();
//    addr_determ();
//    Do.open();
    BaseToPc.set_addr( Model.get_mb_addr() ); //модбас-адрес обмена определяется настройкой
    BaseToPc.fsm();
    
  }
}
//\---- Задача RTOS -------------------------------------------------------------------------------------------------

static void addr_determ()
{
  uint8_t CodeSw = Model.get_code_sw();
  
  if ( CodeSw == 0U )
  {
    BaseToPc.set_addr( Model.get_mb_addr() ); //модбас-адрес обмена определяется настройкой
  }
  else
  {
    BaseToPc.set_addr( CodeSw ); //модбас-адрес обмена определяется кодом с галетника
  }
}
