#include "sens_to_master.h"
#include "rtos_headers.h"
#include "modbus_app.h"
#include "dev_determ.h"
#include "model.h"
#include "discrete_out.h"

TModbusApp SensToMaster = {
                           UsartExt_HW,
                           TModbusLink::TProcType::__SLAVE,
											     &SensToPcPduHandler, //предварительная настройка для обмена с Датчика с ПК (работает, если код на галетнике != 0U )
                           &SlaveRtoTrigSem,
                           &SlaveCommErrSem
                          };

void chk_if_sets( 
                 uint8_t BaudRateNbr, 
                 uint8_t ParityAndStopsNbr             
                );                          //функцию разрешается вызывать только при отключенном интерфейсе

static void addr_determ();
                
//----- Задача RTOS -------------------------------------------------------------------------------------------------
void sens_to_master( void *Params ) //обмен датчика с базой; только для датчика
{
  constexpr uint16_t PLACEBO_DLY_MS = 1000U;
  constexpr uint16_t START_DLY_MS   =  250U;

//  Model.set_hc_state( TSettings::THC::_OPENED );
//  Model.set_base_pos_err( TSettings::TPosErr::_POS_ERR );

  if ( DevDeterm.get_dev_type() != TModel::TDevType::_HC )
  {
    for ( ;; )
    {
      vTaskDelay( pdMS_TO_TICKS( PLACEBO_DLY_MS ) );
    }
  }
  
  vTaskDelay( pdMS_TO_TICKS( START_DLY_MS ) ); //задержка чтобы успела хотя бы 2 цикла отработать задача считывания данных с кодового переключателя
  
  SensToMaster.StrIx = static_cast<uint8_t>( Model.get_dev_type() );
  SensToMaster.pin_clk_config();

  SensToMaster.hw_init( TModbusLink::_1_5_CH_BITS_NBR );
  SensToMaster.init_dma();
  
//  TIf If;

  for ( ;; )
  {  
    addr_determ();
    
    SensToMaster.fsm();
  }
}
//\---- Задача RTOS -------------------------------------------------------------------------------------------------

static void addr_determ()
{
  if ( Model.get_interconn() == TInterconn::__BASE )
  {
//    Do.closed();
    SensToMaster.PduHandler = &SensToBasePduHandler;           //взаимодействие с базой
    SensToMaster.set_addr( TModbusApp::BASE_TO_SENS_MB_ADDR ); //модбас-адрес обмена жестко установлен

    TModbusApp::TSets IfSets{ 
                             TModbusApp::TBaudRate::_115200, 
                             TModbusApp::TParity::_EVEN, 
                             TModbusApp::TStops::_STOPBITS_1
                            };
    
    SensToMaster.set_sets( IfSets ); //usart отключается, переинициализируется и включается
//    Do.open();
  }
  else
  {
    SensToMaster.PduHandler = &SensToPcPduHandler; //взаимодействие с пк (или каким-то другим мастером)
    SensToMaster.set_addr( Model.get_mb_addr() );  //модбас-адрес обмена определяется настройкой

    TModbusApp::TSets IfSets{ 
                             ( TModbusApp::TBaudRate )Model.get_u_baud_rate_bit_to_sec(), 
                             TModbusApp::TParity::_EVEN, 
                             TModbusApp::TStops::_STOPBITS_1
                            };
    
    SensToMaster.set_sets( IfSets ); //usart отключается, переинициализируется и включается
  }
}
