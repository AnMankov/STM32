#include <algorithm>

#include "base_to_sens.h"
#include "rtos_headers.h"
#include "modbus_app.h"
#include "dev_determ.h"
#include "discrete_out.h"

TModbusApp BaseToSens = {
                         UsartInt_HW,
                         TModbusLink::TProcType::__MASTER,
											   &BaseToSensPduHandler,
                         &MasterRtoTrigSem,
                         &MasterCommErrSem
                        };

static void acc_rdy();
static void read_meas();
static void pd_poll();
static void wr_axis_rotate();

//----- Задача RTOS -------------------------------------------------------------------------------------------------
void base_to_sens( void *Params ) //обмен базы с датчиком; только для базы; запросы от базы к датчику
{
  constexpr uint16_t DLY_MS = 5U;

  if ( Model.get_dev_type() != TModel::TDevType::_BASE )
  {
    for ( ;; )
    {
      vTaskDelay( pdMS_TO_TICKS( DLY_MS * 200U ) );
    }
  }

  BaseToSens.StrIx = static_cast<uint8_t>( Model.get_dev_type() );
  BaseToSens.pin_clk_config();

  //в объекте до аппаратной инициализации настройки по умолчанию
  //для обмена БАЗА<->ДАТЧИК, настройки usart неизменны
  BaseToSens.set_addr( TModbusApp::BASE_TO_SENS_MB_ADDR );
  
  TModbusApp::TSets IfSets{ 
                           TModbusApp::TBaudRate::_115200, 
                           TModbusApp::TParity::_EVEN, 
                           TModbusApp::TStops::_STOPBITS_1
                          };
  
  BaseToSens.set_sets( IfSets ); //usart отключается, переинициализируется и включается
  
//  BaseToSens.hw_init( TModbusLink::_1_5_CH_BITS_NBR );
  BaseToSens.init_dma();
  
  xTimerStart( PdTmr, 0U );
  
  for ( ;; )
  {
    
    //опрос семафоров \
      от результата опроса зависит запрос, который необходимо сформировать.
    //последовательность выдачи семафоров всегда одинакова: \
      1. семафор запуска измерений - DevSensStartSem \
      2. семафор считывания данных по окончании измерений DevSensResSem
    
    typedef void (*TFnct)();
    
    struct TReq
    {
      SemaphoreHandle_t *Sem;
      TFnct Fnct;
    };
    
    TReq Req[] =
    {
      {
        &DevSensStartSem,
        acc_rdy,
      },
      {
        &DevSensResSem,
        read_meas,
      },
      {
        &PdSem,
        pd_poll,
      },
      {
        &SensWrAxisRotateSem,
        wr_axis_rotate,
      },
    };
    
    for ( auto item : Req )
    {
      if ( xSemaphoreTake( *item.Sem, 0U ) == pdPASS )
      {
        item.Fnct();
      
        BaseToSens.State = TModbusApp::TState::__IDLE;    
        
        do
        {
          BaseToSens.fsm();
        } while ( BaseToSens.State != TModbusApp::TState::__IDLE );
        
        break;
      }
    }
    
    vTaskDelay( pdMS_TO_TICKS( DLY_MS ) );  
  }
}
//\---- Задача RTOS -------------------------------------------------------------------------------------------------

static void wr_axis_rotate()
{
  //формирование запроса на запись регистра
  auto beg = &BaseToSens.PduHandler->Buf[ 0 ];
	auto end = &BaseToSens.PduHandler->Buf[ BaseToSens.PduHandler->BUF_SIZE ];
  
//  Do.open();
  
  TModbusApp::TPDU *PDU =
  std::find_if( beg, end, []( TModbusApp::TPDU item ){
    return ( item.Fnct == &TModbusApp::my_write_sens_axis_rotate );
  } );
  
  if ( PDU == end )
  {
  
  }
  else
  {
    //Пакет запроса, функция 6    
    __packed struct TBase
    {
      uint8_t Addr;
      uint8_t FnctCode;
      TModbusApp::THalfWordParse RegAddr;
      TModbusApp::THalfWordParse NewVal;
    };
    
    __packed struct TReqWrSingleReg
    {
      TBase Base;
      uint16_t Crc;
    };
    
    TReqWrSingleReg *ReqPkt         = reinterpret_cast<TReqWrSingleReg *>(BaseToSens.TxBuf);
    ReqPkt->Base.Addr               = BaseToSens.get_addr();
    ReqPkt->Base.FnctCode           = PDU->FnctNbr;
    ReqPkt->Base.RegAddr.Segment.Lo = (( TModbusApp::TLittleEndian *)&PDU->RegAddr)->Segment.Lo;
    ReqPkt->Base.RegAddr.Segment.Hi = (( TModbusApp::TLittleEndian *)&PDU->RegAddr)->Segment.Hi;
     
    int16_t Data = 0; //признак передачи
    
    ( BaseToSens.*(PDU->Fnct) )( reinterpret_cast<uint32_t>(&Data) );
    ReqPkt->Base.NewVal.Val = Data;
    ReqPkt->Crc             = CRC16( BaseToSens.TxBuf, sizeof ( TBase ) );
    
    BaseToSens.start_transmit( sizeof ( TReqWrSingleReg ) );
//    Do.closed();
  }
}

static void acc_rdy()
{
  //формирование запроса на запуск измерений
  auto beg = &BaseToSens.PduHandler->Buf[ 0 ];
	auto end = &BaseToSens.PduHandler->Buf[ BaseToSens.PduHandler->BUF_SIZE ];
  
//  Do.open();
  
  TModbusApp::TPDU *PDU =
  std::find_if( beg, end, []( TModbusApp::TPDU item ){
    return ( item.Fnct == &TModbusApp::my_write_start_meas_cmd );
  } );
  
  if ( PDU == end )
  {
  
  }
  else
  {
    //Пакет запроса, функция 5    
    __packed struct TBase
    {
      uint8_t Addr;
      uint8_t FnctCode;
      TModbusApp::THalfWordParse RegAddr;
      TModbusApp::THalfWordParse NewState;
    };
    
    __packed struct TReqWrSingleCoil
    {
      TBase Base;
      uint16_t Crc;
    };
    
    TReqWrSingleCoil *ReqPkt        = reinterpret_cast<TReqWrSingleCoil *>(BaseToSens.TxBuf);
    ReqPkt->Base.Addr               = BaseToSens.get_addr();
    ReqPkt->Base.FnctCode           = PDU->FnctNbr;
    ReqPkt->Base.RegAddr.Segment.Lo = (( TModbusApp::TLittleEndian *)&PDU->RegAddr)->Segment.Lo;
    ReqPkt->Base.RegAddr.Segment.Hi = (( TModbusApp::TLittleEndian *)&PDU->RegAddr)->Segment.Hi;
     
    int16_t Data = 0; //признак передачи
    
    ( BaseToSens.*(PDU->Fnct) )( reinterpret_cast<uint32_t>(&Data) );
    ReqPkt->Base.NewState.Val = Data;
    ReqPkt->Crc               = CRC16( BaseToSens.TxBuf, sizeof ( TBase ) );
    
    BaseToSens.start_transmit( sizeof ( TReqWrSingleCoil ) );
//    Do.closed();
  }
}

static void read_meas()
{
  //формирование запроса считывания данных по окончании измерений
  auto beg = &BaseToSens.PduHandler->Buf[ 0 ];
	auto end = &BaseToSens.PduHandler->Buf[ BaseToSens.PduHandler->BUF_SIZE ];

  //поиск первого элемента массива с функцией 4
  //в массиве по которому осуществляется поиск, адреса регистров должны быть соседними, \
    сами элементы массива также должны быть соседними
  TModbusApp::TPDU *PDU =
  std::find_if( beg, end, []( TModbusApp::TPDU item ){
    return ( item.FnctNbr == TModbusApp::TMbFnct::__READ_INPUT_REGISTERS );
  } );
  
  if ( PDU == end )
  {

  }
  else
  {
    //регистр/ы с требуемой функцией существуют/ет
    //необходимо найти их количество    
    uint16_t RegsQty = 0U;
    std::for_each( beg, end, [ &RegsQty ]( TModbusApp::TPDU item ) {
      if ( item.FnctNbr == TModbusApp::TMbFnct::__READ_INPUT_REGISTERS )
      {
        ++RegsQty;
      }
    } );  
  
    //Пакет запроса, функция 4
    
    __packed struct TBase
    {
      uint8_t Addr;
      uint8_t FnctCode;
      TModbusApp::THalfWordParse StartRegAddr;
      TModbusApp::THalfWordParse RegsQty;
    };
    
    __packed struct TReqRdInputRegs
    {
      TBase Base;
      uint16_t Crc;
    };

    TReqRdInputRegs *ReqPkt              = reinterpret_cast<TReqRdInputRegs *>(BaseToSens.TxBuf);
    ReqPkt->Base.Addr                    = BaseToSens.get_addr();
    ReqPkt->Base.FnctCode                = PDU->FnctNbr;
    BaseToSens.m4RegInfo.StartAddr        = PDU->RegAddr;
    BaseToSens.m4RegInfo.Qty              = RegsQty;
    
    ReqPkt->Base.StartRegAddr.Segment.Lo = (( TModbusApp::TLittleEndian * )&PDU->RegAddr)->Segment.Lo;
    ReqPkt->Base.StartRegAddr.Segment.Hi = (( TModbusApp::TLittleEndian * )&PDU->RegAddr)->Segment.Hi;
    ReqPkt->Base.RegsQty.Segment.Lo      = (( TModbusApp::TLittleEndian * )&RegsQty)->Segment.Lo;
    ReqPkt->Base.RegsQty.Segment.Hi      = (( TModbusApp::TLittleEndian * )&RegsQty)->Segment.Hi;
    
    ReqPkt->Crc                          = CRC16( BaseToSens.TxBuf, sizeof ( TBase ) );

    BaseToSens.start_transmit( sizeof ( TReqRdInputRegs ) );
  }
}

static void pd_poll()
{
  //формирование запроса считывания данных по окончании измерений
  auto beg = &BaseToSens.PduHandler->Buf[ 0 ];
	auto end = &BaseToSens.PduHandler->Buf[ BaseToSens.PduHandler->BUF_SIZE ];

  //поиск первого элемента массива с функцией 3
  //в массиве по которому осуществляется поиск, адреса регистров должны быть соседними, \
    сами элементы массива также должны быть соседними
  TModbusApp::TPDU *PDU =
  std::find_if( beg, end, []( TModbusApp::TPDU item ){
    return ( item.FnctNbr == TModbusApp::TMbFnct::__READ_HOLDING_REGISTERS );
  } );
  
  if ( PDU == end )
  {

  }
  else
  {
    //регистр/ы с требуемой функцией существуют/ет
    //необходимо найти их количество    
    uint16_t RegsQty = 0U;
    std::for_each( beg, end, [ &RegsQty ]( TModbusApp::TPDU item ) {
      if ( item.FnctNbr == TModbusApp::TMbFnct::__READ_HOLDING_REGISTERS )
      {
        ++RegsQty;
      }
    } );  
  
    //Пакет запроса, функция 3
    
    __packed struct TBase
    {
      uint8_t Addr;
      uint8_t FnctCode;
      TModbusApp::THalfWordParse StartRegAddr;
      TModbusApp::THalfWordParse RegsQty;
    };
    
    __packed struct TReqRdInputRegs
    {
      TBase Base;
      uint16_t Crc;
    };

    TReqRdInputRegs *ReqPkt              = reinterpret_cast<TReqRdInputRegs *>(BaseToSens.TxBuf);
    ReqPkt->Base.Addr                    = 4U;
    ReqPkt->Base.FnctCode                = PDU->FnctNbr;
    BaseToSens.m3RegInfo.StartAddr        = PDU->RegAddr;
    BaseToSens.m3RegInfo.Qty              = RegsQty;
    
    ReqPkt->Base.StartRegAddr.Segment.Lo = (( TModbusApp::TLittleEndian * )&PDU->RegAddr)->Segment.Lo;
    ReqPkt->Base.StartRegAddr.Segment.Hi = (( TModbusApp::TLittleEndian * )&PDU->RegAddr)->Segment.Hi;
    ReqPkt->Base.RegsQty.Segment.Lo      = (( TModbusApp::TLittleEndian * )&RegsQty)->Segment.Lo;
    ReqPkt->Base.RegsQty.Segment.Hi      = (( TModbusApp::TLittleEndian * )&RegsQty)->Segment.Hi;
    
    ReqPkt->Crc                          = CRC16( BaseToSens.TxBuf, sizeof ( TBase ) );

    BaseToSens.start_transmit( sizeof ( TReqRdInputRegs ) );
  }
}

//по таймеру 1 раз в секунду происходит опрос тестовый опрос СЕНС ПД \
  Параметры: 115200, 8E1, 4, 1016 (float)
void pd_tmr( TimerHandle_t xTimer ) //обработка таймера кодового переключателя
{
//  xSemaphoreGive(PdSem);
}
