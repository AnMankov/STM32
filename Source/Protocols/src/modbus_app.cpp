#include <algorithm>
#include <cstring>

#include "modbus_app.h"
#include "model.h"
#include "discrete_out.h"

uint8_t BUF_SIZE = 0U;

uint8_t DinputCtr;

/**********************************
 * КАРТА РЕГИСТРОВ MODBUS для DKS *
 **********************************/

//Адрес;   Параметр;          Описание параметра;                         Тип данных;   Функция в MB;  Примечание;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//1000;    HC;                Состояние крышки;                           boolean;      2;             (для датчика собственное состояние, для базы - результирующее)
//1001;    HC_PosErr;         Ошибка положения крышки;                    boolean;      2;             
//1002;    Base_PosErr;       Ошибка положения базы;                      boolean;      2;             только для базы
//1003;    Connect;           Связь с датчиком на крышке;                 boolean;      2;             только для базы
//1010;    OpenAngle;         Результирующий угол;                        int16;        4;             только для базы
//1015;    CodeSw;            Данные кодового переключателя;              int16;        4;
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
//2000;    Do;                Нормальное состояние дискретного выхода;    boolean;      (1,5);         0 - нормально разомкнут; 1 - нормально замкнут
//2001;    Thr;               Порог срабатывания кршышки, °;              uint16;       (3,6);         только для базы
//2002;    Hyst;              Гистерезис срабатывания кршышки, °;         uint16;       (3,6);         только для базы
//2003;    Bias;              Смещение угла срабатывания кршышки, °;      uint16;       (3,6);         только для базы
//2004;    AxisRotate;        Поворот оси наклона;                        uint16;       (3,6);         только для базы
//2064;    USpeed;            Скорость передачи данных через RS-485;      uint16;       (3,6);         
//2065;    UPar;              Четность и количество стоповых бит;         uint16;       (3,6);         
//2066;    MBAddr;            Адрес в сети Modbus;                        uint16;       (3,6);         
//2420;    ProgNbr;           Версия ПО;                                  uint16;       4;
//2426;    AdmPswd;           Пароль администратора;                      float32;      (3,6);         
//2428;    SuperPswd;         Пароль суперадминистратора;                 float32;      (3,6);         
//3000;    Calib;             Калибровка;                                 int16;        (3,6);         запись N - выполнение команды калибровки CAL N \
                                                                                                       чтение - результат выполнения команды калибровки *
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
//7000;    MyAngle;           Собственный угол платы;                     int16;        4;              
//7001;    SensAngle;         Угол платы датчика;                         int16;        4;             при работе датчика с базой
//7002;    SampleValidSign;   Признак валидности текущего окна выборок;   boolean;      2;             при работе датчика с базой (0 - валидно; 1 - не валидно)
//7003;    BaseCalib;         Калибровка акселерометра на базе;           boolean;      2;             0 - не калиброван; 1 - калиброван
//7004;    HCCalib;           Калибровка акселерометра на датчике;        boolean;      2;             0 - не калиброван; 1 - калиброван
//7015;    CodeSw;            Данные кодового переключателя;              int16;        4;             при работе датчика с базой
//7016;    ProgNbr;           Версия ПО;                                  uint16;       4;             при работе датчика с базой
//7017;    ProgUpd;           Обновление ПО;                              boolean;      (1,5);
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

// *  0 - отказ в выполнении \
     85 - идет выполнение \
     90 - выполнено \
     99 - команды калибровки не выполнялись с момента включения устройства

TModbusApp::TModbusApp(
                       const TUsart_HW &Usart_HW,
                       TProcType _ProcType,
											 TPduHandler *_PduHandler,
                       SemaphoreHandle_t *_RtoTrigSem,
                       SemaphoreHandle_t *_CommErrSem        
                      )
:
TModbusLink( Usart_HW, _ProcType, _RtoTrigSem, _CommErrSem ),
TDevHandlers( _PduHandler ),
m3RegInfo( { 0U, 0U } ),
m4RegInfo( { 0U, 0U } )
{

}

TModbusApp::~TModbusApp()
{

}

//----- конечные автоматы -------------------------------------------------------------------------

void TModbusApp::fsm()
{
  typedef void (TModbusApp::*TFnct)();
  
//  constexpr uint16_t DLY_MS = 5U;
  
  TFnct Fnct[] =
  {
    &TModbusApp::idle,                       //начальное состояние после подачи питания
    &TModbusApp::s_checking_request,
    &TModbusApp::s_formatting_normal_reply
  };
  
  ( this->*Fnct[ State ] )();
}
//-------------------------------------------------------------------------------------------------


//----- обработчики состояний -------------------
void TModbusApp::idle()
{
  //начальное состояние после подачи питания \
    ожидание принятия пакета с запросом
//  IxSlice.Tail = IxSlice.Head;
  if ( get_eof( StrIx, ProcType ) == false ) //если нет неотправленного пакета
  {
    if (
        StrIx == 0U
        &&      
        ProcType == TProcType::__SLAVE 
       )
    {
//      Do.closed();
    }
    start_handle(); //запуск обработки пакета
  }
  else
  {
    if (
        StrIx == 0U
        &&      
        ProcType == TProcType::__MASTER 
       )
    {
//      Do.toggle();
    }
    vTaskDelay( pdMS_TO_TICKS( 2U ) );
    
    return; //есть неотправленный пакет => принимать нельзя
  }
 
//  uint32_t xTicksToWait[] =
//  {
//    pdMS_TO_TICKS( 100U ), //__MASTER
//    portMAX_DELAY,         //__SLAVE 
//  };
  
  constexpr uint32_t xTicksToWait = 100U;
  
    if (
        StrIx == 0U
        &&      
        ProcType == TProcType::__SLAVE 
       )
    {
//      Do.open();
    }
  if (
      xSemaphoreTake( *RtoTrigSem, xTicksToWait ) == pdPASS //ожидание срабатывания флага RTO в USART
      &&
      xSemaphoreTake( *CommErrSem, 0U ) == pdFAIL //семафор не был выдан
     )
  {
    if (
        StrIx == 0U
        &&      
        ProcType == TProcType::__SLAVE 
       )
    {
//      Do.toggle();
    }
    
    State = TState::__CHECKING_REQUEST;
  }
  else
  {
    //на мастере здесь, возможно, необходимо выдавать семафор для повторной подачи запроса
    
    if ( ProcType == TProcType::__MASTER  )
    {
      stop_receive();                     
    }
    
    State = TState::__IDLE; //при приеме пакета произошли ошибки связи => пакет отбрасывается, ответ мастеру не возвращается
  }
}

void TModbusApp::s_checking_request()
{  
  static uint8_t PrevQty = 0U;
  
  IxSlice.Head = get_rx_dma_data_qty();           //голова буфера
  
  if ( IxSlice.Head == IxSlice.Tail )
  {
//    Do.open();
    static uint8_t Ctr = 0U;
    ++Ctr;
//    Do.toggle();
  }
  
  PrevQty = IxSlice.Head;
  
//  en_eob_detect( TModbusLink::_3_5_CH_BITS_NBR ); //флаг RTO на 1.5Ch сработал перестраиваем RTO на 3.5Ch
  
  
  if ( IxSlice.Tail <= IxSlice.Head )
  {
    BUF_SIZE = IxSlice.Head - IxSlice.Tail;
    
    auto beg = &RxBuf[ IxSlice.Tail ];
    auto end = &RxBuf[ IxSlice.Head ];
    std::copy( beg, end, &HandleBuf[ 0U ] );
  }
  else
  {
    BUF_SIZE = RX_BUF_SIZE - IxSlice.Tail + IxSlice.Head;
    
    auto beg = &RxBuf[ IxSlice.Tail ];
    auto end = &RxBuf[ RX_BUF_SIZE ];
    std::copy( beg, end, &HandleBuf[ 0U ] );
    
    beg = &RxBuf[ 0U ];
    end = &RxBuf[ IxSlice.Head ];
    std::copy( beg, end, &HandleBuf[ BUF_SIZE - IxSlice.Head ] );
  }
  
  if ( RxBuf[IxSlice.Tail + 1] > 6U )
  {
    static uint8_t Ctr = 0U;
    ++Ctr;
  }
  
  if (
      StrIx == 0U
      &&      
      ProcType == TProcType::__MASTER
      &&
      HandleBuf[ 1 ] == 4U
     )
  {
//    Do.closed();
  }
  
  if ( handle_frame( HandleBuf, BUF_SIZE ) == true )
  {
//    Do.closed();
    typedef void (TModbusApp::*TFnct)( uint8_t *, uint16_t );
    
    TFnct Fnct[] =
    {
      &TModbusApp::m_processing_reply,           //__MASTER
      &TModbusApp::s_processing_required_action, //__SLAVE 
    };
    
    ( this->*Fnct[ ProcType ] )( HandleBuf, BUF_SIZE );
  }
  else
  {
    State = TState::__IDLE;
  }
  
//  Do.closed();
  stop_receive();
  
  if ( State != TState::__IDLE)
  {
    static uint8_t Ctr = 0U;
    ++Ctr;
  }
}

void TModbusApp::s_formatting_normal_reply()
{

}

void TModbusApp::s_processing_required_action( uint8_t *Buf, uint16_t BUF_SIZE )
{
  typedef void (TModbusApp::*TFnct)( uint8_t *, uint16_t );
  
  struct TAction
  {
    uint8_t Code;
    TFnct   Fnct;
  };

  TAction Action[] =
  {
    { TMbFnct::__READ_COILS,               &TModbusApp::s_read_coils,              },
    { TMbFnct::__READ_DISCRETE_INPUTS,     &TModbusApp::s_read_discrete_inputs,    },
    { TMbFnct::__READ_HOLDING_REGISTERS,   &TModbusApp::s_read_holding_registers,  },
    { TMbFnct::__READ_INPUT_REGISTERS,     &TModbusApp::s_read_input_registers,    },
    { TMbFnct::__WRITE_SINGLE_COIL,        &TModbusApp::s_write_single_coil,       },
    { TMbFnct::__WRITE_SINGLE_REGISTER,    &TModbusApp::s_write_single_register,   },
    { TMbFnct::__WRITE_MULTIPLE_REGISTERS, &TModbusApp::s_write_multiple_register, },
    { TMbFnct::__REPORT_SERVER_ID,         &TModbusApp::s_report_server_id,        },
    { TMbFnct::__BOOT_MODE,                &TModbusApp::s_boot_mode,               },
  };
    
  constexpr uint8_t ACTION_SIZE = sizeof Action / sizeof Action[0];
  
  auto beg = &Action[0];
  auto end = &Action[ACTION_SIZE];
  
	TAction *CurAction = 
	std::find_if( beg, end, [ Buf ]( TAction item ){
		return ( item.Code == Buf[ 1U ] );
	} );
	
  //проверка поддержки сервером запрошенной функции
	if (
			end == CurAction
		 )                                                                    
	{
		//сформировать и отправить пакет с кодом исключения 1 (ILLEGAL FUNCTION)
		Buf[ 1U ] |= 0x80;                                  //Function
		Buf[ 2U ] = TExceptCodes::__ILLEGAL_FNCT;           //Exception Code

//		State = TState::__FORMATTING_ERROR_REPLY;
    s_formatting_error_reply( Buf );
    
    return;
	}
	else
	{
//    Do.closed();
//    Do.toggle();
//    if ( Buf[ 1U ] == 5U )
//    {
//      Do.closed();
//    }
//    if ( Buf[ 1U ] == 4U )
//    {
//      Do.open();
//    }
    
	  ( this->*(CurAction->Fnct) )( Buf, BUF_SIZE ); //выполнение требуемой функции
	}
}

void TModbusApp::s_formatting_error_reply( uint8_t *Buf )
{
  __packed struct TExceptPkt
  {
    uint8_t  Addr;
    uint8_t  FnctCode;
    uint8_t  ExcCode;
  };
  
  __packed struct TErrorPkt
  {
    TExceptPkt ExceptPkt;
    uint16_t   Crc;
  };
  
  *( TExceptPkt *)TxBuf = *( TExceptPkt *)Buf;
  ( (TErrorPkt *)TxBuf )->Crc = CRC16( TxBuf, sizeof (TExceptPkt) );

  if ( (( TExceptPkt *)TxBuf)->FnctCode == 1U )
  {
    static uint8_t Ctr = 0U;
    
    ++Ctr;
  }
  
	start_transmit( sizeof (TErrorPkt) ); //отправить сформированый пакет
  
  State = TState::__IDLE;
}

//----- обработчики состояний для мастера -------
void TModbusApp::m_idle()
{

}

void TModbusApp::m_waiting_turnaround_delay() //реализация при необходимости широковещательного режима
{

}

void TModbusApp::m_waiting_for_reply()
{
  //здесь должен работать прием через DMA
}

void TModbusApp::m_processing_reply( uint8_t *Buf, uint16_t BUF_SIZE )
{
  typedef void (TModbusApp::*TFnct)( uint8_t *, uint16_t );
  
  struct TAction
  {
    uint8_t Code;
    TFnct   Fnct;
  };

  TAction Action[] =
  {
    { TMbFnct::__READ_DISCRETE_INPUTS,     &TModbusApp::m_read_discrete_inputs,    }, // 2
    { TMbFnct::__READ_HOLDING_REGISTERS,   &TModbusApp::m_read_holding_registers,  }, // 3
    { TMbFnct::__READ_INPUT_REGISTERS,     &TModbusApp::m_read_input_registers,    }, // 4
    { TMbFnct::__WRITE_SINGLE_COIL,        &TModbusApp::m_write_single_coil,       }, // 5
    { TMbFnct::__WRITE_SINGLE_REGISTER,    &TModbusApp::m_write_single_register,   }, // 6
  };
    
  constexpr uint8_t ACTION_SIZE = sizeof Action / sizeof Action[0];
  
  auto beg = &Action[0];
  auto end = &Action[ACTION_SIZE];
  
	TAction *CurAction = 
	std::find_if( beg, end, [ Buf ]( TAction item ){
		return ( item.Code == Buf[ 1U ] );
	} );
	
  //проверка поддержки клиентом функции в ответе
	if (
			end == CurAction
		 )                                                                    
	{
    //на мастере здесь, возможно, необходимо выдавать семафор для повторной подачи запроса
    State = TState::__IDLE;
	}
	else
	{
	  ( this->*(CurAction->Fnct) )( Buf, BUF_SIZE ); //выполнение требуемой функции
	}
}

void TModbusApp::m_processing_error()
{

}
//-----------------------------------------------

//----- обработчики функций протокола для мастера ----------
void TModbusApp::m_read_discrete_inputs( uint8_t *Buf, uint16_t BUF_SIZE ) //2
{

}

void TModbusApp::m_read_holding_registers( uint8_t *Buf, uint16_t BUF_SIZE ) //3
{
  auto beg = &PduHandler->Buf[ 0 ];
	auto end = &PduHandler->Buf[ PduHandler->BUF_SIZE ];
	
  __packed union TRegVal
  {
    __packed struct
    {
      uint8_t Hi;
      uint8_t Lo;
    } Part;
    uint16_t Data;
  };
  
	__packed struct TBasic
	{
		uint8_t Addr;
		uint8_t FnctCode;
    uint8_t BytesQty;
    TRegVal First;			
	};
	
	const TBasic *InputPkt = reinterpret_cast<const TBasic *>(Buf);
  
  if ( InputPkt->BytesQty != m3RegInfo.Qty * 2U ) //если количество принятых байтов с данными не равно количеству запрошенных
  {
    //на мастере здесь, возможно, необходимо выдавать семафор для повторной подачи запроса
    State = TState::__IDLE;
    
    return;
  }
 
  for ( uint8_t Ctr = 0U; Ctr < m3RegInfo.Qty; ++Ctr ) //Проверить существования всех регистров, которые пришли в ответном пакете
  {
    uint16_t CurAddr = m3RegInfo.StartAddr + Ctr;
    
    TPDU *PDU =
    std::find_if( beg, end, [ CurAddr ]( TPDU item ){
      return ( item.RegAddr == CurAddr );
    } );
    
    if ( PDU == end ) //если регистр не найден
    {
      //на мастере здесь, возможно, необходимо выдавать семафор для повторной подачи запроса
      State = TState::__IDLE;
      return;
    }
  }

  for ( uint8_t Ctr = 0U; Ctr < m3RegInfo.Qty; ++Ctr )
  {
    uint16_t CurAddr = m3RegInfo.StartAddr + Ctr;
    const TRegVal *RegVal = &InputPkt->First + Ctr;

    TPDU *PDU =
    std::find_if( beg, end, [ CurAddr ]( TPDU item ){
      return ( item.RegAddr == CurAddr );
    } );
    
    uint16_t Data = RegVal->Data;
    Data = RegVal->Part.Lo + ( RegVal->Part.Hi << 8U );
    ( this->*( PDU->Fnct ) )( reinterpret_cast<uint32_t>(&Data) );
  }

//  m_read_input_registers_complete(); //по окончании вызывается callback, реализация устройствозависимая

  State = TState::__IDLE;
}

void TModbusApp::m_read_input_registers( uint8_t *Buf, uint16_t BUF_SIZE ) //4
{
  auto beg = &PduHandler->Buf[ 0 ];
	auto end = &PduHandler->Buf[ PduHandler->BUF_SIZE ];
	
  __packed union TRegVal
  {
    __packed struct
    {
      uint8_t Hi;
      uint8_t Lo;
    } Part;
    uint16_t Data;
  };
  
	__packed struct TBasic
	{
		uint8_t Addr;
		uint8_t FnctCode;
    uint8_t BytesQty;
    TRegVal First;			
	};
	
	const TBasic *InputPkt = reinterpret_cast<const TBasic *>(Buf);
  
  if ( InputPkt->BytesQty != m4RegInfo.Qty * 2U ) //если количество принятых байтов с данными не равно количеству запрошенных
  {
    //на мастере здесь, возможно, необходимо выдавать семафор для повторной подачи запроса
    State = TState::__IDLE;
    
    return;
  }
 
  for ( uint8_t Ctr = 0U; Ctr < m4RegInfo.Qty; ++Ctr ) //Проверить существования всех регистров, которые пришли в ответном пакете
  {
    uint16_t CurAddr = m4RegInfo.StartAddr + Ctr;
    
    TPDU *PDU =
    std::find_if( beg, end, [ CurAddr ]( TPDU item ){
      return ( item.RegAddr == CurAddr );
    } );
    
    if ( PDU == end ) //если регистр не найден
    {
      //на мастере здесь, возможно, необходимо выдавать семафор для повторной подачи запроса
      State = TState::__IDLE;
      return;
    }
  }

  for ( uint8_t Ctr = 0U; Ctr < m4RegInfo.Qty; ++Ctr )
  {
    uint16_t CurAddr = m4RegInfo.StartAddr + Ctr;
    const TRegVal *RegVal = &InputPkt->First + Ctr;

    TPDU *PDU =
    std::find_if( beg, end, [ CurAddr ]( TPDU item ){
      return ( item.RegAddr == CurAddr );
    } );
    
    uint16_t Data = RegVal->Data;
    Data = RegVal->Part.Lo + ( RegVal->Part.Hi << 8U );
    ( this->*( PDU->Fnct ) )( reinterpret_cast<uint32_t>(&Data) );
  }

  m_read_input_registers_complete(); //по окончании вызывается callback, реализация устройствозависимая

  State = TState::__IDLE;
}

void TModbusApp::m_write_single_coil( uint8_t *Buf, uint16_t BUF_SIZE ) //5
{
  auto beg = &PduHandler->Buf[ 0 ];
	auto end = &PduHandler->Buf[ PduHandler->BUF_SIZE ];
      
	__packed struct TBasic
	{
		uint8_t  Addr;
		uint8_t  FnctCode;
    THalfWordParse CoilAddr;
    uint16_t NewState;
    uint16_t Crc;		
	};

  const TBasic *InputPkt = reinterpret_cast<const TBasic *>(Buf);
  
  TLittleEndian CoilAddr;
  CoilAddr.Segment.Lo = InputPkt->CoilAddr.Segment.Lo;
  CoilAddr.Segment.Hi = InputPkt->CoilAddr.Segment.Hi;
  
  TPDU *PDU =
  std::find_if( beg, end, [ CoilAddr ]( TPDU item ){
    return ( item.RegAddr == CoilAddr.Val );
  } );
  
  if ( PDU == end )
  {
    //регистр не найден
    //на мастере здесь, возможно, необходимо выдавать семафор для повторной подачи запроса
    //return;
  }
  else
  {
    uint16_t Data = __RESP_SUCCESS;    
    ( this->*( PDU->Fnct ) )( reinterpret_cast<uint32_t>( &Data ) );    
  }
  
  State = TState::__IDLE;
}

void TModbusApp::m_write_single_register( uint8_t *Buf, uint16_t BUF_SIZE ) //6
{
  auto beg = &PduHandler->Buf[ 0 ];
	auto end = &PduHandler->Buf[ PduHandler->BUF_SIZE ];
      
	__packed struct TBasic
	{
		uint8_t        Addr;
		uint8_t        FnctCode;
    THalfWordParse RegAddr;
    uint16_t       RegVal;
    uint16_t       Crc;		
	};

  const TBasic *InputPkt = reinterpret_cast<const TBasic *>(Buf);
  
  TLittleEndian RegAddr;
  RegAddr.Segment.Lo = InputPkt->RegAddr.Segment.Lo;
  RegAddr.Segment.Hi = InputPkt->RegAddr.Segment.Hi;
  
  TPDU *PDU =
  std::find_if( beg, end, [ RegAddr ]( TPDU item ){
    return ( item.RegAddr == RegAddr.Val );
  } );
  
  if ( PDU == end )
  {
    //регистр не найден
    //на мастере здесь, возможно, необходимо выдавать семафор для повторной подачи запроса
    //return;
  }
  else
  {
    uint16_t Data = InputPkt->RegVal;    
    ( this->*( PDU->Fnct ) )( reinterpret_cast<uint32_t>( &Data ) );    
  }
  
  State = TState::__IDLE;
}

//----------------------------------------------------------

//----- обработчики функций протокола для слейва -----------
void TModbusApp::s_read_coils( uint8_t *Buf, uint16_t BUF_SIZE ) //1
{
  read_bits( Buf, BUF_SIZE, TMbFnct::__READ_COILS );
}
  
void TModbusApp::s_read_discrete_inputs( uint8_t *Buf, uint16_t BUF_SIZE ) //2
{
  read_bits( Buf, BUF_SIZE, TMbFnct::__READ_DISCRETE_INPUTS );
}
  
void TModbusApp::s_read_holding_registers( uint8_t *Buf, uint16_t BUF_SIZE ) //3
{
  read_regs( Buf, BUF_SIZE, TMbFnct::__READ_HOLDING_REGISTERS );
}
  
void TModbusApp::s_read_input_registers( uint8_t *Buf, uint16_t BUF_SIZE ) //4
{
  read_regs( Buf, BUF_SIZE, TMbFnct::__READ_INPUT_REGISTERS );
}
  
void TModbusApp::s_write_single_coil( uint8_t *Buf, uint16_t BUF_SIZE ) //5
{
  write_single( Buf, BUF_SIZE, TMbFnct::__WRITE_SINGLE_COIL );
}
  
void TModbusApp::s_write_single_register( uint8_t *Buf, uint16_t BUF_SIZE ) //6
{
  write_single( Buf, BUF_SIZE, TMbFnct::__WRITE_SINGLE_REGISTER );                                                                                             
}

void TModbusApp::s_write_multiple_register( uint8_t *Buf, uint16_t BUF_SIZE ) //16
{
  write_mul( Buf, BUF_SIZE, TMbFnct::__WRITE_MULTIPLE_REGISTERS );
}

void TModbusApp::s_report_server_id( uint8_t *Buf, uint16_t BUF_SIZE ) // 17
{
//  write_single( Buf, BUF_SIZE, TMbFnct::__WRITE_SINGLE_REGISTER );
  __packed struct TBasic
	{
	  uint8_t Addr;
		uint8_t FnctCode;
	};

  __packed struct TRespPkt
	{
	  TBasic Basic;
		uint8_t ByteCnt;
		uint8_t ServerID;
	};

	*(TBasic *)TxBuf = *(TBasic *)Buf;

	auto Src = get_id_17();

  ((TRespPkt *)TxBuf)->ByteCnt = ( sizeof Src ) + 1U;
	auto Dest = &((TRespPkt *)TxBuf)->ServerID;



	while ( *Dest++ = *Src++ ) {};

	*--Dest = 0xFF; //замена нулевого символа конца строки на Run Indicator Status = ON
  Dest++;

  auto DataPktSize = sizeof (TRespPkt);
  DataPktSize += std::strlen( get_id_17() ); //размер пакета без Crc
	*(uint16_t *)Dest = CRC16( TxBuf, DataPktSize );
	start_transmit( DataPktSize + 2U ); //отправить сформированый пакет с Crc
	State = TState::__IDLE;
}

void TModbusApp::s_boot_mode( uint8_t *Buf, uint16_t BUF_SIZE ) // 100
{
  __packed struct TReqPkt
	{
    uint8_t  Addr;
    uint8_t  FnctCode;
    uint8_t  ByteCnt;
    uint8_t  Msg[sizeof TDevHandlers::__BOOT_MODE - 1];
    uint16_t Crc;
	};
  
  TReqPkt *Pkt = ( TReqPkt * )Buf;
  
  auto beg_r = reinterpret_cast<uint8_t *>(&Pkt->Msg);
  auto end_r = reinterpret_cast<uint8_t *>(&Pkt->Crc);
  
  auto beg_t = &TDevHandlers::__BOOT_MODE[0U];
  
  static int16_t Determ = 0U;
  
  if ( std::equal( beg_r, end_r, beg_t ) ) //возвращает true, если элементы одинаковы в двух диапазонах
  {
    //определен пакет с командой на обновление прошивки
    ++Determ;
    Model.set_boot_mode_flag( TModel::TBootModeFlag::__BOOT_MODE ); //
    NVIC_SystemReset();  //сброс
  }
  else
  {
    //пакет с командой на обновление прошивки не определен
    --Determ;
  }
  
  set_eof( StrIx, ProcType, false ); //на пакет с командой обновления прошивки отвечать не нужно
	State = TState::__IDLE;
}
//----------------------------------------------------------

void TModbusApp::read_bits( uint8_t *Buf, uint16_t BUF_SIZE, TMbFnct MbFnct )
{
	/*
	Buf[0] - адрес
	Buf[1] - код функции
	Buf[2] - начальный адрес катушки (Hi)
	Buf[3] - начальный адрес катушки (Lo)
	Buf[4] - количество катушек (Hi)
	Buf[5] - количество катушек (Lo)
	Buf[6] - CRC (Lo)
	Buf[7] - CRC (Hi)
	*/
  TPDU *beg = &PduHandler->Buf[ 0 ];
	TPDU *end = &PduHandler->Buf[ PduHandler->BUF_SIZE ];
	
	struct TReadCoilsPkt
	{
		uint8_t Addr;
		uint8_t FnctCode;
		union
		{
			struct
			{
				uint8_t Hi;
				uint8_t Lo;
			} Part;
			uint16_t Data;
		} CoilAddr;
		union
		{
			struct
			{
				uint8_t Hi;
				uint8_t Lo;
			} Part;
			uint16_t Data;
		} CoilQty;
		union
		{
			struct
			{
				uint8_t Lo;
				uint8_t Hi;
			} Part;
			uint16_t Data;
		} Crc;			
	};
	
	const TReadCoilsPkt *ReadCoilsPkt = reinterpret_cast<const TReadCoilsPkt *>(Buf);
  uint16_t CoilAddr = ReadCoilsPkt->CoilAddr.Part.Lo + ( ReadCoilsPkt->CoilAddr.Part.Hi << 8U );
  uint16_t CoilQty_; 
  CoilQty_ = ReadCoilsPkt->CoilQty.Part.Lo  + ( ReadCoilsPkt->CoilQty.Part.Hi  << 8U );

  if (
      CoilQty_ < 1
      ||
      CoilQty_ > 0xA
     )
  {
    //сформировать и отправить пакет с кодом исключения 3 (ILLEGAL DATA VALUE)
    Buf[ 1U ] |= 0x80;                                  //Function
    Buf[ 2U ] = TExceptCodes::__ILLEGAL_DATA_VAL;       //Exception Code

//		  State = TState::__FORMATTING_ERROR_REPLY;
    s_formatting_error_reply( Buf );

    return;
  }

	for ( uint16_t Ctr = 0; Ctr < CoilQty_; Ctr++ )
	{
		uint16_t CurAddr = CoilAddr + Ctr;

//    typedef bool (TDevHandlers::*TChkFnct)( void * );
    
    const TPDU *PDU = nullptr;
    
    if (                          //если хотя-бы один из запрошенных регистров не найден или доступ к нему не разрешен
        chk_legal_item(
                       beg, 
                       end, 
                       CurAddr, 
                       MbFnct,
                       &TDevHandlers::chk_access,
                       &PDU
                      ) == false
       )
    {
		  //сформировать и отправить пакет с кодом исключения 2 (ILLEGAL DATA ADDRESS)
		  Buf[ 1U ] |= 0x80;                                  //Function
		  Buf[ 2U ] = TExceptCodes::__ILLEGAL_DATA_ADDR;      //Exception Code
      
//		  State = TState::__FORMATTING_ERROR_REPLY;
      s_formatting_error_reply( Buf );

			return;
    }
	}
	
	uint8_t BytesQty = 5U;             //( 1:Addr + 1:CodeFnct + 1:BytesQty + 2:CRC )
  uint8_t ByteCnt = 0U;
	BytesQty += ( CoilQty_ / 8U != 0U )  
	          ? ( ByteCnt = CoilQty_ / 8U )           
	          : ( ByteCnt = 1U );           //+ n:Data
  
//  uint8_t *DynBuf = new uint8_t [ BytesQty ]; //память под массив необходимо выделить динамически
	
	uint8_t ByteNbr = 0U;
	TxBuf[ ByteNbr++ ] = Buf[0U];
	TxBuf[ ByteNbr++ ] = Buf[1U];
	TxBuf[ ByteNbr++ ] = ByteCnt;
	
	uint8_t CurByte = 0U;
	for ( uint16_t Ctr = 0, BitsNbr = 0U; Ctr < CoilQty_; Ctr++ )
	{
		uint16_t CurAddr = CoilAddr + Ctr;

		TPDU *PDU =
		std::find_if( beg, end, [ CurAddr ]( TPDU item ){
      return ( item.RegAddr == CurAddr );
		} );
    
		uint8_t Data = 0U;
		
		( this->*(PDU->Fnct) )( reinterpret_cast<uint32_t>(&Data) );
		
		Data <<= BitsNbr;
		CurByte |= Data;
		if ( ++BitsNbr == 8U )
		{
	    TxBuf[ ByteNbr++ ] = CurByte;
			
			BitsNbr = 0U;
			CurByte = 0U;
		}
		else
		{
		  TxBuf[ ByteNbr ] = CurByte;
		}
	}
	
	*(uint16_t *)&TxBuf[ BytesQty - 2U ] = CRC16( TxBuf, BytesQty - 2U );
  
  ++DinputCtr;

	start_transmit( BytesQty ); //отправить сформированый пакет
  
  State = TState::__IDLE;
}

void TModbusApp::read_regs( uint8_t *Buf, uint16_t BUF_SIZE, TMbFnct MbFnct )
{
//  Do.open();
  auto beg = &PduHandler->Buf[ 0 ];
	auto end = &PduHandler->Buf[ PduHandler->BUF_SIZE ];
	
	struct TReadRegsPkt
	{
		uint8_t Addr;
		uint8_t FnctCode;
		union
		{
			struct
			{
				uint8_t Hi;
				uint8_t Lo;
			} Part;
			uint16_t Data;
		} RegAddr;
		union
		{
			struct
			{
				uint8_t Hi;
				uint8_t Lo;
			} Part;
			uint16_t Data;
		} RegQty;
		union
		{
			struct
			{
				uint8_t Lo;
				uint8_t Hi;
			} Part;
			uint16_t Data;
		} Crc;			
	};
	
	const TReadRegsPkt *ReadCoilsPkt = reinterpret_cast<const TReadRegsPkt *>(Buf);
  uint16_t RegAddr = ReadCoilsPkt->RegAddr.Part.Lo + ( ReadCoilsPkt->RegAddr.Part.Hi << 8U );
  uint16_t RegQty_; 
  RegQty_ = ReadCoilsPkt->RegQty.Part.Lo  + ( ReadCoilsPkt->RegQty.Part.Hi  << 8U );

  if ( 
      RegQty_ < 1
      ||
      RegQty_ > 0xA      
     )
  {
    //сформировать и отправить пакет с кодом исключения 3 (ILLEGAL DATA VALUE)
    Buf[ 1U ] |= 0x80;                                  //Function
    Buf[ 2U ] = TExceptCodes::__ILLEGAL_DATA_VAL;       //Exception Code
    
//		  State = TState::__FORMATTING_ERROR_REPLY;
    s_formatting_error_reply( Buf );

    return;
  }

	for ( uint16_t Ctr = 0; Ctr < RegQty_; Ctr++ )
	{
		uint16_t CurAddr = RegAddr + Ctr;
    
    const TPDU *PDU = nullptr;
    
    if (                          //если хотя-бы один из запрошенных регистров не найден или доступ к нему не разрешен
        chk_legal_item(
                       beg, 
                       end, 
                       CurAddr, 
                       MbFnct,
                       &TDevHandlers::chk_access,
                       &PDU
                      ) == false
       )
		{
		  //сформировать и отправить пакет с кодом исключения 2 (ILLEGAL DATA ADDRESS)
		  Buf[ 1U ] |= 0x80;                                  //Function
		  Buf[ 2U ] = TExceptCodes::__ILLEGAL_DATA_ADDR;      //Exception Code

//		  State = TState::__FORMATTING_ERROR_REPLY;
      s_formatting_error_reply( Buf );

			return;
		}
	}
	
	uint8_t BytesQty = 5U;           //( 1:Addr + 1:CodeFnct + 1:BytesQty + 2:CRC )
  uint8_t ByteCnt  = RegQty_ * 2U; //количество байтов с данными в PDU
	BytesQty        += ByteCnt;      //общее количество байтов в пакете
  	
	uint8_t ByteNbr = 0U;            //номер текущего байта
	TxBuf[ ByteNbr++ ] = Buf[0U];
	TxBuf[ ByteNbr++ ] = Buf[1U];
	TxBuf[ ByteNbr++ ] = ByteCnt;
	  
  for ( uint16_t RegCtr = 0U; RegCtr < RegQty_; ++RegCtr )
  {
		uint16_t CurAddr = RegAddr + RegCtr;

		TPDU *PDU =
		std::find_if( beg, end, [ CurAddr ]( TPDU item ){
      return ( item.RegAddr == CurAddr );
		} );
  
    __packed union TDataParse
    {
      uint16_t Val;
      __packed struct
      {
        uint8_t Lo;
        uint8_t Hi;
      } Segment;
    };
    
    TDataParse Data;
    Data.Val = 0U;
		
		( this->*(PDU->Fnct) )( reinterpret_cast<uint32_t>(&Data) );
    TxBuf[ ByteNbr++ ] = Data.Segment.Hi;
    TxBuf[ ByteNbr++ ] = Data.Segment.Lo;
  }
	
	*(uint16_t *)&TxBuf[ BytesQty - 2U ] = CRC16( TxBuf, BytesQty - 2U );

  
	start_transmit( BytesQty ); //отправить сформированый пакет
  
  State = TState::__IDLE;
}

void TModbusApp::write_single( uint8_t *Buf, uint16_t BUF_SIZE, TMbFnct MbFnct )
{
	
//    Do.closed();
  /*
	Buf[0] - адрес
	Buf[1] - код функции
	Buf[2] - адрес катушки/регистра (Hi)
	Buf[3] - адрес катушки/регистра (Lo)
	Buf[4] - новое состояние катушки/значение регистра (Hi)
	Buf[5] - новое состояние катушки/значение регистра (Lo)
	Buf[6] - CRC (Lo)
	Buf[7] - CRC (Hi)
	*/
  auto beg = &PduHandler->Buf[ 0 ];
	auto end = &PduHandler->Buf[ PduHandler->BUF_SIZE ];
	
	struct TWriteSinglePkt
	{
		uint8_t Addr;
		uint8_t FnctCode;
		union
		{
			struct
			{
				uint8_t Hi;
				uint8_t Lo;
			} Part;
			uint16_t Data;
		} RegAddr;
		union
		{
			struct
			{
				uint8_t Hi;
				uint8_t Lo;
			} Part;
			uint16_t Data;
		} RegVal;
		union
		{
			struct
			{
				uint8_t Lo;
				uint8_t Hi;
			} Part;
			uint16_t Data;
		} Crc;			
	};
	
	const TWriteSinglePkt *WriteSinglePkt = reinterpret_cast<const TWriteSinglePkt *>(Buf);
  uint16_t RegAddr = WriteSinglePkt->RegAddr.Part.Lo + ( WriteSinglePkt->RegAddr.Part.Hi << 8U );
  int16_t NewVal  = WriteSinglePkt->RegVal.Part.Lo  + ( WriteSinglePkt->RegVal.Part.Hi  << 8U );

  if ( 
      chk_new_val( NewVal, MbFnct ) == false    
     )
  {
    //сформировать и отправить пакет с кодом исключения 3 (ILLEGAL DATA VALUE)
    Buf[ 1U ] |= 0x80;                                  //Function
    Buf[ 2U ] = TExceptCodes::__ILLEGAL_DATA_VAL;       //Exception Code
    
//		  State = TState::__FORMATTING_ERROR_REPLY;
    s_formatting_error_reply( Buf );

    return;
  }

  const TPDU *PDU = nullptr;
  
  if (                          //если хотя-бы один из запрошенных регистров не найден или доступ к нему не разрешен
      chk_legal_item(
                     beg, 
                     end, 
                     RegAddr, 
                     MbFnct,
                     &TDevHandlers::chk_access,
                     &PDU
                    ) == false
     )
  {
		  //сформировать и отправить пакет с кодом исключения 2 (ILLEGAL DATA ADDRESS)
		  Buf[ 1U ] |= 0x80;                                  //Function
		  Buf[ 2U ] = TExceptCodes::__ILLEGAL_DATA_ADDR;      //Exception Code

//		  State = TState::__FORMATTING_ERROR_REPLY;
      s_formatting_error_reply( Buf );

			return;
  }

  *(TWriteSinglePkt *)TxBuf = *(TWriteSinglePkt *)Buf; //общее количество байтов в пакете = BUF_SIZE \
                                                         ответ - эхо запроса
                                                         
  int16_t Data = NewVal;
  
  if (
      ( this->*(PDU->Fnct) )( reinterpret_cast<uint32_t>(&Data) ) == true
     )
  {
    *(uint16_t *)&TxBuf[ BUF_SIZE - 2U ] = CRC16( TxBuf, BUF_SIZE - 2U );

    start_transmit( BUF_SIZE ); //отправить сформированый пакет
    
//    Do.open();
    
    State = TState::__IDLE;
  }
  else
  {
    //сформировать и отправить пакет с кодом исключения 4 (SERVER DEVICE FAILURE)
    Buf[ 1U ] |= 0x80;                                  //Function
    Buf[ 2U ] = TExceptCodes::__SERVER_DEVICE_FAILURE;  //Exception Code

//		  State = TState::__FORMATTING_ERROR_REPLY;
    s_formatting_error_reply( Buf );

    return;
  }
}

void TModbusApp::write_mul( uint8_t *Buf, uint16_t BUF_SIZE, TMbFnct MbFnct )
{
	/*
	Buf[0] - адрес
	Buf[1] - код функции
	Buf[2] - начальный адрес регистра (Hi)
	Buf[3] - начальный адрес регистра (Lo)
	Buf[4] - количество регистров (Hi)
	Buf[5] - количество регистров (Lo)
	Buf[6] - количество байтов - содержимое регистров
	Buf[7] - CRC (Lo)
	Buf[8] - CRC (Hi)
	*/
  auto beg = &PduHandler->Buf[ 0 ];
	auto end = &PduHandler->Buf[ PduHandler->BUF_SIZE ];
	
	__packed struct TBasic
	{
		uint8_t Addr;
		uint8_t FnctCode;
		__packed union
		{
			__packed struct
			{
				uint8_t Hi;
				uint8_t Lo;
			} Part;
			uint16_t Data;
		} RegAddr;
		
		__packed union
		{
			__packed struct
			{
				uint8_t Hi;
				uint8_t Lo;
			} Part;
			uint16_t Data;
		} RegQty;
	};
	
	__packed struct TReqPkt
	{
		TBasic Basic;
    uint8_t ByteCnt;
		__packed union
		{
			__packed struct
			{
				uint8_t Hi;
				uint8_t Lo;
			} Part;
			int16_t Data;
		} RegStartVal;		
	};
	
	__packed struct TRespPkt
	{
		TBasic Basic;
		uint16_t Crc;
	};
	
	const TReqPkt *ReqPkt = reinterpret_cast<const TReqPkt *>(Buf);
  uint16_t RegAddr = ReqPkt->Basic.RegAddr.Part.Lo + ( ReqPkt->Basic.RegAddr.Part.Hi << 8U ); //начальный адрес регистра для записи
  uint16_t RegQty  = ReqPkt->Basic.RegQty.Part.Lo  + ( ReqPkt->Basic.RegQty.Part.Hi  << 8U ); //количество регистров для записи
	uint8_t ByteCnt  = ReqPkt->ByteCnt;

  if ( 
      chk_new_val( RegQty, MbFnct ) == false
			||
			ByteCnt != RegQty * 2U
     )
  {
    //сформировать и отправить пакет с кодом исключения 3 (ILLEGAL DATA VALUE)
    Buf[ 1U ] |= 0x80;                                  //Function
    Buf[ 2U ] = TExceptCodes::__ILLEGAL_DATA_VAL;       //Exception Code
    
//		  State = TState::__FORMATTING_ERROR_REPLY;
    s_formatting_error_reply( Buf );

    return;
  }
    
	for ( uint8_t Ctr = 0U; Ctr < RegQty; ++Ctr )
	{
		uint16_t CurRegAddr = RegAddr + Ctr;

    const TPDU *PDU = nullptr;
    
    if (                          //если хотя-бы один из запрошенных регистров не найден или доступ к нему не разрешен
        chk_legal_item(
                       beg, 
                       end, 
                       CurRegAddr, 
                       MbFnct,
                       &TDevHandlers::chk_access,
                       &PDU
                      ) == false
       )                                                                 //если запрошенный регистр не найден или доступ к нему не разрешен
		{
				//сформировать и отправить пакет с кодом исключения 2 (ILLEGAL DATA ADDRESS)
				Buf[ 1U ] |= 0x80;                                  //Function
				Buf[ 2U ] = TExceptCodes::__ILLEGAL_DATA_ADDR;      //Exception Code

	//		  State = TState::__FORMATTING_ERROR_REPLY;
				s_formatting_error_reply( Buf );

				return;
		}
	}
  
  TPDU *PDU = nullptr;
	
	for ( uint8_t Ctr = 0U; Ctr < RegQty; ++Ctr )
	{
		uint16_t CurRegAddr = RegAddr + Ctr;
		
		PDU = std::find_if( beg, end, [ CurRegAddr, MbFnct ]( TPDU item ){
						return ( 
										item.RegAddr == CurRegAddr 
										&&
										item.FnctNbr == MbFnct
									 );
		} );
		
    
    __packed union TLittleEndian
    {
      uint16_t Val;
      __packed struct
      {
        uint8_t Lo;
        uint8_t Hi;
      } Segment;
    };
    
		uint16_t Data = *( &ReqPkt->RegStartVal.Data + Ctr );
    Data = ( (TLittleEndian *)&Data )->Segment.Hi + ( ( (TLittleEndian *)&Data )->Segment.Lo << 8U );
		
		if (
				( this->*(PDU->Fnct) )( reinterpret_cast<uint32_t>(&Data) ) == true
			 )
		{

		}
		else
		{
			//сформировать и отправить пакет с кодом исключения 4 (SERVER DEVICE FAILURE)
			Buf[ 1U ] |= 0x80;                                  //Function
			Buf[ 2U ] = TExceptCodes::__SERVER_DEVICE_FAILURE;  //Exception Code

	//		  State = TState::__FORMATTING_ERROR_REPLY;
			s_formatting_error_reply( Buf );

			return;
		}
	}
	
	*(TBasic *)TxBuf = *(TBasic *)Buf;
	( (TRespPkt *)TxBuf )->Crc = CRC16( TxBuf, sizeof (TBasic) );
	start_transmit( sizeof (TRespPkt) ); //отправить сформированый пакет	
	State = TState::__IDLE;	
}

bool TModbusApp::chk_new_val( uint16_t NewVal, TMbFnct MbFnct )
{
  struct TRange
  {
    uint16_t MIN;
    uint16_t MAX;
  };
  
  TRange Range = { 0U, 0U };
  
  switch ( MbFnct )
  {
    case TMbFnct::__WRITE_SINGLE_COIL:
         Range.MIN = 0x0000;
         Range.MAX = 0xFF00;
         return ( 
                 NewVal == Range.MIN
                 ||
                 NewVal == Range.MAX
                );
    case TMbFnct::__WRITE_SINGLE_REGISTER:
         Range.MIN = 0x0000;
         Range.MAX = 0xFFFF;
         return ( 
                 NewVal >= Range.MIN
                 &&
                 NewVal <= Range.MAX
                );
    case TMbFnct::__WRITE_MULTIPLE_REGISTERS:
         Range.MIN = 0x0001;
         Range.MAX = 0x007B;
         return ( 
                 NewVal >= Range.MIN
                 &&
                 NewVal <= Range.MAX
                );
    default:
         return false;
  }
}

void TModbusApp::cmp_if_sets()
{
  TUsart::TSets Sets = get_sets();
  
  TBaudRate BaudRate[] =
  {
    TBaudRate::_1200,   //0
    TBaudRate::_2400,   //1
    TBaudRate::_4800,   //2
    TBaudRate::_9600,   //3
    TBaudRate::_14400,  //4
    TBaudRate::_19200,  //5 - настройка по умолчанию
    TBaudRate::_38400,  //6
    TBaudRate::_56000,  //7
    TBaudRate::_57600,  //8
    TBaudRate::_115200, //9
  };

  struct TParityAndStops
  {
    TParity Parity;
    TStops  Stops;
  };

  TParityAndStops ParityAndStops[] =
  {
    { TUsart::TParity::_NONE, TUsart::TStops::_STOPBITS_1 }, //0
    { TUsart::TParity::_NONE, TUsart::TStops::_STOPBITS_2 }, //1
    { TUsart::TParity::_ODD,  TUsart::TStops::_STOPBITS_1 }, //2
    { TUsart::TParity::_EVEN, TUsart::TStops::_STOPBITS_1 }, //3 - настройка по умолчанию
  };
  
  TBaudRate       BR = BaudRate[ Model.get_u_baud_rate() ];
  TParityAndStops PS = ParityAndStops[ Model.get_u_par() ];
  
  bool ChngFlag = false;
  
  if ( BR != Sets.BaudRate )
  {
    Sets.BaudRate = BR;
    ChngFlag      = true;
  }
  if ( PS.Parity != Sets.Parity )
  {
    Sets.Parity = PS.Parity;
    ChngFlag    = true;
  }
  if ( PS.Stops != Sets.Stops )
  {
    Sets.Stops = PS.Stops;
    ChngFlag   = true;
  }
  
  if ( ChngFlag == true )
  {
    set_sets( Sets );
    //по флагу TCIF=1 в DMA_ISR: \
      отключить USART \
      вызвать функцию hw_init();
  }  
}

bool TModbusApp::chk_legal_item(
                                const TPDU  *beg, 
                                const TPDU  *end, 
                                uint16_t     CurAddr, 
                                TMbFnct      MbFnct,
                                TChkFnct     chk_fnct,
                                const TPDU **target
                               )
{
  for ( auto item = beg; item < end; ++item ) //end - интератор после конца
  {
    if ( 
        item->RegAddr == CurAddr 
        &&
        item->FnctNbr == MbFnct
        &&
        ( this->*chk_fnct )( item->Chk )
       )
    {
      *target = item;
      
      return true; //найден искомый элемент
    }
  }
  
  *target = end;
  
  return false;
}


