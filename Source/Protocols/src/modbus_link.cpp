#include <algorithm>

#include "modbus_link.h"
#include "discrete_out.h"
#include "dev_determ.h"
#include "dma_types.h"

static BaseType_t SlaveRTOHigherPriorityTaskWoken;
static BaseType_t SlaveCommErrHigherPriorityTaskWoken;

static BaseType_t MasterRTOHigherPriorityTaskWoken;
static BaseType_t MasterCommErrHigherPriorityTaskWoken;


constexpr uint8_t DEV_QTY = 2U;

TEof Eof[ DEV_QTY ][ TModbusLink::TProcType::__MAX ] =
{
  {
   { DMA1,    LL_DMA_CHANNEL_7, false }, //[_BASE][__MASTER]
   { DMA2,    LL_DMA_CHANNEL_6, false }, //[_BASE][__SLAVE]
  },
  {
   { nullptr, 0U,               false }, //[_HC][__MASTER] - запрещенное состояние
   { DMA1,    LL_DMA_CHANNEL_4, false }, //[_HC][__SLAVE]
  }  
};

static void set_eof( DMA_TypeDef *DMA, uint32_t TxChannel, bool NewVal ); //для вызова из обработчиков прерываний

const TDma Dma[][TModbusLink::TProcType::__MAX] =
{
  {
	  {                                                              //[_BASE][__MASTER] - делает запросы, слушает ответы
     { 
       DMA1,
       LL_AHB1_GRP1_EnableClock,
       LL_AHB1_GRP1_PERIPH_DMA1,
       LL_DMA_ClearFlag_TC7
     },                                                              //Nbr
	   { LL_DMA_CHANNEL_6, DMA1_Channel6_IRQn },                       //RxChannel
	   { LL_DMA_CHANNEL_7, DMA1_Channel7_IRQn }                        //TxChannel 
    },
    {                                                              //[_BASE][__SLAVE]  - слушает, отвечает на запросы
     {
       DMA2,
       LL_AHB1_GRP1_EnableClock,
       LL_AHB1_GRP1_PERIPH_DMA2,
       LL_DMA_ClearFlag_TC6
     },                                                              //Nbr
     { LL_DMA_CHANNEL_7, DMA2_Channel7_IRQn },                       //RxChannel 
     { LL_DMA_CHANNEL_6, DMA2_Channel6_IRQn }                        //TxChannel 
    }	
	},
	{
    {                                                              //[_HC][__MASTER]   - запрещенное состояние
     { nullptr, nullptr, 0U, nullptr        },                       //Nbr
     { 0U,               DMA2_Channel7_IRQn },                       //RxChannel
     { 0U,               DMA2_Channel6_IRQn }                        //TxChannel 
    },                                                                      
    {                                                              //[_HC][__SLAVE]    - слушает, отвечает на запросы
     {
       DMA1,
       LL_AHB1_GRP1_EnableClock,
       LL_AHB1_GRP1_PERIPH_DMA1,
       LL_DMA_ClearFlag_TC4
     },                                                              //Nbr
     { LL_DMA_CHANNEL_5, DMA1_Channel5_IRQn },                       //RxChannel 
     { LL_DMA_CHANNEL_4, DMA1_Channel4_IRQn }                        //TxChannel
    }
	}
};
  
TModbusLink::TModbusLink(
                         const TUsart_HW &Usart_HW,
                         TProcType _ProcType,
                         SemaphoreHandle_t *_RtoTrigSem,
                         SemaphoreHandle_t *_CommErrSem
                        )
:
TUsart( Usart_HW ),
StrIx( 0U ),
RtoTrigSem( _RtoTrigSem ),
CommErrSem( _CommErrSem ),
State( __IDLE ),
IxSlice( { 0U, 0U } ),
ProcType( _ProcType )
{
//  init_tmr();
}

TModbusLink::~TModbusLink()
{

}

void TModbusLink::start_transmit( uint8_t NbrBytes )
{
//  Do.open();
  LL_DMA_SetDataLength(
                       Dma[ StrIx ][ ProcType ].Periph.Nbr,
                       Dma[ StrIx ][ ProcType ].TxChannel.Nbr,
                       NbrBytes
                      );

  Dma[ StrIx ][ ProcType ].Periph.clr_flag_tc_fnct( Dma[ StrIx ][ ProcType ].Periph.Nbr );
  LL_DMA_EnableChannel( Dma[ StrIx ][ ProcType ].Periph.Nbr, Dma[ StrIx ][ ProcType ].TxChannel.Nbr );
}

void TModbusLink::stop_receive()
{  
//  if ( ProcType == TProcType::__SLAVE ) Do.open();
  dis_eob_detect();                                              //отключение RTO, запрет прерываний
  LL_DMA_DisableChannel(                                         //отключение приемного DMA - канала
                        Dma[ StrIx ][ ProcType ].Periph.Nbr,
                        Dma[ StrIx ][ ProcType ].RxChannel.Nbr
                       );
  IxSlice.Head = get_rx_dma_data_qty(); //голова буфера
  IxSlice.Tail = IxSlice.Head;          //хвост буфера
}

void TModbusLink::start_handle()
{
//  if ( ProcType == TProcType::__SLAVE ) Do.closed();
  en_eob_detect(  TModbusLink::_1_5_CH_BITS_NBR  );             //включить определение конца блока и прерываний USART
  LL_DMA_EnableChannel(                                         //включение приемного DMA - канала
                       Dma[ StrIx ][ ProcType ].Periph.Nbr,
                       Dma[ StrIx ][ ProcType ].RxChannel.Nbr
                      );
}

bool TModbusLink::handle_frame( const uint8_t *Buf, uint16_t BUF_SIZE )
{
//  constexpr uint16_t DLY_MS = 50U;
  TFrameState FrameState;
  
  if ( BUF_SIZE < 2U ) //защита на минимальный размер для подсчета CRC
  {
    FrameState = TFrameState::_FRAME_NOK;
  }
  else
  {
    FrameState = chk_frame( Buf, BUF_SIZE ); //проверка контрольной суммы пакета и адреса в пакете, \
                                               если они кривые, то ответ мастеру не отправляется
  }

  dis_eob_detect(); //отключение RTO, запрет прерываний
  vTaskDelay( pdMS_TO_TICKS( _2_ch_cnt() ) );
  
  if (
//      xSemaphoreTake( SlaveRtoTrigSem, DLY_MS ) == pdPASS
//      &&
      IxSlice.Head == get_rx_dma_data_qty()               //если новых символов за 3.5Ch принято не было 
      &&                                                  //и ...
      FrameState != TFrameState::_FRAME_NOK               //кадр ранее не был признан ошибочным
     )
  {
    //кадр можно обрабатывать \
      и отправлять ответ на запрос
//	  State = TState::__PROCESSING_REQUIRED_ACTION;
		
    if ( ProcType == TProcType::__SLAVE ) //для слейв-устройства следующий прием возможен только после отправки ответа \
                                            мастер на прием ответа ничего не отвечает, а только использует полученные данные
    {
      set_eof( StrIx, ProcType, true );
    }
		return true;
  }
  else
  {
    //EOF в true не устанавливается, т.к. на кривой пакет отвечать не надо
    stop_receive();                     //при приеме пакета произошли ошибки связи => пакет отбрасывается, ответ мастеру не возвращается
//    State = TState::__IDLE;
		
		return false;
  }
}

uint16_t TModbusLink::_2_ch_cnt()
{
  volatile uint16_t Numerator   = 22U;
  volatile uint16_t Denominator = ( static_cast<uint16_t>(Sets.BaudRate) + 1000U / 2 ) / 1000U;
  volatile uint16_t Res         = Numerator / Denominator;

  return ( Res < 1U ) ? 1U
                      : Res;
}

TModbusLink::TFrameState TModbusLink::chk_frame( const uint8_t *Buf, uint16_t BUF_SIZE )
{  
  const uint16_t MSG_SIZE = BUF_SIZE - 2U;
  uint16_t Crc16 = CRC16( Buf, MSG_SIZE ); //контрольная сумма в принятом пакете \
                                             расчитывается без учета 2-х последних байтов \
                                             В любом Modbus RTU сообщении 2 последних байта - CRC
  union TCRC
  {
    struct
    {
      uint8_t Low;
      uint8_t High;     
    };
    uint16_t Crc;
  };
  
  uint16_t BufCrc = (( TCRC * )&Buf[ MSG_SIZE ])->Crc;
    
  return (
          Crc16 == BufCrc
          &&
          (
           Buf[0] == Addr //проверка поля с адресом. В любом Modbus RTU сообщении 1-ый байт - Address
           ||
           Buf[0] == 0U //нулевой адрес для работы с программой "Настройка устройств Modbus" \
                          для работы с другими мастерами так делать ЗАПРЕЩЕНО протоколом
//           ||
//           (
//            Buf[0] == 4U   //тестовый код (под СЕНС ПД)
//            &&
//            ProcType == TProcType::__MASTER
//           )
          )
         )
         ? TFrameState::_FRAME_OK   //кадр может быть обработан
         : TFrameState::_FRAME_NOK; //кадр отбрасывается
}
//-----------------------------------------------


//void TModbusLink::init_tmr()
//{
//  if ( Tmr != TIM6 )
//  {
//  
//  }
//  else
//  {
//    LL_TIM_InitTypeDef TIM_InitStruct;

////  ----- Включить тактирование таймера ---------------------------------
//    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM6 ); //F3_RM, Rev 8, c.126: If(APB1 prescaler = 1) x1 else x2 => \
//                                                            => TIM7 тактируется частотой 72МГц

//    LL_RCC_ClocksTypeDef RCC_Clocks;
//    LL_RCC_GetSystemClocksFreq(&RCC_Clocks);    

//    constexpr uint32_t F_CNT             = 200000U; //частота работы счетчика таймера
//    const     uint32_t F_TIMER           = ( ( 1.0f + ( (uint16_t)Sets.BaudRate / 2U ) ) / (uint16_t)Sets.BaudRate ) * _1_5_CH_BITS_NBR; //частота работы таймера
//    uint16_t  PRESCALER_VALUE            = (RCC_Clocks.PCLK1_Frequency + (F_CNT / 2U)) / F_CNT - 1U;
//    const     uint32_t AUTORELOAD_VALUE  = (F_CNT + (F_TIMER / 2U)) / F_TIMER - 1U;
//    constexpr uint32_t START_TIMER_VALUE = 0x00;

////  ----- Инициализация таймера -----------------------------------------
//    TIM_InitStruct.Prescaler     = PRESCALER_VALUE;
//    TIM_InitStruct.CounterMode   = LL_TIM_COUNTERMODE_UP;
//    TIM_InitStruct.Autoreload    = AUTORELOAD_VALUE;
//    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
////    TIM_InitStruct.RepetitionCounter = ;

//    LL_TIM_Init(Tmr, &TIM_InitStruct);                            // Configure the TIMx time base unit
//    LL_TIM_ClearFlag_UPDATE(Tmr);

////    LL_TIM_DisableUpdateEvent(MAIN_TIMER);                        // Enable update event generation
//    LL_TIM_SetUpdateSource(Tmr, LL_TIM_UPDATESOURCE_COUNTER);     // Set update event source
//    LL_TIM_SetOnePulseMode(Tmr, LL_TIM_ONEPULSEMODE_REPETITIVE);  // Set one pulse mode
//    LL_TIM_SetCounterMode(Tmr, LL_TIM_COUNTERMODE_UP);            // Set the timer counter counting mode
//    LL_TIM_DisableARRPreload(Tmr);                                // Enable auto-reload (ARR) preload
//    LL_TIM_SetCounter(Tmr, START_TIMER_VALUE);                    // Set the counter value

//    //настройка NVIC
//    NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));  // 5 - максимальный уровень приоритета для прерывания из которого можно вызывать API функции FreeRTOS 
//    NVIC_EnableIRQ(TIM6_DAC_IRQn);
//  }
//}

void TModbusLink::init_dma()
{	
  TChannelSets Rx =
  {
    (uint32_t)&HW.If->RDR,
    (uint32_t)RxBuf,
    LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
    LL_DMA_MODE_CIRCULAR,
    RX_BUF_SIZE,
  };
  
  TChannelSets Tx =
  {
    (uint32_t)&HW.If->TDR,
    (uint32_t)TxBuf,
    LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
    LL_DMA_MODE_NORMAL,
    TX_BUF_SIZE,
  };
  
  Dma[ StrIx ][ ProcType ].Periph.en_clk( Dma[ StrIx ][ ProcType ].Periph.ClkMask );

  LL_DMA_InitTypeDef DMA_Init;
  
	DMA_Init.PeriphOrM2MSrcAddress  = Tx.PeriphAddr;             //базовый адрес источника
	DMA_Init.MemoryOrM2MDstAddress  = Tx.MemAddr;                //базовый адрес места назначения
	DMA_Init.Direction              = Tx.Direction;              //направление трансфера
	DMA_Init.Mode                   = Tx.Mode;                   //режим работы DMA (нормальный или кольцевой)
	DMA_Init.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT; //инкремент блоков источника данных (периферии или ОЗУ)
	DMA_Init.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;   //инкремент блоков места назначения данных (периферии или ОЗУ)
	DMA_Init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;    //выравнивание данных источника данных (периферии или ОЗУ)
	DMA_Init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;    //выравнивание данных места назначения данных (периферии или ОЗУ)
	DMA_Init.NbData                 = Tx.NbData;                 //количество блоков данных для обмена - перезаписывается под актуальный буфер
	DMA_Init.PeriphRequest          = LL_DMA_REQUEST_2;          //запрос периферии
	DMA_Init.Priority               = LL_DMA_PRIORITY_MEDIUM;  //приоритет DMA канала
  
  do { } while ( 
                LL_DMA_Init( 
                            Dma[ StrIx ][ ProcType ].Periph.Nbr,
                            Dma[ StrIx ][ ProcType ].TxChannel.Nbr,
                            &DMA_Init
                           ) != SUCCESS           
               );
               
	DMA_Init.PeriphOrM2MSrcAddress  = Rx.PeriphAddr;    //базовый адрес источника
	DMA_Init.MemoryOrM2MDstAddress  = Rx.MemAddr;       //базовый адрес места назначения
	DMA_Init.Direction              = Rx.Direction;     //направление трансфера
	DMA_Init.Mode                   = Rx.Mode;          //режим работы DMA (нормальный или кольцевой)
	DMA_Init.NbData                 = Rx.NbData;        //количество блоков данных для обмена
	DMA_Init.PeriphRequest          = LL_DMA_REQUEST_2; //запрос периферии
  
  do { } while ( 
                LL_DMA_Init( 
                            Dma[ StrIx ][ ProcType ].Periph.Nbr,
                            Dma[ StrIx ][ ProcType ].RxChannel.Nbr,
                            &DMA_Init
                           ) != SUCCESS           
               );

  //настройка NVIC
  NVIC_SetPriority( 
                   Dma[ StrIx ][ ProcType ].TxChannel.IRQ, 
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0) 
                  );                                                     //5 - максимальный уровень приоритета для прерывания \
                                                                           из которого можно вызывать API функции FreeRTOS 
  NVIC_EnableIRQ( Dma[ StrIx ][ ProcType ].TxChannel.IRQ );            //прерывания при передаче через USART
  
	LL_DMA_EnableIT_TC(
                     Dma[ StrIx ][ ProcType ].Periph.Nbr, 
                     Dma[ StrIx ][ ProcType ].TxChannel.Nbr 
                    );                                                   //Transfer complete interrupt
    
  LL_USART_EnableDMAReq_RX( HW.If );                                     //включение DMA режима для приема через USART
  LL_USART_EnableDMAReq_TX( HW.If );                                     //включение DMA режима для передачи через USART

	//активация DMA каналов
  LL_DMA_EnableChannel( Dma[ StrIx ][ ProcType ].Periph.Nbr, Dma[ StrIx ][ ProcType ].RxChannel.Nbr ); //канал включен => канал контроллера DMA может \
                                                                                                         обработать запрос от периферии, \
	                                                                                                       подключенной к этому каналу
	LL_DMA_DisableChannel( Dma[ StrIx ][ ProcType ].Periph.Nbr, Dma[ StrIx ][ ProcType ].TxChannel.Nbr );
	
}

uint16_t TModbusLink::get_rx_dma_data_qty()
{
  CNDTR = LL_DMA_GetDataLength(
                               Dma[ StrIx ][ ProcType ].Periph.Nbr,
                               Dma[ StrIx ][ ProcType ].RxChannel.Nbr
                              );
  
  return ( 
          sizeof RxBuf - CNDTR
         );
}

///* Table of CRC values for high–order byte */
//static unsigned char auchCRCHi[] = 
//{
//  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
//  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
//  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
//  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
//  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
//  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
//  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
//  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
//  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
//  0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
//  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
//  0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
//  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
//  0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
//  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
//  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
//  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
//  0x40
//}; 

///* Table of CRC values for low–order byte */ 
//static char auchCRCLo[] = 
//{
//  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
//  0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
//  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
//  0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
//  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
//  0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
//  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
//  0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
//  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
//  0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
//  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
//  0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
//  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
//  0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
//  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
//  0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
//  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
//  0x40
//};

//unsigned short TModbusLink::CRC16( 
//                                 const unsigned char *puchMsg, 
//                                 unsigned short usDataLen 
//                                )                         /* The function returns the CRC as a unsigned short type */
//{                                                         
//  //puchMsg;                                              /* message to calculate CRC upon */
//  //usDataLen;                                            /* quantity of bytes in message */
//  unsigned char uchCRCHi = 0xFF;                          /* high byte of CRC initialized */
//  unsigned char uchCRCLo = 0xFF;                          /* low byte of CRC initialized */
//  unsigned uIndex;                                        /* will index into CRC lookup table */
//                                                          
//  while (usDataLen--)                                     /* pass through message buffer */
//  {                                                       
//    uIndex = uchCRCLo ^ *puchMsg++;                       /* calculate the CRC */
//    uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
//    uchCRCHi = auchCRCLo[uIndex] ;
//  }

//  return (uchCRCHi << 8 | uchCRCLo) ;
//}

void TModbusLink::set_addr( uint8_t _Addr )
{
  Addr = _Addr;
}

uint8_t TModbusLink::get_addr()
{
  return Addr;
}


//******************************************************************************
//  Обработчики прерываний
//******************************************************************************
//extern "C" void TIM6_DAC_IRQHandler(void)
//{
//  if ( LL_TIM_IsActiveFlag_UPDATE( TIM6 ) )
//  {
////		TmrHigherPriorityTaskWoken = pdFALSE;
////		if ( xSemaphoreGiveFromISR( Trig_1_5Ch_Sem, &TmrHigherPriorityTaskWoken ) == pdFAIL ) //отправить семафор окончания записи
////    {
////      //семафор уже был доступен, т.е. ранее отдан другой задачей или прерыванием
////    }  
////	  if ( TmrHigherPriorityTaskWoken == pdPASS )
////	  {
////       portYIELD_FROM_ISR( TmrHigherPriorityTaskWoken ); //принудительное переключение контекста для разблокировки задачи - обработчика
////	  }

//    LL_TIM_ClearFlag_UPDATE( TIM6 ); // Clear the update interrupt flag (UIF)
//  }
//}

struct TUsartInt
{
  USART_TypeDef     *If;
  BaseType_t        *RTOHigherPriorityTaskWoken;
  BaseType_t        *CommErrHigherPriorityTaskWoken;
  SemaphoreHandle_t *RtoTrigSem;
  SemaphoreHandle_t *CommErrSem;
};

static void usart_int_handler( TUsartInt * );

extern "C" void USART1_IRQHandler(void)
{
  TUsartInt UsartInt{
                     USART1,
                     &SlaveRTOHigherPriorityTaskWoken,
                     &SlaveCommErrHigherPriorityTaskWoken,
                     &SlaveRtoTrigSem,
                     &SlaveCommErrSem,
                    };
  usart_int_handler( &UsartInt );
}

extern "C" void USART2_IRQHandler(void)
{
  TUsartInt UsartInt{
                     USART2,
                     &MasterRTOHigherPriorityTaskWoken,
                     &MasterCommErrHigherPriorityTaskWoken,
                     &MasterRtoTrigSem,
                     &MasterCommErrSem,
                    };
  usart_int_handler( &UsartInt ); 
}

extern "C" void DMA1_Channel7_IRQHandler(void)
{
  if ( LL_DMA_IsActiveFlag_TC7( DMA1 ) )             //проверка флага transfer complete
  {
    LL_DMA_ClearFlag_TC7( DMA1 );
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_7 );
  }
  set_eof( DMA1, LL_DMA_CHANNEL_7, false );
}

extern "C" void DMA2_Channel6_IRQHandler(void)
{
  if ( LL_DMA_IsActiveFlag_TC6( DMA2 ) )             //проверка флага transfer complete
  {
    LL_DMA_ClearFlag_TC6( DMA2 );
    LL_DMA_DisableChannel( DMA2, LL_DMA_CHANNEL_6 );
    
//    Do.open();
  }
  set_eof( DMA2, LL_DMA_CHANNEL_6, false );
}

extern "C" void DMA1_Channel4_IRQHandler(void)
{
  if ( LL_DMA_IsActiveFlag_TC4( DMA1 ) )             //проверка флага transfer complete
  {
    LL_DMA_ClearFlag_TC4( DMA1 );
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_4 );
  }
  set_eof( DMA1, LL_DMA_CHANNEL_4, false );
//  Do.closed();
}

//----- Вспомогательные функции обработчиков прерываний --------------------------------------------------
static void usart_int_handler( TUsartInt *UsartInt )
{
  if ( 
      LL_USART_IsActiveFlag_RTO( UsartInt->If )
      &&
      LL_USART_IsEnabledIT_RTO( UsartInt->If )     
     )                                    //прерывание по таймауту прихода байтов
  {    
//    Do.toggle();
		if ( UsartInt->If == USART1 )
    {
//      Do.closed();
    }
    
    *UsartInt->RTOHigherPriorityTaskWoken = pdFALSE;
		if ( xSemaphoreGiveFromISR( *UsartInt->RtoTrigSem, UsartInt->RTOHigherPriorityTaskWoken ) == pdFAIL ) //отправить семафор окончания записи
    {
      //семафор уже был доступен, т.е. ранее отдан другой задачей или прерыванием
    }  
	  if ( *UsartInt->RTOHigherPriorityTaskWoken == pdPASS )
	  {
       portYIELD_FROM_ISR( *UsartInt->RTOHigherPriorityTaskWoken ); //принудительное переключение контекста для разблокировки задачи - обработчика
	  }
    
    LL_USART_ClearFlag_RTO( UsartInt->If ); 
  }
  
  if (
      ( 
       LL_USART_IsActiveFlag_PE( UsartInt->If )
       &&
       LL_USART_IsEnabledIT_PE( UsartInt->If )     
      )
      ||
      (
        LL_USART_IsEnabledIT_ERROR( UsartInt->If )
        &&
        (
         LL_USART_IsActiveFlag_NE( UsartInt->If )
         ||
         LL_USART_IsActiveFlag_ORE( UsartInt->If )
         ||
         LL_USART_IsActiveFlag_FE( UsartInt->If )
        )
      )
     )                                    //USART Parity Error Interrupt
  {
    //сбросить все флаги ошибок и выдать семафор ошибки связи через USART
    if ( UsartInt->If == USART1 )
    {
//      Do.closed();
//    Do.open();
//      Do.toggle();
    }
    
    LL_USART_ClearFlag_PE( UsartInt->If );
    LL_USART_ClearFlag_NE( UsartInt->If );
    LL_USART_ClearFlag_ORE( UsartInt->If );
    LL_USART_ClearFlag_FE( UsartInt->If );
    
		*UsartInt->CommErrHigherPriorityTaskWoken = pdFALSE;
		if ( xSemaphoreGiveFromISR( *UsartInt->CommErrSem, UsartInt->CommErrHigherPriorityTaskWoken ) == pdFAIL ) //отправить семафор окончания записи
    {
      //семафор уже был доступен, т.е. ранее отдан другой задачей или прерыванием
    }  
	  if ( *UsartInt->CommErrHigherPriorityTaskWoken == pdPASS )
	  {
       portYIELD_FROM_ISR( *UsartInt->CommErrHigherPriorityTaskWoken ); //принудительное переключение контекста для разблокировки задачи - обработчика
	  }
  }
}


bool get_eof( uint8_t StrIx, TModbusLink::TProcType ProcType )
{
  bool Flag = false;
  
  __disable_irq();
    Flag = Eof[ StrIx ][ ProcType ].Flag;
  __enable_irq();  
  
  return Flag;
}

void set_eof( uint8_t StrIx, TModbusLink::TProcType ProcType, bool NewVal ) //для вызова из методов класса
{
  __disable_irq(); 
    Eof[ StrIx ][ ProcType ].Flag = NewVal;
  __enable_irq();
}

static void set_eof( DMA_TypeDef *DMA, uint32_t TxChannel, bool NewVal ) //для вызова из обработчиков прерываний
{
  if ( 
      DMA2 == DMA 
      &&
      TxChannel == LL_DMA_CHANNEL_6
     )
  {
//     Do.closed();
  }
  
  __disable_irq();
  for ( auto &row : Eof )
  {
    for ( auto &item : row )
    {
      if (
          DMA == item.DMA
          &&
          TxChannel == item.TxChannel
         )
      {
        item.Flag = NewVal;
        
        goto EXIT;
      }
    }
  }
  
  EXIT: __enable_irq();
}
//--------------------------------------------------------------------------------------------------------
