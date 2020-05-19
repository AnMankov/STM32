#include "SPI_driver.h"
#include "dma_types.h"

TSPI::TSPI( 
           const TSpi_HW &_HW,
           TRate _Rate
          )
:
HW( _HW ),
Rate( _Rate )
{

}

TSPI::~TSPI()
{

}

void TSPI::pin_clk_config()
{
  /*тактирование GPIO, к которым подключены SDA и SCL и тактирование интерфейса SPI*/
  HW.SO.en_clk( HW.SO.ClkPortMask );
  HW.SI.en_clk( HW.SI.ClkPortMask );
  HW.SCK.en_clk( HW.SCK.ClkPortMask );
  HW.CS.en_clk( HW.CS.ClkPortMask );
  
  HW.Clk.en_periph( HW.Clk.PeriphMask );
  
  /* инициализация выводов с I2C */
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  do
  {
    GPIO_InitStruct.Pin        = HW.SO.Nbr;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_5;
  } while ( SUCCESS != LL_GPIO_Init( HW.SO.Gpio, &GPIO_InitStruct ) );
  
  do
  {
    GPIO_InitStruct.Pin        = HW.SI.Nbr;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  } while ( SUCCESS != LL_GPIO_Init( HW.SI.Gpio, &GPIO_InitStruct ) );
  
  do
  {
    GPIO_InitStruct.Pin        = HW.SCK.Nbr;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  } while ( SUCCESS != LL_GPIO_Init( HW.SCK.Gpio, &GPIO_InitStruct ) );
  
  do
  {
    GPIO_InitStruct.Pin        = HW.CS.Nbr;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  } while ( SUCCESS != LL_GPIO_Init( HW.CS.Gpio, &GPIO_InitStruct ) );
}

void TSPI::hw_init()
{
  LL_RCC_ClocksTypeDef RCC_Clocks;
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);

  uint32_t BrPresc[] =
  {
    LL_SPI_BAUDRATEPRESCALER_DIV256, //__MIN         
    LL_SPI_BAUDRATEPRESCALER_DIV2,   //__MAX         
    LL_SPI_BAUDRATEPRESCALER_DIV2,   //__PCLK_DIV_2  
    LL_SPI_BAUDRATEPRESCALER_DIV4,   //__PCLK_DIV_4  
    LL_SPI_BAUDRATEPRESCALER_DIV8,   //__PCLK_DIV_8  
    LL_SPI_BAUDRATEPRESCALER_DIV16,  //__PCLK_DIV_16 
    LL_SPI_BAUDRATEPRESCALER_DIV32,  //__PCLK_DIV_32 
    LL_SPI_BAUDRATEPRESCALER_DIV64,  //__PCLK_DIV_64 
    LL_SPI_BAUDRATEPRESCALER_DIV128, //__PCLK_DIV_128
    LL_SPI_BAUDRATEPRESCALER_DIV256, //__PCLK_DIV_256
  };
  
  LL_SPI_DeInit( HW.If );
  LL_SPI_InitTypeDef SPI_InitStruct;
  
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX; //Для Simplex и Full-duplex настройка не отличается \
                                                           неиспользуемый Spi-вывод, для Simplex, инициализировать как GPIO
  SPI_InitStruct.Mode              = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth         = LL_SPI_DATAWIDTH_8BIT;
//  SPI_InitStruct.ClockPolarity     = LL_SPI_POLARITY_HIGH;
//  SPI_InitStruct.ClockPhase        = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.ClockPolarity     = LL_SPI_POLARITY_HIGH; /*!< Clock to 0 when idle */
  SPI_InitStruct.ClockPhase        = LL_SPI_PHASE_2EDGE;   /*!< Second clock transition is the first data capture edge */
  SPI_InitStruct.NSS               = LL_SPI_NSS_SOFT;
//  SPI_InitStruct.NSS               = LL_SPI_NSS_HARD_OUTPUT;
  SPI_InitStruct.BaudRate          = BrPresc[ Rate ];
  SPI_InitStruct.BitOrder          = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE;
//  SPI_InitStruct.CRCPoly           = ;
  
  LL_SPI_Init( HW.If, &SPI_InitStruct );
  
  LL_SPI_SetRxFIFOThreshold( HW.If, LL_SPI_RX_FIFO_TH_QUARTER ); /*!< RXNE event is generated if FIFO level is greater than or equel to 1/4 (8-bit)  */
  
  // Настройка NVIC
  NVIC_SetPriority(
                   HW.IRQ,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)
                  );                                                     //обработчик вызывает API функцию RTOS => приоритет д.б. \
                                                                           логически ниже или равен, но численно больше или равен,\
                                                                           установленному  в макросе \
                                                                           configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
  NVIC_EnableIRQ( HW.IRQ);
  
  
//   dma_init();
  //procedure for enabling SPI
  //SPI включается и данные для отправки записаны в Tx буфер => мастер начинает обмен
  //как обработать DMA 26.3.11
  
//  Для мастера
//  1. MOSI и SCK запускаются только когда SPI включен.
//  Отключается SPI => уровни на выводах зависят от настроек GPIO
//  2. SCK запущен => BSY в активном состоянии
//  3. TXFIFO переполняется => TXE сбрасывается
//  4. TXDMAEN устанавливается => запускается процесс арбитража DMA
//  TXE прерывание генерируется после установки TXEIE
//  
//  5.Если все данные для отправки могут поместиться в TxFIFO, TCIF может 
//  быть поднят перед началом связи по шине SPI.
//  TCIF всегда поднимается перед завершение SPI транзакции.
//  
//  6. В режиме с упаковкой данных, события TxE и RxNE являются парными и
//  каждый доступ на чтение/запись в FIFO происходит по 16 бит, при этом
//  количество кадров является четным. 
  
  
  
  
}

void TSPI::dma_init()
{
  struct TSpiDma
  {
    SPI_TypeDef *If;
    TDmaDuplex   Dma;
  };
  
  TSpiDma SpiDma[] =
  {
    {
      SPI1,
      {
        { 
          DMA1,
          LL_AHB1_GRP1_EnableClock,
          LL_AHB1_GRP1_PERIPH_DMA1,
          LL_DMA_ClearFlag_TC3
        },                                                              //Nbr
	      { LL_DMA_CHANNEL_2, DMA1_Channel2_IRQn },                       //RxChannel
	      { LL_DMA_CHANNEL_3, DMA1_Channel3_IRQn }                        //TxChannel
      }
    },
    {
      SPI1,
      {
        { 
          DMA2,
          LL_AHB1_GRP1_EnableClock,
          LL_AHB1_GRP1_PERIPH_DMA2,
          LL_DMA_ClearFlag_TC4
        },                                                              //Nbr
	      { LL_DMA_CHANNEL_3, DMA1_Channel3_IRQn },                       //RxChannel
	      { LL_DMA_CHANNEL_4, DMA1_Channel4_IRQn }                        //TxChannel
      }
    },
    {
      SPI2,
      {
        { 
          DMA1,
          LL_AHB1_GRP1_EnableClock,
          LL_AHB1_GRP1_PERIPH_DMA1,
          LL_DMA_ClearFlag_TC5
        },                                                              //Nbr
	      { LL_DMA_CHANNEL_4, DMA1_Channel4_IRQn },                       //RxChannel
	      { LL_DMA_CHANNEL_5, DMA1_Channel5_IRQn }                        //TxChannel
      }
    },
    {
      SPI3,
      {
        { 
          DMA2,
          LL_AHB1_GRP1_EnableClock,
          LL_AHB1_GRP1_PERIPH_DMA2,
          LL_DMA_ClearFlag_TC2
        },                                                              //Nbr
	      { LL_DMA_CHANNEL_1, DMA1_Channel1_IRQn },                       //RxChannel
	      { LL_DMA_CHANNEL_2, DMA1_Channel2_IRQn }                        //TxChannel
      }
    },
  };
  
//  HW.If->
//  
//  TChannelSets Rx =
//  {
//    (uint32_t)&HW.If->RDR,
//    (uint32_t)RxBuf,
//    LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
//    LL_DMA_MODE_CIRCULAR,
//    RX_BUF_SIZE,
//  };
//  
//  TChannelSets Tx =
//  {
//    (uint32_t)&HW.If->TDR,
//    (uint32_t)TxBuf,
//    LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
//    LL_DMA_MODE_NORMAL,
//    TX_BUF_SIZE,
//  };
//  
//  Dma[ StrIx ][ ProcType ].Periph.en_clk( Dma[ StrIx ][ ProcType ].Periph.ClkMask );

//  LL_DMA_InitTypeDef DMA_Init;
//  
//	DMA_Init.PeriphOrM2MSrcAddress  = Tx.PeriphAddr;             //базовый адрес источника
//	DMA_Init.MemoryOrM2MDstAddress  = Tx.MemAddr;                //базовый адрес места назначения
//	DMA_Init.Direction              = Tx.Direction;              //направление трансфера
//	DMA_Init.Mode                   = Tx.Mode;                   //режим работы DMA (нормальный или кольцевой)
//	DMA_Init.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT; //инкремент блоков источника данных (периферии или ОЗУ)
//	DMA_Init.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;   //инкремент блоков места назначения данных (периферии или ОЗУ)
//	DMA_Init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;    //выравнивание данных источника данных (периферии или ОЗУ)
//	DMA_Init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;    //выравнивание данных места назначения данных (периферии или ОЗУ)
//	DMA_Init.NbData                 = Tx.NbData;                 //количество блоков данных для обмена - перезаписывается под актуальный буфер
//	DMA_Init.PeriphRequest          = LL_DMA_REQUEST_2;          //запрос периферии
//	DMA_Init.Priority               = LL_DMA_PRIORITY_MEDIUM;  //приоритет DMA канала
//  
//  do { } while ( 
//                LL_DMA_Init( 
//                            Dma[ StrIx ][ ProcType ].Periph.Nbr,
//                            Dma[ StrIx ][ ProcType ].TxChannel.Nbr,
//                            &DMA_Init
//                           ) != SUCCESS           
//               );
//               
//	DMA_Init.PeriphOrM2MSrcAddress  = Rx.PeriphAddr;    //базовый адрес источника
//	DMA_Init.MemoryOrM2MDstAddress  = Rx.MemAddr;       //базовый адрес места назначения
//	DMA_Init.Direction              = Rx.Direction;     //направление трансфера
//	DMA_Init.Mode                   = Rx.Mode;          //режим работы DMA (нормальный или кольцевой)
//	DMA_Init.NbData                 = Rx.NbData;        //количество блоков данных для обмена
//	DMA_Init.PeriphRequest          = LL_DMA_REQUEST_2; //запрос периферии
//  
//  do { } while ( 
//                LL_DMA_Init( 
//                            Dma[ StrIx ][ ProcType ].Periph.Nbr,
//                            Dma[ StrIx ][ ProcType ].RxChannel.Nbr,
//                            &DMA_Init
//                           ) != SUCCESS           
//               );

//  //настройка NVIC
//  NVIC_SetPriority( 
//                   Dma[ StrIx ][ ProcType ].TxChannel.IRQ, 
//                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0) 
//                  );                                                     //5 - максимальный уровень приоритета для прерывания \
//                                                                           из которого можно вызывать API функции FreeRTOS 
//  NVIC_EnableIRQ( Dma[ StrIx ][ ProcType ].TxChannel.IRQ );            //прерывания при передаче через USART
//  
//	LL_DMA_EnableIT_TC(
//                     Dma[ StrIx ][ ProcType ].Periph.Nbr, 
//                     Dma[ StrIx ][ ProcType ].TxChannel.Nbr 
//                    );                                                   //Transfer complete interrupt
//    
//  LL_USART_EnableDMAReq_RX( HW.If );                                     //включение DMA режима для приема через USART
//  LL_USART_EnableDMAReq_TX( HW.If );                                     //включение DMA режима для передачи через USART

//	//активация DMA каналов
//  LL_DMA_EnableChannel( Dma[ StrIx ][ ProcType ].Periph.Nbr, Dma[ StrIx ][ ProcType ].RxChannel.Nbr ); //канал включен => канал контроллера DMA может \
//                                                                                                         обработать запрос от периферии, \
//	                                                                                                       подключенной к этому каналу
//	LL_DMA_DisableChannel( Dma[ StrIx ][ ProcType ].Periph.Nbr, Dma[ StrIx ][ ProcType ].TxChannel.Nbr );
  
}
  
void TSPI::write_frame( const uint8_t *Buf, uint8_t Size )
{

}

void TSPI::read_frame()
{

}

TSPI::TSpiState TSPI::write_byte( uint8_t Byte )
{
  if ( check_busy() != TSpiState::__FREE)
  {
    return TSpiState::__BSY;
  }
    
  LL_GPIO_ResetOutputPin( HW.CS.Gpio, HW.CS.Nbr );
//  LL_DMA_DisableChannel( DMA.Nbr, DMA.TxCh ); //для передачи одного байта DMA не используется

  LL_SPI_TransmitData8( HW.If, Byte);
  LL_SPI_Enable( HW.If );                              //Enable SPI peripheral \
                                                         на выводе NSS уровень, если включено аппаратное управление 

  LL_SPI_EnableIT_TXE( HW.If ); //включить прерывание TXE

  return TSpiState::__START;
}

TSPI::TSpiState TSPI::check_busy()
{
  if (                                            //проверка наличия транзакции SPI
      LL_SPI_IsActiveFlag_BSY( HW.If )
      ||
      !LL_SPI_IsActiveFlag_TXE( HW.If )
     )
  {
    return TSpiState::__BSY;                      //SPI занят => передача нового кадра не запускается
  }

  return (
          LL_SPI_IsActiveFlag_BSY( HW.If )
         )
         ? TSpiState::__BSY
         : TSpiState::__FREE;
}
