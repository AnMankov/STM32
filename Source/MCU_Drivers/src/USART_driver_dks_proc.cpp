#include "main.h"
#include "USART_driver.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_rcc.h"
#include "dks_protocol.h"

volatile static const uint8_t *TxBufPtr;       //буфер передачи
volatile static uint8_t TxBufSize;             //размер буфера передачи
volatile static TProc   TxProc;                //протокол передачи
volatile static uint8_t TxCrc;                 //контрольная сумма процесса передачи 
volatile static dks_tx_proc::TTxPhase TxPhase; //фаза процесса передачи
volatile static uint8_t TxByteCtr;             //счетчик перданных байтов данных
static BaseType_t TxHigherPriorityTaskWoken;

//static uint8_t *RxBufPtr;             //буфер приема
//static uint8_t RxBufSize;             //размер буфера приема
//static TProc   RxProc;                //протокол приема
//static uint8_t RxCrc;                 //контрольная сумма процесса приема
//static dks_rx_proc::TRxPhase RxPhase; //фаза процесса приема
//static uint8_t RxByteCtr;             //счетчик принятых байтов данных
//static BaseType_t RxHigherPriorityTaskWoken;

static uint32_t USARTClockFreq;
static uint32_t USARTClockSource;

extern bool TxEndFlag;

namespace USART
{
  namespace
  {
    //Модифицировать константу под схемотехнику
    const TUSARTHw Gpio[] =
    {
      {
	    USART1,
        GPIOA,
        GPIOA,
        GPIOA,
        LL_AHB1_GRP1_PERIPH_GPIOA,
        LL_RCC_USART1_CLKSOURCE_SYSCLK,
        LL_RCC_USART1_CLKSOURCE,
        LL_APB2_GRP1_PERIPH_USART1,
        LL_GPIO_PIN_9,
        LL_GPIO_PIN_10,
        LL_GPIO_PIN_12,
        "USART1"
      },
      {
	    USART2,
        GPIOA,
        GPIOA,
        GPIOA,
        LL_AHB1_GRP1_PERIPH_GPIOA, 
        LL_RCC_USART2_CLKSOURCE_SYSCLK,
        LL_RCC_USART2_CLKSOURCE, 
        LL_APB1_GRP1_PERIPH_USART2,
        LL_GPIO_PIN_2,
        LL_GPIO_PIN_3,
        LL_GPIO_PIN_1, 
        "USART2"
      },
      {
	    USART3,
        GPIOB,
        GPIOB,
        GPIOB,
        LL_AHB1_GRP1_PERIPH_GPIOB,
        LL_RCC_USART3_CLKSOURCE_PCLK1,
        LL_RCC_USART3_CLKSOURCE,
        LL_APB1_GRP1_PERIPH_USART3,
        LL_GPIO_PIN_10,
        LL_GPIO_PIN_11,
        LL_GPIO_PIN_14,
        "USART3"
      }
    };
  }

  constexpr uint32_t MAX_DEDT    = 31;
  constexpr uint32_t MAX_DEAT    = 31;
  constexpr uint8_t SYM_BITS_NUM = 11U;
  constexpr uint8_t SYN_NUM      =  2U;
}

namespace USART
{
  TUSART::TUSART(
                 USART_TypeDef *USARTx,
				         TMode _Mode, 
				         TProc _Proc,
				         const TSet &_Set
				        )
  :
  Proc(_Proc),
  Mode(_Mode), 
  Set(_Set)
  {
	  for (auto item : Gpio)
	  {
	    if (USARTx == item.USARTx)
      {
		    USARTHw = item;
		    break;
		  }		
	  }
	  u.ustate = TUsartState::U_IDLE;

    bpphead = 1;
	  bpptail = 1;
    uerr    = 0;
  }

  TUSART::~TUSART()	
  {

  }

  void TUSART::pin_clk_config()
  {
    /*тактирование GPIO и USART*/
    if (!LL_AHB1_GRP1_IsEnabledClock(USARTHw.PinClkMask))    //включение тактирования GPIO для порта с USART, если не включено
    {
      LL_AHB1_GRP1_EnableClock(USARTHw.PinClkMask);
    }
    if (!LL_APB1_GRP1_IsEnabledClock(USARTHw.PeriphClkMask)) //включение тактирования от шины APB1 для USART, если не включено
    {
      LL_APB1_GRP1_EnableClock(USARTHw.PeriphClkMask);
    }
    
    /* инициализация выводов с USART */
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    do
    {
      GPIO_InitStruct.Pin        = USARTHw.TxPinMask;
      GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
      GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
      GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
      GPIO_InitStruct.Alternate  = LL_GPIO_AF_7;
    } while ( SUCCESS != LL_GPIO_Init(USARTHw.GpioTX, &GPIO_InitStruct) );
	 do
	 {
      GPIO_InitStruct.Pin = USARTHw.RxPinMask;
	 } while ( SUCCESS != LL_GPIO_Init(USARTHw.GpioRX, &GPIO_InitStruct) );
#ifndef __DEBUG__	 
	 do
	 {
      GPIO_InitStruct.Pin = USARTHw.DEPinMask;
	 } while ( SUCCESS != LL_GPIO_Init(USARTHw.GpioDE, &GPIO_InitStruct) );
#endif

	 LL_RCC_SetUSARTClockSource(USARTHw.ClkSrc);	 //The clock source must be chosen before enabling the USART \
	                                                (by setting the UE bit)
  }

  void TUSART::usart_hw_init()
  {
	  LL_USART_InitTypeDef USART_InitStruct;
    
	  do
	  {
	    USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;
//	    USART_InitStruct.BaudRate            = ;
	    USART_InitStruct.DataWidth           = LL_USART_DATAWIDTH_8B;
	    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	    USART_InitStruct.Parity              = (uint32_t)Set.Parity;     //значение из данных объекта
	    USART_InitStruct.StopBits            = (uint32_t)Set.Stops;      //значение из данных объекта
	    USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX; //Transmitter and Receiver are enabled
	  } while (SUCCESS != LL_USART_Init(USARTHw.USARTx, &USART_InitStruct));
    
	  uint32_t Fck = LL_RCC_GetUSARTClockFreq(USARTHw.CheckClkSrc);
	  uint32_t OvS = LL_USART_GetOverSampling(USARTHw.USARTx);
    
	  if (IS_UART_DRIVER_ENABLE_INSTANCE(USARTHw.USARTx))
	  {
       switch (Mode)
	    {
	      case TMode::_RS485_DE:
              switch (get_config())
	           {
	             case (ADM2587E::TConfigurable::_HALF_DUPLEX):
	  	              usart_set_485_hd(Fck, OvS);
	    	           break;
	             case (ADM2587E::TConfigurable::_FULL_DUPLEX):
	    	      	  break;
	             default:
	                  break;
	           }     
	  	       break;
	  	  case TMode::_STANDARD:
              usart_set_232(Fck, OvS);
	  	       break;
	      default:  //реализация остальных режимов - по необходимости
	  	       break;
	    }
	  }
	  else
	  {
	    //какие-то действия,  если данный экземпляр USART'a не поддерживает Driver Enable
		 //реализация по мере необходимости
	  }
	  
	  LL_USART_SetTransferBitOrder(USARTHw.USARTx, LL_USART_BITORDER_LSBFIRST);
	  
	  usart_set_interrupt(); //настроить прерывания
	  
	  switch (Proc)
	  {
	    case TProc::_AUTO:
		      break;
	    case TProc::_MODBUS:
//	 	     break;
	    case TProc::_SENS:
		      sens_set();
		      break;
	    case TProc::_DKS_TO_PC:
		      break;
	    case TProc::_OMNICOMM:
		      break;
	    default:
		      break;
	  }
	   
	  LL_USART_Enable(USARTHw.USARTx);
  }
  
  void TUSART::sens_set()
  {
    sens_timer_set(TIM6, TIM6_DAC_IRQn);    
	  sens_usart_set();
	  sens_dma_set();
  }
  
  void TUSART::sens_dma_set()
  {
    enum TModuleNum : uint8_t
	  {
	    _ONE = 0,
		  _TWO = 1
	  };
	 
	 struct TDMA
	 {
	   DMA_TypeDef *DmaNum;
		 uint32_t ClkMask;
	 };
	 
	 constexpr TDMA DMA_HW[] =           //константы для описания аппаратной части контроллеров DMA
	 {
	   {DMA1, LL_AHB1_GRP1_PERIPH_DMA1}, //для второго контроллера описать по необходимости
	 };
	 
	 constexpr DMA_TypeDef *DmaNum = DMA_HW[_ONE].DmaNum;
	 constexpr uint32_t ClkMask    = DMA_HW[_ONE].ClkMask; 
	 
	 
	 if ( !LL_AHB1_GRP1_IsEnabledClock(ClkMask) )
	 {
	   LL_AHB1_GRP1_EnableClock(ClkMask);
	 }
	 
	 /*
	  * Конфигурация DMA - канала для передачи через USART 
	 */
	 LL_DMA_InitTypeDef DMA_InitStruct;
	 
	 DMA_InitStruct.PeriphOrM2MSrcAddress  = (uint32_t)&USARTHw.USARTx->TDR;    //установка адреса регистра периферии для манипуляций с памятью по событию от периферии
	 DMA_InitStruct.MemoryOrM2MDstAddress  = (uint32_t)&Usart.u.trbuf;          //базовый адрес ОЗУ
	 DMA_InitStruct.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	 DMA_InitStruct.Mode                   = LL_DMA_MODE_NORMAL;
	 DMA_InitStruct.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT; //режим инкрементирования адреса источника - периферии
	 DMA_InitStruct.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;   //режим инкрементирования адреса места назначения - в ОЗУ
	 DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;    //количество байт для единицы данных периферии
	 DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;    //количество байт для единицы данных ОЗУ
	 DMA_InitStruct.NbData                 = USART_TXBUFSIZE;           //количество единиц данных (байт, полуслов, слов в зависимости от настроек выше) для перемещений
	 DMA_InitStruct.Priority               = LL_DMA_PRIORITY_MEDIUM;    //уровень приоритета канала
	 
	 do
	 {
	 
	 } while ( SUCCESS != LL_DMA_Init(DmaNum, LL_DMA_CHANNEL_7, &DMA_InitStruct) );

	 /*
	  * Конфигурация DMA - канала для приема через USART
	 */
	 DMA_InitStruct.PeriphOrM2MSrcAddress  = (uint32_t)&USARTHw.USARTx->RDR;    //установка адреса регистра периферии для манипуляций с памятью по событию от периферии
	 DMA_InitStruct.MemoryOrM2MDstAddress  = (uint32_t)&Usart.inbuffer;         //базовый адрес ОЗУ
	 DMA_InitStruct.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	 DMA_InitStruct.Mode                   = LL_DMA_MODE_CIRCULAR;
	 DMA_InitStruct.NbData                 = INBUFFER_SIZE;                     //количество единиц данных (байт, полуслов, слов в зависимости от настроек выше) для перемещений
    
	 do
	 {
	 
	 } while ( SUCCESS != LL_DMA_Init(DmaNum, LL_DMA_CHANNEL_6, &DMA_InitStruct) );
    
    //настройка NVIC
    NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)); // 5 - максимальный уровень приоритета для прерывания из которого можно вызывать API функции FreeRTOS 
    NVIC_EnableIRQ(DMA1_Channel7_IRQn); //прерывания при передаче через USART
    
	 LL_DMA_EnableIT_TC(DmaNum, LL_DMA_CHANNEL_7); //Transfer complete interrupt
    
	 //DMA режим для передачи
    //данные загружаются в периферию (TDR) из ОЗУ когда устанавливается бит TXE
    
	 LL_USART_EnableDMAReq_RX(USARTHw.USARTx); //включение DMA режима для приема через USART
	 LL_USART_EnableDMAReq_TX(USARTHw.USARTx); //включение DMA режима для передачи через USART

	 //активация DMA каналов
	 if ( !LL_DMA_IsEnabledChannel(DmaNum, LL_DMA_CHANNEL_6) )
	 {
      LL_DMA_EnableChannel(DmaNum, LL_DMA_CHANNEL_6); //канал включен => канал контроллера DMA может обработать запрос от периферии, \
	                                                     подключенной к этому каналу
	 }
	 if (LL_DMA_IsEnabledChannel(DmaNum, LL_DMA_CHANNEL_7))
	 {
	   LL_DMA_DisableChannel(DmaNum, LL_DMA_CHANNEL_7);
	 }
  }
  
  void TUSART::sens_usart_set()
  {
	 if ( !LL_USART_IsEnabledRxTimeout(USARTHw.USARTx) )
	 {
	   LL_USART_EnableRxTimeout(USARTHw.USARTx); //включить функцию определения конца блока по программируемому таймауту
	 }
	 
	 LL_USART_SetRxTimeout(USARTHw.USARTx, SYM_BITS_NUM * SYN_NUM); //установка величины таймаута в битах принимаемого символа
	 
	 LL_USART_ClearFlag_RTO(USARTHw.USARTx);
	 LL_USART_EnableIT_RTO(USARTHw.USARTx);   //разрешение прерывания по RTO
	 LL_USART_EnableIT_PE(USARTHw.USARTx);    //разрешение прерывания по PE
	 LL_USART_EnableIT_ERROR(USARTHw.USARTx); //разрешение прерываний: framing error, overrun error or noise flag
  }
  
  void TUSART::sens_timer_set(TIM_TypeDef *TimProtocol, IRQn_Type TimIRQn)
  {
	 
    /*
	  * TIM6 - таймер генерации паузы перед отправкой пакета (пауза в 2 байта)
     */
    //настройка NVIC
    NVIC_SetPriority(TimIRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));  // 5 - максимальный уровень приоритета для прерывания из которого можно вызывать API функции FreeRTOS 
    NVIC_EnableIRQ(TimIRQn);
    
	 if (!LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_TIM6)) //включение тактирования от шины APB1 для TIM6, если не включено
    {
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
    }
	 
	 do 
	 {
	 
	 } while ( SUCCESS != LL_TIM_DeInit(TimProtocol) );
	 
	 LL_TIM_InitTypeDef TIM_InitStruct;
	 
	 LL_TIM_StructInit(&TIM_InitStruct);      //записать в поля структуры конфигурацию таймера по умолчанию	 
	 LL_RCC_ClocksTypeDef RCC_Clocks;         //частоты различных шин тактирования [Гц]
	 LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
	 
	 uint32_t TimClk = RCC_Clocks.PCLK1_Frequency;
	 if (RCC_Clocks.HCLK_Frequency != RCC_Clocks.PCLK1_Frequency)
	 {
      TimClk *= 2U; //RM0316, Rev 8: if (APB1 prescaler = 1) x1 else x2
	 }
	 
    /*
	  * Для TIM6 максимальное значение Autoreload = 0xFFFF (65535)
	  * Диапазон поддерживаемых устройством значений скорости: 1200..115200 бит/сек
	  * Пауза перед отправкой пакета 2 байта (22бита) - из ПМП201
	  * Так как частота тактирования таймера в МГц, необходимо снижать 
	  * частоту внутреннего счетчика до кГц, чтобы уложиться в разрядность Autoreload для всего диапазона скоростей
	  * в тоже время частота не должна быть слишком низкой, т.к. требуется поддержать высокие скорости ]
	  * Пр. 115200 => 1бит за 8,68мкс; 22бита за 191мкс. Т.е. 1 такт таймера д.б. минимум в несколько раз меньше 191мкс
	 */	 
    constexpr uint16_t PRESCALER_VALUE = 0x3E8; //тики с частотой в 1000раз меньшей частоты тактирования TIM6
	 uint32_t CtrFreq                   = (TimClk + (PRESCALER_VALUE >> 1)) / PRESCALER_VALUE; //частота тактирования внутреннего счетчика в таймере [Гц] 
	 	 
	 TIM_InitStruct.Prescaler         = PRESCALER_VALUE - 1;
	 TIM_InitStruct.CounterMode       = LL_TIM_COUNTERMODE_UP;	 
	 TIM_InitStruct.Autoreload        = ( (CtrFreq * SYM_BITS_NUM * SYN_NUM + ((uint32_t)Set.BaudRate >> 1)) / (uint32_t)Set.BaudRate ) - 1; //количество периодов до автоперезагрузки счетчика таймера
	 TIM_InitStruct.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
//	 TIM_InitStruct.RepetitionCounter = ;

    LL_TIM_Init(TimProtocol, &TIM_InitStruct);
	 
	 LL_TIM_ClearFlag_UPDATE(TimProtocol);
    LL_TIM_EnableIT_UPDATE(TimProtocol);
	 
	 LL_TIM_DisableCounter(TimProtocol);
  }
  
  
  void TUSART::usart_set_485_hd(uint32_t Fck, uint32_t OvS)
  {   
	LL_USART_EnableDEMode(USARTHw.USARTx);
	LL_USART_SetBaudRate(USARTHw.USARTx, Fck, OvS, (uint32_t)Set.BaudRate); //значение из данных объекта
    LL_USART_SetDEDeassertionTime(USARTHw.USARTx, MAX_DEDT);
    LL_USART_SetDEAssertionTime(USARTHw.USARTx, MAX_DEAT);
    LL_USART_SetDESignalPolarity(USARTHw.USARTx, LL_USART_DE_POLARITY_HIGH);
  }
  
  void TUSART::usart_set_232(uint32_t Fck, uint32_t OvS)
  {
	 LL_USART_SetBaudRate(USARTHw.USARTx, Fck, OvS, (uint32_t)Set.BaudRate);
  }

  void TUSART::usart_set_interrupt()
  {
    //Настройка NVIC
    IRQn_Type IRQEvent = (USARTHw.USARTx == USART1) ? USART1_IRQn
	                                                 : (USARTHw.USARTx == USART2)
														            ? USART2_IRQn
														            : USART3_IRQn;

    NVIC_SetPriority(IRQEvent, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)); //обработчик вызывает API функцию RTOS => приоритет д.б. логически ниже или равен, \
	                                                                                      но численно больше или равен, установленнму в макросе configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    NVIC_EnableIRQ(IRQEvent);
  }

  void TUSART::tx_single_byte(uint8_t TxByte, TProc Proc)
  {
    tx_int_set();
	 //необходимо защитить ресурс при нахождении в процессе отправки байта
	 //с помощью флага или FreeRTOS
	 LL_USART_TransmitData8(USARTHw.USARTx, TxByte);
  }

  void TUSART::tx_burst(const uint8_t *DataBurst, uint8_t Size, TProc Proc)
  {
	uint8_t FirstByte;
    
	TxBufPtr  = DataBurst;
	TxBufSize = Size;
	TxProc    = Proc;
	TxCrc     = calc_crc(DataBurst, Size);
    
	tx_int_set();
	
	switch (TxProc)
    {
	  case TProc::_DKS_TO_PC:
	       FirstByte = dks_tx_proc::ID_BURST; //передача по протоколу DKS
	 	   TxPhase   = dks_tx_proc::_PH_ID_ACK;
	       break;
	  case TProc::_AUTO:
	       //реализация по мере необходимости
//			  break;
	  case TProc::_MODBUS:
	       //реализация по мере необходимости
//			  break;
	  case TProc::_SENS:
	       //реализация по мере необходимости
//			  break;
	  case TProc::_OMNICOMM:
	       //реализация по мере необходимости
//			  break;
	  default:
	       break;
	}
	
	LL_USART_TransmitData8(USARTHw.USARTx, FirstByte); //запуск передачи
    LL_USART_ClearFlag_TC(USARTHw.USARTx);             //по сбросу флаг установлен 
	LL_USART_EnableIT_TXE(USARTHw.USARTx);
  }

  void TUSART::tx_int_set()
  {
    LL_USART_DisableIT_RXNE(USARTHw.USARTx);
    LL_USART_DisableDirectionRx(USARTHw.USARTx); //Receiver Disable.
    
    LL_USART_ClearFlag_TC(USARTHw.USARTx);       //по сбросу флаг установлен 
    LL_USART_EnableDirectionTx(USARTHw.USARTx);  //Transmitter Enable \
                                                   отправляется idle кадр
  }

  void TUSART::rx_single_byte(uint8_t *RxByte)
  {
    rx_int_set();
    //необходимо защитить ресурс при нахождении в процессе приема байта
    //с помощью флага или FreeRTOS
  }

  void TUSART::rx_burst(uint8_t *DataBurst, uint8_t Size)
  {
    rx_int_set();
    //необходимо защитить ресурс при нахождении в процессе приема пакета байтов
    //с помощью флага или FreeRTOS
  }

  void TUSART::rx_int_set()
  {
    LL_USART_DisableIT_TXE(USARTHw.USARTx);
    LL_USART_DisableIT_TC(USARTHw.USARTx);
    LL_USART_DisableDirectionTx(USARTHw.USARTx); //Transmitter Disable.
    
    LL_USART_EnableIT_RXNE(USARTHw.USARTx);
    LL_USART_EnableDirectionRx(USARTHw.USARTx);  //Receiver Enable
  }

  uint8_t TUSART::calc_crc(const uint8_t *Add, uint8_t LenData)
  {
    uint8_t Sum = LenData;

    for (uint8_t BytesCtr = 0; BytesCtr < LenData; ++BytesCtr)
    {
      Sum ^= Add[BytesCtr];
    }

    return Sum;
  }

  bool TUSART::set_cmp(const TSet &_Set) const
  {
    return (Set.BaudRate == _Set.BaudRate) &&
           (Set.Parity   == _Set.Parity)   &&
           (Set.Stops    == _Set.Stops);
  }

  void TUSART::set_chng(const TSet &_Set)
  {
    Set = _Set;
  }

  void TUSART::hw_reinit()
  {
	 LL_USART_Disable(USARTHw.USARTx);
    
	 usart_hw_init();
  }
  void TUSART::reset_receive()
  {
    __disable_irq();
      u.ustate = TUsartState::U_IDLE;
      u.rscnt  = 0;
      uerr     = 0;
      if (bpptail != bpphead)
      {
        educt_pkg();
      }
	  __enable_irq();
  }
  
  void TUSART::educt_pkg()
  {
    uint16_t n;
	  uint16_t st;
	  uint16_t en;
    uint16_t bptm;
	 
    if (u.ustate == TUsartState::U_WT)
    {
	   return; //пакет ещё не обработан
	  }
    
	  n = 0;
	 
    do 
	  {
      if (bpptail == bpphead)
      {
	     return; //нет пакетов в буфере
	    }
		
      if (bpptail == 0)
      {
		  bptm = 15;
		  }
      else
      {
		  bptm = bpptail - 1;
		  }
		
      st    = bppoint[bptm];
      u.err = bpperr[bpptail];
      en    = bppoint[bpptail++];
      
		  if (bpptail >= 16)
      {
		  bpptail = 0;
		  }
    } while (st == en);
    
	  n = 0;
	 
    do 
	  {
      u.rsbuf[n++] = inbuffer[st++];
      if (st >= INBUFFER_SIZE)
      {
		  st = 0;
		  }
    } while ((st != en) && (n < USART_RXBUFSIZE));
    
	  u.rscnt  = n;
    u.ustate = TUsartState::U_WT;
  }
  
  void TUSART::transmit()
  {
    //нечего передавать
    if (!u.trcnt) 
    {
      reset_receive();	
      
		  return;
    }
    
    //если передача уже ведется - игнорируем вызов
    if (u.ustate == TUsartState::U_TX) 
    {
      return;
    }
    
    //запускаем передачу
    reset_receive();
    u.ustate = TUsartState::U_TX;
    
    //запрещаем прием
    //DMA_Cmd(DMA1_Channel6,DISABLE);
    //разрешаем передачу
	 LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, u.trcnt); //Set Number of data to transfer
	 LL_DMA_ClearFlag_TC7(DMA1);                            //Clear Channel 7 transfer complete flag
	 LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);          //Enable DMA channel
	 LL_USART_ClearFlag_TC(USART2);                         //Clear Transmission Complete Flag
	 LL_USART_EnableIT_TC(USART2);                          //Enable Transmission Complete Interrupt
  }
}


//******************************************************************************
// Функции, вызываемые из обработчиков прерываний
//******************************************************************************
__INLINE static void dks_txe_ok(void)
{
	 uint8_t TestLen;
	 TestLen = sizeof(TCanData);
	 ++TestLen;
	 
	 switch (TxPhase)
	 {
//	   case dks_tx_proc::_PH_ID_BURST:
//           RelFour.off();
//	        LL_USART_TransmitData8(USART2, dks_tx_proc::ID_BURST);
//			  TxPhase = dks_tx_proc::_PH_ID_ACK;
//		     break;
	   case dks_tx_proc::_PH_ID_ACK:
	        LL_USART_TransmitData8(USART2, dks_tx_proc::ID_ACK);
//           USART2->RDR = dks_tx_proc::ID_ACK;
			  TxPhase = dks_tx_proc::_PH_LEN_DATA;
		     break;
	   case dks_tx_proc::_PH_LEN_DATA:
	        LL_USART_TransmitData8(USART2, dks_tx_proc::LEN_DATA);
			  TxPhase = dks_tx_proc::_PH_DATA;
		     break;
	   case dks_tx_proc::_PH_DATA:
		     if (TxByteCtr == TxBufSize)
			  {
			    TxPhase = dks_tx_proc::_PH_CRC;
				 TxByteCtr = 0;
			    break;
			  }
	        LL_USART_TransmitData8(USART2, TxBufPtr[TxByteCtr++]);  //отправка нового байта данных
//			  TxPhase = dks_tx_proc::_PH_CRC;
		    break;
	   case dks_tx_proc::_PH_CRC:
	        LL_USART_TransmitData8(USART2, TxCrc);
			TxPhase = dks_tx_proc::_PH_ALL;
		    break;
	   case dks_tx_proc::_PH_ALL:
//	        LL_USART_TransmitData8(USART2, TxCrc);
//			TxPhase = dks_tx_proc::_PH_LEN_DATA;
	        
			//TXE сбросится при отправке следующего байта (запуск протокола передачи)
			LL_USART_DisableIT_TXE(USART2); //все байты переданы, переходим к ожиданию TC
	        LL_USART_EnableIT_TC(USART2);
			TxEndFlag = true; //!!!отладка, передача завершена
		    break;
	   default:
		    break;		     
    }
}

//******************************************************************************
//  Обработчики прерываний
//******************************************************************************
extern "C" void USART1_IRQHandler(void)
{

}

extern "C" void USART2_IRQHandler(void)
{
//  USARTClockFreq   = LL_RCC_GetUSARTClockFreq(LL_RCC_USART2_CLKSOURCE);
//  USARTClockSource = LL_RCC_GetUSARTClockSource(LL_RCC_USART2_CLKSOURCE);

//  if (LL_USART_IsActiveFlag_TC(USART2) && LL_USART_IsEnabledIT_TC(USART2))
//  {
//    LL_USART_ClearFlag_TC(USART2);
//	 LL_USART_DisableIT_TC(USART2); //Disable Transmission Complete Interrupt

//	 Usart.reset_receive();
//  }

//  if (
//      LL_USART_IsActiveFlag_PE(USART2) ||
//		LL_USART_IsActiveFlag_FE(USART2) ||
//		LL_USART_IsActiveFlag_NE(USART2) ||
//		LL_USART_IsActiveFlag_ORE(USART2)		
//	  )
//  {
//    Usart.uerr |= (uint8_t)(USART2->ISR & 0x0F);

//	 LL_USART_ClearFlag_PE(USART2);
//    LL_USART_ClearFlag_FE(USART2);
//    LL_USART_ClearFlag_NE(USART2);
//    LL_USART_ClearFlag_ORE(USART2);
//  }
//  
//  if (LL_USART_IsActiveFlag_RTO(USART2)) //прерывание по таймауту прихода байтов
//  {
//    LL_USART_ClearFlag_RTO(USART2);
//	 
//    Usart.bppoint[Usart.bpphead] = USART::INBUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6);;
//    Usart.bpperr[Usart.bpphead]  = Usart.uerr;
//    Usart.bpphead++;
//    Usart.uerr             = 0;
//    if (Usart.bpphead >= 16)
//    {
//	   Usart.bpphead = 0;
//	 }
//    Usart.educt_pkg();
//    TIM6->CNT = 0;
//	 LL_TIM_ClearFlag_UPDATE(TIM6);       //Clear the update interrupt flag (UIF)
//    LL_TIM_EnableCounter(TIM6);          //Enable timer counter.
//	 LL_USART_DisableDirectionTx(USART2); //Transmitter Disable
//  }
  
  if (
      LL_USART_IsActiveFlag_RXNE(USART2) &&
      LL_USART_IsEnabledIT_RXNE(USART2)
     ) //Check if the USART Read Data Register Not Empty Flag is set or not.
  {
    LL_USART_ReceiveData8(USART2);
  }

  else if (
           LL_USART_IsActiveFlag_TC(USART2) &&
           LL_USART_IsEnabledIT_TC(USART2)
          ) //Check if the USART Transmission Complete Flag is set or not.
  {
    LL_USART_ClearFlag_TC(USART2);
	  LL_USART_DisableIT_TC(USART2); //USART2 свободен от передачи
	                                //можно перевести USART2 на другую задачу
    TxHigherPriorityTaskWoken = pdFALSE;
	  if (xSemaphoreGiveFromISR(UsartTxSemphr, &TxHigherPriorityTaskWoken) == pdFAIL)
	  {
//       RelOne.on();
	  }
	  else
	  {
//       RelOne.off();
	  }
	  if (TxHigherPriorityTaskWoken == pdTRUE)
	  {
      portYIELD_FROM_ISR(TxHigherPriorityTaskWoken);  // Принудительное переключение контекста для разблокировки задачи - обработчика
	  }
  }
  
  else if (
           LL_USART_IsActiveFlag_TXE(USART2) &&
           LL_USART_IsEnabledIT_TXE(USART2) 
          ) //Check if the USART Transmit Data Register Empty Flag is set or not.
  { 
	  switch (TxProc)
	  {
//	    case TProc::_SIMPLE:
//	 	     //релизовать при необходимости
//	 	     break;
	    case TProc::_DKS_TO_PC:
		      dks_txe_ok();
		      break;
		  case TProc::_AUTO:
		       //реализация по мере необходимости
//	  		  break;
		  case TProc::_MODBUS:
		       //реализация по мере необходимости
//	  		  break;
		  case TProc::_SENS:
		       //реализация по мере необходимости
//	  		  break;
		  case TProc::_OMNICOMM:
		       //реализация по мере необходимости
//	 		  break;
	    default:
		       break;
	  }
  }
}

extern "C" void USART3_IRQHandler(void)
{

}

extern "C" void DMA1_Channel7_IRQHandler(void)
{
  if (LL_DMA_IsActiveFlag_TC7(DMA1))  //проверка флага transfer complete для канала 7 DMA
  {
    LL_DMA_ClearFlag_TC7(DMA1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
  }
}

extern "C" void TIM6_DAC_IRQHandler(void)
{
  if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) //проверка флага interrupt flag (UIF)
  {	 
	 LL_TIM_ClearFlag_UPDATE(TIM6);
	 LL_TIM_DisableCounter(TIM6);
	 LL_USART_EnableDirectionTx(USART2); //Transmitter enable
  } 
}

