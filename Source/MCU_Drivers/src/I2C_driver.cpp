
#include "rtos_headers.h"
#include "main.h"
#include "lib.h"
#include "I2C_driver.h"


static uint8_t *TxBufPtr;             //буфер передачи
static uint8_t TxBufSize;             //размер буфера передачи
//static uint8_t TxByteCtr;           //счетчик перданных байтов данных
static bool IsTxSlaveRegAddr;         //флаг - Slave адрес отправлен
static BaseType_t TxHigherPriorityTaskWoken;

static uint8_t *RxBufPtr;             //буфер приема
static uint8_t RxBufSize;             //размер буфера приема
static uint8_t RxByteCtr;             //счетчик принятых байтов данных
static uint8_t RxSlaveAddr;           //адрес slave устройства. используется при считывании для повстарта
static BaseType_t RxHigherPriorityTaskWoken;

static I2C::TOperation OperationFlag;
static uint8_t StartRegAddr;          //начальный адрес регистра при записи/чтении

//static uint32_t I2CClockSource;
//static uint32_t I2CClockFreq;

namespace I2C
{
  using std::uint32_t;
  using std::uint8_t;
  
  bool TI2C::SlaveAddrFlag   = false;
}

namespace I2C
{
  using std::uint32_t;

  TI2C::TI2C(
             const TI2C_HW &_HW,             
             TMode _Mode, 
             TRate _Rate, 
             TAddressing _Addressing
            ) 
  :
  HW( _HW ),
  Mode( _Mode ),
  Rate( _Rate ),
	Addressing( _Addressing )
  {
	  SlaveRegAddr = 0x00;
	  TransferSize = 0x00;
	  Addr         = 0x00;
	 
//	 for (auto item : Gpio)
//	 {
//	   if (I2Cx == item.I2Cx)
//      {
//		  I2CHw = item;
//		  break;
//		}		
//	 }
	 	 
  }

  TI2C::~TI2C()
  {

  }
  
  
  void TI2C::pin_clk_config()
  {
    /*тактирование GPIO, к которым подключены SDA и SCL и тактирование интерфейса I2C*/
    HW.SDA.en_clk( HW.SDA.ClkPortMask);
    HW.SCL.en_clk( HW.SCL.ClkPortMask );
    HW.Clk.en_periph( HW.Clk.PeriphMask );
    
    /* инициализация выводов с I2C */
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    do
    {
      GPIO_InitStruct.Pin = HW.SDA.Nbr;
      GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
      GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
      GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
      GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    } while ( SUCCESS != LL_GPIO_Init( HW.SDA.Gpio, &GPIO_InitStruct ) );
	  do
	  {
       GPIO_InitStruct.Pin = HW.SCL.Nbr;
	  } while ( SUCCESS != LL_GPIO_Init( HW.SCL.Gpio, &GPIO_InitStruct ) );
    
    LL_RCC_SetI2CClockSource( HW.Clk.SrcMask ); //вырбрать источник тактирования I2C - SYSCLK
  }
  
  void TI2C::i2c_hw_init()
  {
    LL_I2C_InitTypeDef I2C_InitStruct;

    do
    {
      LL_I2C_Disable( HW.If ); // SCL и SDA отпущены, Внутренний автомат и биты статуса сброшены
    } while (LL_I2C_IsEnabled( HW.If ));

    LL_I2C_EnableClockStretching( HW.If ); //I2C д.б. отключен

	  do 
	  {
      switch (Mode)
		  {
		    case TMode::_SLAVE:
//	  	       I2C_InitTypeDef.OwnAddress1 = ; //реализация по необходимости
//             I2C_InitTypeDef.TypeAcknowledge = LL_I2C_ACK; //реализация по необходимости
//             I2C_InitTypeDef.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT; //реализация по необходимости
		    case TMode::_MASTER:
		         I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
		         break;
		    case TMode::_SMBUS_HOST: 
		         //реализация по необходимости
		         break;
		    case TMode::_SMBUS_DEVICE: 
		         //реализация по необходимости
		         break;
		    default:
		         break;
		  }
		  switch (Rate)
		  {
		    case TRate::_STANDARD:
	  	        I2C_InitStruct.Timing = 0x10B07DB7;  /* I2C Timing Configuration Tool for STM32F3xx and STM32F0xx devices v1.0.1
                                                      * Device Mode                   - Master, 
  	       	 										          * I2C Speed Mode                - Standard Mode, 
  	       	 										          * I2C Speed Frequency           - 100kHz, 
  	       	 										          * I2C Clock Source Frequency    - 64000kHz, 
  	       	 										          * Analog Filter Delay           - ON, 
  	       	 										          * Coefficient of Digital Filter - 0, 
  	       	 										          * Rise Time (ns)                - 100, 
  	       	 										          * Fall Time (ns)                - 50
  	       	 										          * Master, настраивается только при отключенном I2C
  	       	 */
		         break;
		    case TRate::_FAST:
#ifndef __DEBUG__
//             I2C_InitStruct.Timing = 0x00E02577; //72МГц, 400кГц, Rise Time = 100ns, Fall Time = 50ns
//             I2C_InitStruct.Timing = 0x00500C29; //26МГц, 400кГц, Rise Time = 100ns, Fall Time = 50ns
             I2C_InitStruct.Timing = 0x00200411; //12МГц, 400кГц, Rise Time = 100ns, Fall Time = 50ns
#else		  
             I2C_InitStruct.Timing = 0x00C02169; //400kHz
#endif
             /* I2C Timing Configuration Tool for STM32F3xx and STM32F0xx devices v1.0.1
                * Device Mode                   - Master, 
  	       	    * I2C Speed Mode                - Fast Mode, 
  	       	    * I2C Speed Frequency           - 400kHz, 
  	       	    * I2C Clock Source Frequency    - 64000kHz, 
  	       	    * Analog Filter Delay           - ON, 
  	       	    * Coefficient of Digital Filter - 0, 
  	       	    * Rise Time (ns)                - 100, 
  	       	    * Fall Time (ns)                - 50
  	       	    * Master, настраивается только при отключенном I2C
  	       	  */	       
		        I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
		        break;
		   case TRate::_FAST_MODE_PLUS:
	         I2C_InitStruct.Timing = 0x00900B1F; /* I2C Timing Configuration Tool for STM32F3xx and STM32F0xx devices v1.0.1
                                                    * Device Mode                   - Master, 
  	       	 										          * I2C Speed Mode                - Fast Mode Plus, 
  	       	 										          * I2C Speed Frequency           - 1000kHz, 
  	       	 										          * I2C Clock Source Frequency    - 64000kHz, 
  	       	 										          * Analog Filter Delay           - ON, 
  	       	 										          * Coefficient of Digital Filter - 0, 
  	       	 										          * Rise Time (ns)                - 100, 
  	       	 										          * Fall Time (ns)                - 50
  	       	 										          * Master, настраивается только при отключенном I2C
  	       	 										        */
		        break;
		   default:
		        break;
		 }
     I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
     I2C_InitStruct.DigitalFilter = 0x00U; //Min_Data = 0x00
    
	 } while (SUCCESS != LL_I2C_Init( HW.If, &I2C_InitStruct ));

	 
	 uint32_t AddressingMode = ((Addressing == TAddressing::_7_BIT) ? LL_I2C_ADDRESSING_MODE_7BIT 
	                                                                : LL_I2C_ADDRESSING_MODE_10BIT);
	 LL_I2C_SetMasterAddressingMode( HW.If, AddressingMode ); //конфигурация мастера для работы в 7-ми или 10-ти битном режиме  
	 LL_I2C_DisableReloadMode( HW.If );

	 i2c_set_interrupt(); //настройка NVIC

    LL_I2C_Enable( HW.If ); //Включить I2C
  }

  void TI2C::i2c_set_interrupt()
  {
    // Настройка NVIC
    NVIC_SetPriority(
                     HW.Ev_IRQ, 
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)
                    );                                                     //обработчик вызывает API функцию RTOS => приоритет д.б. \
                                                                             логически ниже или равен, но численно больше или равен,\
                                                                             установленному  в макросе \
                                                                             configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    NVIC_EnableIRQ(HW.Ev_IRQ);
  }  
    
  void TI2C::write_single_byte(uint8_t *TxByte, uint8_t _RegAddr, uint8_t SlaveAddr)
  {
    start: constexpr uint32_t DELAY_TIME_MS = 100;
    uint8_t BufSize = 0;
	 	  
		__disable_irq();
      TxBufSize     = 0x01U;
		  BufSize       = TxBufSize;
      TxBufPtr      = TxByte;
      OperationFlag = TOperation::_WRITE; //Общий флаг процесса - запись, которая начинается с записи адреса slave.    
		  StartRegAddr  = _RegAddr;
    __enable_irq();
		LL_I2C_SetTransferSize(  HW.If, 1U + BufSize ); //Количество байтов для записи
    start_transfer( SlaveAddr );
		
    if ( xSemaphoreTake( I2C_TxSem, pdMS_TO_TICKS(DELAY_TIME_MS) ) == pdFALSE) //ждать семафор окончания отправки байта из прерывания
		{
      LL_I2C_Disable(  HW.If  );
      LL_I2C_ClearFlag_ARLO(  HW.If  );
      LL_I2C_Enable(  HW.If  );
      goto start;
		}
	  
  }

  void TI2C::write_burst(uint8_t *DataBurst, uint8_t Size, uint8_t _StartRegAddr, uint8_t SlaveAddr)
  {
    start: constexpr uint32_t DELAY_TIME_MS = 100;
    
    uint8_t BufSize = 0;
	  
		__disable_irq();
		  TxBufSize     = Size; //
		  BufSize       = TxBufSize;
		  TxBufPtr      = DataBurst;
        OperationFlag = TOperation::_WRITE; //Общий флаг процесса - запись, которая начинается с записи адреса slave.
		  StartRegAddr  = _StartRegAddr;
		__enable_irq();
		LL_I2C_SetTransferSize(  HW.If , 1U + BufSize );            //Количество байтов для записи
    start_transfer( SlaveAddr );
		
	  if ( xSemaphoreTake( I2C_TxSem, pdMS_TO_TICKS(DELAY_TIME_MS) ) == pdFALSE) //ждать семафор окончания отправки кадра с данными из прерывания
		{
      LL_I2C_Disable(  HW.If  );
      LL_I2C_ClearFlag_ARLO(  HW.If  );
      LL_I2C_Enable(  HW.If  );
      goto start;
		}
  }

  void TI2C::read_single_byte(uint8_t *RxByte, uint8_t RegAddr, uint8_t SlaveAddr)
  {
    start: constexpr uint32_t DELAY_TIME_MS = 100;
    
		__disable_irq();
	     OperationFlag = TOperation::_READ; //Общий флаг процесса - считывание, которое начинается с записи адреса slave.
		  RxBufSize     = 0x01U;             //
		  RxBufPtr      = RxByte;
		  StartRegAddr  = RegAddr;
		  RxSlaveAddr   = SlaveAddr;
		  RxByteCtr     = 0U;
		__enable_irq();
    
		LL_I2C_SetTransferSize(  HW.If , 1U );            //Количество байтов для записи
    start_transfer( SlaveAddr );
    uint32_t HeapSize = xPortGetFreeHeapSize();
		
	  if ( xSemaphoreTake( I2C_RxSem, pdMS_TO_TICKS(DELAY_TIME_MS) ) == pdFALSE ) //ждать семафор окончания приема байта с данными из прерывания
		{
      LL_I2C_Disable( HW.If );
      LL_I2C_ClearFlag_ARLO( HW.If );
      LL_I2C_Enable( HW.If );
      goto start;
		}
  }
  
  void TI2C::read_burst(uint8_t *DataBurst, uint8_t _TransferSize, uint8_t _StartRegAddr, uint8_t SlaveAddr)
  {
    start: constexpr uint32_t DELAY_TIME_MS = 100;
		__disable_irq();
	     OperationFlag = TOperation::_READ; //Общий флаг процесса - считывание, которое начинается с записи адреса slave.
		  RxBufSize     = _TransferSize;     //
		  RxBufPtr      = DataBurst;
		  StartRegAddr  = _StartRegAddr;
		  RxSlaveAddr   = SlaveAddr;
		  RxByteCtr     = 0U;
		__enable_irq();
    
		LL_I2C_SetTransferSize( HW.If, 1U);            //Количество байтов для записи
    start_transfer(SlaveAddr);
    
	  if ( xSemaphoreTake( I2C_RxSem, pdMS_TO_TICKS(DELAY_TIME_MS) ) == pdFALSE ) //ждать семафор окончания приема кадра с данными из прерывания
		{
      LL_I2C_Disable( HW.If );
      LL_I2C_ClearFlag_ARLO( HW.If );
      LL_I2C_Enable( HW.If );
      goto start;
		}
  }
  
  void TI2C::start_transfer(uint8_t SlaveAddr)
  {
		LL_I2C_DisableAutoEndMode( HW.If );                       //Software end: TC флаг устанавливается, когда было передано/принято NBYTES данных. \
	                                                                  Растяжка SCL
		LL_I2C_SetMasterAddressingMode( HW.If, LL_I2C_ADDRESSING_MODE_7BIT );
		LL_I2C_SetSlaveAddr( HW.If, SlaveAddr << 1 ); //Конфигурирует SLAVE адрес для обмена (режим мастера). \
														                        Нельзя изменять при установленном START бите
//		LL_I2C_SetSlaveAddr(I2CHw.I2Cx, SlaveAddr);                  //Конфигурирует SLAVE адрес для обмена (режим мастера). \
//														                           Нельзя изменять при установленном START бите
	   LL_I2C_SetTransferRequest( HW.If, LL_I2C_REQUEST_WRITE ); //Кофигурирует направление обмена (в режиме мастера). \
	                                                               Нельзя изменять при установленном START бите. \
																							                   Для всех реализуемых протоколов сначала необходимо \
                                                                 записать адрес slave регистра в микросхему.

		__disable_irq();
        IsTxSlaveRegAddr = false;                                    //Сброс программного флага перед стартом отправки, т.е. адрес регистра в slave еще не отправлен 
		__enable_irq();
		
	   LL_I2C_GenerateStartCondition( HW.If );                   //Generate a START or RESTART condition. \
	                                                               Старт бит может быть установлен даже если шина I2C занята или I2C в slave режиме. \
	                                                               Не влияет когда установлен RELOAD.	
//      RelFour.off();																					
		LL_I2C_EnableIT_TX( HW.If ); // Enable TXIS interrupt
  }
  
  const char *TI2C::get_i2c_sign() const
  {
    return "I2C1";
  }
}
//******************************************************************************
// Функции, вызываемые из обработчиков прерываний
//******************************************************************************
__INLINE static void dks_txis_active(void)
{
    LL_I2C_ClearFlag_STOP(I2C1); //Clear Stop detection flag
	 if (!IsTxSlaveRegAddr) //Если адрес регистра в slave не отправлен
	 {
	   LL_I2C_TransmitData8(I2C1, StartRegAddr); //Запись адреса регистра в slave в регистр передачи данных I2C \
		                                            START сброшен, ACK принят
      LL_I2C_EnableIT_TC(I2C1);                 //Enable Transfer Complete interrupt
      IsTxSlaveRegAddr = true;
	 }
	 else
	 {
	   switch (OperationFlag)
		{
		  case I2C::TOperation::_READ:
		        break;
		  case I2C::TOperation::_WRITE:
	          LL_I2C_TransmitData8(I2C1, TxBufPtr[--TxBufSize]);      //Запись очередного байта в slave
				 if (!TxBufSize)
				 {
               LL_I2C_DisableIT_TX(I2C1);                            //Disable TXIS interrupt
//	            LL_I2C_EnableIT_TC(I2C1);                            //Enable Transfer Complete interrupt
				 }
             break;
		}
	 }
//	 else //Адрес регистра в slave уже отправлен
//	 {
//	   switch (OperationFlag)
//		{
//		  case I2C::TOperation::_READ:
//		       LL_I2C_SetSlaveAddr(I2C1, RxSlaveAddr);                //Конфигурирует SLAVE адрес для обмена (режим мастера) \
//				 										                          //Нельзя изменять при установленном START бите
//	          LL_I2C_SetTransferRequest(I2C1, LL_I2C_REQUEST_READ);  //Кофигурирует направление обмена (в режиме мастера) \
//	                                                                 //Нельзя изменять при установленном START бите
//             LL_I2C_SetTransferSize(I2C1, RxBufSize);               //Количество байтов для считывания
//	          LL_I2C_DisableIT_TX(I2C1);                             //Disable TXIS interrupt
//				 LL_I2C_EnableIT_RX(I2C1);                              //Enable RXNE interrupt
//             LL_I2C_EnableAutoEndMode(I2C1);                        //Automatic end: Стоп условие отправляется автоматически, когда было передано/принято NBYTES данных
//             LL_I2C_GenerateStartCondition(I2C1);                   //Generate a RESTART condition
//				                                                        //Флаг TC, при AUTOEND = 1, не работает
//																						  //TXIS сбрасывается при 
////             RxIdx = AccGyroMag.TransferSize;
//		       break;
//		  case I2C::TOperation::_WRITE:
//	          LL_I2C_TransmitData8(I2C1, TxBufPtr[TxBufSize--]);      //Запись очередного байта в slave
//				 if (!TxBufSize)
//				 {
//	            LL_I2C_EnableIT_TC(I2C1);                            //Enable Transfer Complete interrupt
//				 }
//		       break;
//		  default:
//		       break;
//		}
//	 }
} 

__INLINE static void dks_rxne_active(void)
{
  switch (OperationFlag)
  {
    case I2C::TOperation::_READ:
	      
//			if (RxByteCtr == 0)
//			{
//			  RelFour.off();
//			}
	      RxBufPtr[RxByteCtr++] = LL_I2C_ReceiveData8(I2C1); //считать принятый байт => флаг RXNE сбросится
			if (RxByteCtr == RxBufSize)
			{
           LL_I2C_DisableIT_RX(I2C1);  //Disable RXNE interrupt
           LL_I2C_EnableIT_STOP(I2C1); //Enable STOP detection interrupt
			}
  	      break;
    case I2C::TOperation::_WRITE:
  	      break;
    default:
  	      break;
  }
} 

__INLINE static void dks_stop_active(void)
{
  LL_I2C_ClearFlag_STOP(I2C1); //Clear Stop detection flag
  LL_I2C_DisableIT_STOP(I2C1); //Disable STOP detection interrupt
//         RelThree.toggle(); 
  		
  switch (OperationFlag)
  {
    case I2C::TOperation::_READ:
//         RelThree.toggle();
         RxHigherPriorityTaskWoken = pdFALSE;
         if (xSemaphoreGiveFromISR(I2C_RxSem, &RxHigherPriorityTaskWoken) == pdFAIL) //отправить семафор окончания считывания
         {
           //семафор уже был доступен, т.е. ранее отдан другой задачей или прерыванием
         } 
         if (RxHigherPriorityTaskWoken == pdPASS)
         {
           portYIELD_FROM_ISR(RxHigherPriorityTaskWoken);  //принудительное переключение контекста для разблокировки задачи - обработчика
         }
  	      break;
    case I2C::TOperation::_WRITE:
  	      break;
    default:
  	      break;
  }
} 

__INLINE static void dks_tc_active(void)
{
  switch (OperationFlag)
  {
    case I2C::TOperation::_READ:
		     LL_I2C_SetSlaveAddr(I2C1, RxSlaveAddr << 1);          //Конфигурирует SLAVE адрес для обмена (режим мастера) \
                                                               //Нельзя изменять при установленном START бите
	       LL_I2C_SetTransferRequest(I2C1, LL_I2C_REQUEST_READ); //Кофигурирует направление обмена (в режиме мастера) \
	                                                             //Нельзя изменять при установленном START бите
         LL_I2C_SetTransferSize(I2C1, RxBufSize);              //Количество байтов для считывания
	       LL_I2C_DisableIT_TX(I2C1);                            //Disable TXIS interrupt
	       LL_I2C_DisableIT_TC(I2C1);                            //Disable Transfer Complete interrupt
         LL_I2C_EnableIT_RX(I2C1);                             //Enable RXNE interrupt
         LL_I2C_EnableAutoEndMode(I2C1);                       //Automatic end: Стоп условие отправляется автоматически, когда было передано/принято NBYTES данных
         LL_I2C_GenerateStartCondition(I2C1);                  //Generate a RESTART condition
                                                               //Флаг TC, при AUTOEND = 1, не работает
                                                               //TXIS сбрасывается при 
  	     break;
    case I2C::TOperation::_WRITE:
//         LL_I2C_ClearFlag_STOP(I2C1); //Clear Stop detection flag
//         RelThree.toggle(); 
	       LL_I2C_GenerateStopCondition(I2C1); //генерирует STOP условие (в режиме мастера) + сброс флага TC
//         LL_I2C_DisableIT_TX(I2C1);          //Disable TXIS interrupt
	       LL_I2C_DisableIT_TC(I2C1);          //Disable Transfer Complete interrupt
			
//  RelFour.toggle();
			   TxHigherPriorityTaskWoken = pdFALSE;
			   if (xSemaphoreGiveFromISR(I2C_TxSem, &TxHigherPriorityTaskWoken) == pdFAIL) //отправить семафор окончания записи
         {
           //семафор уже был доступен, т.е. ранее отдан другой задачей или прерыванием
         }  
	       if ( TxHigherPriorityTaskWoken == pdPASS )
	       {
            portYIELD_FROM_ISR(TxHigherPriorityTaskWoken);  //принудительное переключение контекста для разблокировки задачи - обработчика
	       }
  	     break;
    default:
  	     break;
  }
} 

//******************************************************************************
//  Обработчики прерываний
//******************************************************************************
extern "C" void I2C1_EV_IRQHandler(void)
{  
//  I2CClockSource = LL_RCC_GetI2CClockSource(LL_RCC_I2C1_CLKSOURCE);
//  I2CClockFreq = LL_RCC_GetI2CClockFreq(LL_RCC_I2C1_CLKSOURCE);
  
  if (LL_I2C_IsActiveFlag_TXIS(I2C1))
  {
    dks_txis_active();
  }
  else if (LL_I2C_IsActiveFlag_RXNE(I2C1)) //если байт принят
  {
    dks_rxne_active();
  }
  else if (LL_I2C_IsActiveFlag_STOP(I2C1) && OperationFlag == I2C::TOperation::_READ)
  {
	 dks_stop_active();
  }
  else if (LL_I2C_IsActiveFlag_TC(I2C1))
  {
    dks_tc_active();
  }
}

extern "C" void I2C1_ER_IRQHandler(void)
{

}

extern "C" void I2C2_EV_IRQHandler(void)
{

}

extern "C" void I2C2_ER_IRQHandler(void)
{

}
