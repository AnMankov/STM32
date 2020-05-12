//#include <string.h>

#include "rtos_headers.h"
#include "main.h"
#include "lib.h"

#include "CAN_driver.h"

static uint8_t Fifo0CanTmp[bxCAN::TCAN::DATA_SIZE];
static uint32_t LastErrorCode[bxCAN::_MAX_LEC];
static uint32_t CanError[bxCAN::_MAX_CAN_ERROR];
static uint32_t Fifo0Warinig[bxCAN::_MAX_FIFO_WARNING];
static BaseType_t Rx0HigherPriorityTaskWoken;
//static BaseType_t ManyMsg0HigherPriorityTaskWoken;
static BaseType_t TxHigherPriorityTaskWoken;
static uint32_t IdFifo0; 

static uint8_t Fifo1CanTmp[bxCAN::TCAN::DATA_SIZE];
static uint32_t Fifo1Warinig[bxCAN::_MAX_FIFO_WARNING];
static BaseType_t Rx1HigherPriorityTaskWoken;
static uint32_t IdFifo1;
//static BaseType_t ManyMsg1HigherPriorityTaskWoken;

constexpr bxCAN::TMailboxNum Mailbox = bxCAN::_ONE; //используется в классе CAN и в прерывании

struct TFifoCtrl
{
  __IO uint32_t *RxFifoReg;
  uint32_t MSG_PEND_MASK;
  uint8_t (&Buf)[bxCAN::TCAN::DATA_SIZE];
  uint32_t *pIdFifo;
  __IO uint8_t *RxFifoData;
  __IO uint32_t *RxFifoId;
  uint32_t RELEASE_MAILBOX_MASK;
  BaseType_t *pHPTaskWoken;
  SemaphoreHandle_t *Semphr;
  uint32_t FULL_IE_MASK;
  uint32_t FIFO_FULL_MASK;
  uint32_t *FifoWarinig;
  uint32_t FIFO_OVR_IE_MASK;
  uint32_t FIFO_OVR_REG_MASK;
};

static TFifoCtrl FifoCtrl[(uint8_t)bxCAN::TRxFifoNum::_MAX] =
{
  {
    &CAN->RF0R,
    CAN_RF0R_FMP0,
    Fifo0CanTmp,
    &IdFifo0,
    (__IO uint8_t *)&CAN->sFIFOMailBox[0].RDLR,
    &CAN->sFIFOMailBox[0].RIR,
    CAN_RF0R_RFOM0,
    &Rx0HigherPriorityTaskWoken,
    &CANRx0Semphr,
    CAN_IER_FFIE0,
    CAN_RF0R_FULL0,
    Fifo0Warinig,
    CAN_IER_FOVIE0,
    CAN_RF0R_FOVR0,
  },
  {
    &CAN->RF1R,
    CAN_RF1R_FMP1,
    Fifo1CanTmp,
    &IdFifo1,
    (__IO uint8_t *)&CAN->sFIFOMailBox[1].RDLR,
    &CAN->sFIFOMailBox[1].RIR,
    CAN_RF1R_RFOM1,
    &Rx1HigherPriorityTaskWoken,
    &CANRx1Semphr,
    CAN_IER_FFIE1,
    CAN_RF1R_FULL1,
    Fifo1Warinig,
    CAN_IER_FOVIE1,
    CAN_RF1R_FOVR1,
  },
};

static void rx_handler(TFifoCtrl *Ctrl);

namespace bxCAN //Basic Extended CAN
{  
  //Модифицировать константу под схемотехнику
  const TCANHw Gpio =
  {
    GPIOA,
    GPIOA,
    GPIOB,
    GPIOB,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_PERIPH_GPIOB,
    LL_AHB2_GRP1_PERIPH_GPIOB,
	  LL_APB1_GRP1_PERIPH_CAN1,
    LL_GPIO_PIN_12,
    LL_GPIO_PIN_11,
    LL_GPIO_PIN_15,
    LL_GPIO_PIN_14,
    LL_GPIO_AF_3, //альтернативная функция для инициализации как GPIO
    LL_GPIO_AF_0, //альтернативная функция для инициализации как GPIO
    "CAN"
  };
  
  constexpr T11Id Id[_MAX_DEV+1] =
  {
    TDevice::_HC_1 , //крышка люка 1
    TDevice::_HC_2 , //крышка люка 2
    TDevice::_HC_3 , //крышка люка 3
    TDevice::_HC_4 , //крышка люка 4
    TDevice::_HC_5 , //крышка люка 5
    TDevice::_HC_6 , //крышка люка 6
    TDevice::_HC_7 , //крышка люка 7
    TDevice::_HC_8 , //крышка люка 8
    TDevice::_PLATF, //платформа
  };

//  constexpr uint8_t MAX_DATA_LEN_CODE = 8U;
//  constexpr uint32_t EmptyFlagMask[]  = { CAN_TSR_TME0, CAN_TSR_TME1, CAN_TSR_TME2 };
}

namespace bxCAN //Basic Extended CAN
{
  TCAN::TCAN(TBaudRate _BaudRate, TIdType _IdType)
  :
  CANHw(Gpio),
  BaudRate(_BaudRate),
  Mode(TMode::_SLEEP), //по сбросу CAN в Sleep, CANTX в pull-up
  IdType(_IdType),
  Lec(TLec::_NO_ERR)
  {
    //Configure CAN parameters
    //идентификатор с наименьшим значением обладает бОльшим приоритетом
    //одинаковые идентификаторы => приоритет у почтового ящика с меньшим номером
    
    CAN->sTxMailBox[0].TIR &= ~CAN_TI0R_STID; //обнуляем Standard identifier для отладки
  }

  TCAN::~TCAN()
  {

  }

  void TCAN::pin_clk_config()
  {
    /*тактирование GPIO и CAN*/
    if (!LL_AHB2_GRP1_IsEnabledClock(CANHw.TxPinClkMask))  //включение тактирования GPIO для порта с Tx CAN, если не включено
    {
      LL_AHB2_GRP1_EnableClock(CANHw.TxPinClkMask);
    }
    
	  if (!LL_AHB2_GRP1_IsEnabledClock(CANHw.RxPinClkMask))  //включение тактирования GPIO для порта с Rx CAN, если не включено
    {
      LL_AHB2_GRP1_EnableClock(CANHw.RxPinClkMask);
    }
    
	  if (!LL_APB1_GRP1_IsEnabledClock(CANHw.PeriphClkMask)) //включение тактирования шины APB1 для CAN, если не включено
    {
      LL_APB1_GRP1_EnableClock(CANHw.PeriphClkMask);
    }
    
    /* инициализация выводов с CAN */
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    do
    {
      GPIO_InitStruct.Pin = CANHw.TxPinMask;
      GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
      GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
      GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
      GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
    } while ( SUCCESS != LL_GPIO_Init(CANHw.GpioTx, &GPIO_InitStruct) );
    do
    {
       GPIO_InitStruct.Pin = CANHw.RxPinMask;
    } while ( SUCCESS != LL_GPIO_Init(CANHw.GpioRx, &GPIO_InitStruct) );
     
    aux_init(); //инициализация вспомогательных выводов
  }
  
  void TCAN::hw_init()
  {
	  set_mode(TMode::_INITIALIZATION); //по сбросу bxCAN в Sleep mode, входим в initialization mode

    clear_dbg_freeze();
	  bit_timing_init();
	  filter_init();
    set_interrupt();
	  error_init();

	  set_mode(TMode::_NORMAL); //после завершения инициализации входим в normal mode для синхронизации с CAN шиной \
	                            //и запуска приема и передачи
  }
  
  void TCAN::clear_dbg_freeze()
  {
    constexpr uint32_t CAN_MCR_DBF_Pos = 16U;                                           
    constexpr uint32_t CAN_MCR_DBF_Msk = 0x1U << CAN_MCR_DBF_Pos;
    constexpr uint32_t CAN_MCR_DBF     = CAN_MCR_DBF_Msk;

	  CAN->MCR &= ~CAN_MCR_DBF; //CAN работает при отладке
  }

  void TCAN::set_interrupt()
  {
    NVIC_SetPriority(CAN1_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));  //обработчик вызывает API функцию RTOS => приоритет д.б. логически ниже или равен,
	                                                                                                //но численно больше или равен, установленному  в макросе configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    NVIC_EnableIRQ(CAN1_TX_IRQn);

	  NVIC_SetPriority(CAN1_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)); //обработчик вызывает API функцию RTOS => приоритет д.б. логически ниже или равен,
	                                                                                                //но численно больше или равен, установленному  в макросе configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    NVIC_EnableIRQ(CAN1_RX0_IRQn);

	  NVIC_SetPriority(CAN1_RX1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));        //обработчик вызывает API функцию RTOS => приоритет д.б. логически ниже или равен,
	                                                                                                //но численно больше или равен, установленному  в макросе configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    NVIC_EnableIRQ(CAN1_RX1_IRQn);

	  NVIC_SetPriority(CAN1_SCE_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));        //обработчик вызывает API функцию RTOS => приоритет д.б. логически ниже или равен,
	                                                                                                //но численно больше или равен, установленному  в макросе configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    NVIC_EnableIRQ(CAN1_SCE_IRQn);

	  CAN->IER |= CAN_IER_FMPIE0; //разрешение прерываний для FIFO0 - в Slave это прерывания по приему сообщений от Master \
                                  разрешение прерываний для FIFO0 - в Master это прерывания по приему сообщений от Slave1
    CAN->IER |= CAN_IER_FFIE0;											 
	  CAN->IER |= CAN_IER_FOVIE0;
	  CAN->MCR &= ~CAN_MCR_RFLM; //отключить блокирующую функцию FIFO \
	                               последнее сообщение в FIFO будет перезаписано новым пришедшим сообщением \
										             => приложению доступно самое последнее пришедшее сообщение
	 
	  CAN->MCR |= CAN_MCR_RFLM;  //включить блокирующую функцию FIFO \
	                               последнее сообщение в FIFO будет отброшено \
 										             => в FIFO доступны 3 самых старых сообщения
	  switch (::Device)
	  {
	    case TDevice::_PLATF:
	         CAN->IER |= CAN_IER_FMPIE1; //разрешение прерываний для FIFO1 - в Master это прерывания по приему сообщений от Slave2
	         CAN->IER |= CAN_IER_FFIE1;
	         CAN->IER |= CAN_IER_FOVIE1;
		       break;
	    case TDevice::_HC_1:
	    case TDevice::_HC_2:
           CAN->IER |= CAN_IER_LECIE;
		       break;
	    default:
		       break;
	  }
  }

  void TCAN::bit_timing_init()
  {
	  /*
	  * Ошибка в RM0316,  Rev 8, с.1027
	  * As a safeguard against programming errors, the configuration of the Bit Timing Register
    * (CAN_BTR) is only possible while the device is in Standby mode
	  * На с.1039
	  * This register can only be accessed by the software when the CAN hardware is in
    * initialization mode
    */
    if (get_mode() != TMode::_INITIALIZATION)
    {
      set_mode(TMode::_INITIALIZATION); //выбор тестового режима делается только в режиме инициализации
    }
	 
//	 constexpr uint32_t MAX_FREQ_APB1_HZ = 36000000U;
	 
	  //расчет для определения количества квантов в сегментах находится в файле D:\stm32\CAN_count.xlsx
	  //расчет ведется с целью получения целого значения количества тактов в кванте для заданного Baud Rate CAN
	  constexpr uint8_t SUM_QUANTA_NUM = 10U;                                   //количество квантов в одном бите CAN
	  constexpr uint8_t BS1_QUANTA_NUM = 6U;                                    //количество квантов в BIT SEGMENT 1
	  constexpr uint8_t BS2_QUANTA_NUM = SUM_QUANTA_NUM - (BS1_QUANTA_NUM + 1); //количество квантов в BIT SEGMENT 2
	  constexpr uint8_t SJW_QUANTA_NUM = 4U;                                    //количество квантов в SJW
	  
	  float TPclk1;
	  float NominalBitTimeUS;                          //длительность одного бита CAN
	  float QuantaTime;                                //длительность одного кванта времени CAN
	  uint8_t TickPerQuantaNum;                        //количество тактов источника тактирования CAN в одном кванте
	  
	  LL_RCC_ClocksTypeDef RCC_Clocks;
	  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
//	 if ( RCC_Clocks.PCLK1_Frequency >= MAX_FREQ_APB1_HZ ) //Tpclk
//	 {
//      TPclk1 = 1000000.0f / MAX_FREQ_APB1_HZ;
//	 }
//	 else
//	 {
//      TPclk1 = 1000000.0f / RCC_Clocks.PCLK1_Frequency;
//	 }

    constexpr float BAUD_RATE = 1000000.0f;
    TPclk1 = BAUD_RATE / RCC_Clocks.PCLK1_Frequency;
	  NominalBitTimeUS = BAUD_RATE / (float)BaudRate;
	  QuantaTime       = NominalBitTimeUS / SUM_QUANTA_NUM;
	  TickPerQuantaNum = QuantaTime / TPclk1;
    
	  CAN->BTR = (CAN->BTR & ~(CAN_BTR_BRP | CAN_BTR_TS1 | CAN_BTR_TS2 | CAN_BTR_SJW)) 
	           | ((TickPerQuantaNum - 1) << CAN_BTR_BRP_Pos)
             | ((BS1_QUANTA_NUM - 1)   << CAN_BTR_TS1_Pos)
             | ((BS2_QUANTA_NUM - 1)   << CAN_BTR_TS2_Pos)
             | ((SJW_QUANTA_NUM - 1)   << CAN_BTR_SJW_Pos);
  }
  
  void TCAN::filter_init()
  {
    //инициализацию фильтров не обязательно делать в initialization mode (с.1013 RM, rev.8) \
      но filter scale и mode configuration должны быть настроены перед входом в normal mode
    if ( get_mode() == TMode::_NORMAL )
	  { 
	    set_mode(TMode::_INITIALIZATION);  
	  }
    
    __packed struct TIdList16bit
	  {
	    uint16_t EXID_17_15 : 3;
	    uint16_t IDE        : 1;
	    uint16_t RTR        : 1;
		  uint16_t STID       : 11;
	  };

	  union TIdMapping //доработать эту структуру с помощью union для охвата различных вариантов фильтрации
    {
	    struct
		  {
		    TIdList16bit Id1;
        TIdList16bit Id2;
		  };
		  uint32_t Reg;
	  };
	 
	  TFilterScale FilterScale;     
	  TFilterMode FilterMode;       
	  TFilterBankNum FilterBankNum; 
	  TRxFifoNum RxFifoNum;
    TIdMapping Map;

	  switch (::Device)
	  {
	    case TDevice::_PLATF:
/* FIFO0 - для данных от HatchCover
 * К FIFO0 привязываем Filter Bank 0
*/	 
	         //проверить что бит FACTx сброшен в регистре CAN_FAxR \
	           или проверить что бит FINIT установлен в регистре CAN_FMR => \
	           => CAN_FiRx может быть модифицирован
           filter_activate(TFilterBankNum::_F_NULL, TFilterActivate::_F_NOT_ACTIVE); //деактивировать Filter Bank 0
           filter_activate(TFilterBankNum::_F_ONE, TFilterActivate::_F_NOT_ACTIVE);  //деактивировать Filter Bank 1
           
	         FilterScale   = TFilterScale::_16_Bit;
	         FilterMode    = TFilterMode::_ID_LIST;
           FilterBankNum = TFilterBankNum::_F_NULL;
	         RxFifoNum     = TRxFifoNum::_NULL;
	         filter_bank_modify(FilterScale, FilterMode, FilterBankNum, RxFifoNum);
           
	         Map.Reg      = 0x00;
	         Map.Id1.STID = Id[TDevice::_HC_1].Val;
	         Map.Id1.RTR  = 0U;              //Data frame
	         Map.Id1.IDE  = 0U;              //Standard identifier
	         Map.Id2 = Map.Id1;
	         CAN->sFilterRegister[FilterBankNum].FR1 = Map.Reg; //каждому банку соответствует 2 32-разрядных регистра
	         CAN->sFilterRegister[FilterBankNum].FR2 = CAN->sFilterRegister[FilterBankNum].FR1; //4 одинаковых фильтра в банке

/* FIFO1 - для данных от Lock
 * К FIFO1 привязываем Filter Bank 1
*/
	         FilterScale   = TFilterScale::_16_Bit;
	         FilterMode    = TFilterMode::_ID_LIST;
           FilterBankNum = TFilterBankNum::_F_ONE;
	         RxFifoNum     = TRxFifoNum::_ONE;
	         filter_bank_modify(FilterScale, FilterMode, FilterBankNum, RxFifoNum);
           
	         Map.Reg      = 0x00;
	         Map.Id1.STID = Id[TDevice::_HC_2].Val;
	         Map.Id1.RTR  = 0U;              //Data frame
	         Map.Id1.IDE  = 0U;              //Standard identifier
	         Map.Id2 = Map.Id1;
	         CAN->sFilterRegister[FilterBankNum].FR1 = Map.Reg; //каждому банку соответствует 2 32-разрядных регистра
	         CAN->sFilterRegister[FilterBankNum].FR2 = CAN->sFilterRegister[FilterBankNum].FR1; //4 одинаковых фильтра в банке

           filter_activate(TFilterBankNum::_F_NULL, TFilterActivate::_F_ACTIVE); //активировать Filter Bank 0
           filter_activate(TFilterBankNum::_F_ONE, TFilterActivate::_F_ACTIVE);  //активировать Filter Bank 1
		       break;
	    case TDevice::_HC_1:
	    case TDevice::_HC_2:
           filter_activate(TFilterBankNum::_F_NULL, TFilterActivate::_F_NOT_ACTIVE); //деактивировать Filter Bank 0
           
	         FilterScale   = TFilterScale::_16_Bit;
	         FilterMode    = TFilterMode::_ID_LIST;
           FilterBankNum = TFilterBankNum::_F_NULL;
	         RxFifoNum     = TRxFifoNum::_NULL;
	         filter_bank_modify(FilterScale, FilterMode, FilterBankNum, RxFifoNum);
           
	         Map.Reg      = 0x00;
	         Map.Id1.STID = Id[TDevice::_PLATF].Val;
	         Map.Id1.RTR  = 0U;              //Data frame
	         Map.Id1.IDE  = 0U;              //Standard identifier
	         Map.Id2 = Map.Id1;
	         CAN->sFilterRegister[FilterBankNum * 2U].FR1 = Map.Reg; //каждому банку соответствует 2 32-разрядных регистра
	         CAN->sFilterRegister[FilterBankNum * 2U].FR2 = CAN->sFilterRegister[FilterBankNum * 2].FR1;
           
           filter_activate(TFilterBankNum::_F_NULL, TFilterActivate::_F_ACTIVE); //активировать Filter Bank 0
		       break;
	    default:
           break;
	  }
  }

  void TCAN::filter_bank_modify(TFilterScale Scale, TFilterMode Mode, TFilterBankNum BankNum, TRxFifoNum RxFifoNum)
  {
    struct TBankSettings
	  {
	    uint32_t ModeMask;
	    uint32_t ScaleMask;
	    uint32_t FifoAssignMask;
	  };

	  constexpr TBankSettings BankSettings[] =
	  {
	    { CAN_FM1R_FBM0,  CAN_FS1R_FSC0,  CAN_FFA1R_FFA0  },
	    { CAN_FM1R_FBM1,  CAN_FS1R_FSC1,  CAN_FFA1R_FFA1  },
	    { CAN_FM1R_FBM2,  CAN_FS1R_FSC2,  CAN_FFA1R_FFA2  },
	    { CAN_FM1R_FBM3,  CAN_FS1R_FSC3,  CAN_FFA1R_FFA3  },
	    { CAN_FM1R_FBM4,  CAN_FS1R_FSC4,  CAN_FFA1R_FFA4  },
	    { CAN_FM1R_FBM5,  CAN_FS1R_FSC5,  CAN_FFA1R_FFA5  },
	    { CAN_FM1R_FBM6,  CAN_FS1R_FSC6,  CAN_FFA1R_FFA6  },
	    { CAN_FM1R_FBM7,  CAN_FS1R_FSC7,  CAN_FFA1R_FFA7  },
	    { CAN_FM1R_FBM8,  CAN_FS1R_FSC8,  CAN_FFA1R_FFA8  },
	    { CAN_FM1R_FBM9,  CAN_FS1R_FSC9,  CAN_FFA1R_FFA9  },
	    { CAN_FM1R_FBM10, CAN_FS1R_FSC10, CAN_FFA1R_FFA10 },
	    { CAN_FM1R_FBM11, CAN_FS1R_FSC11, CAN_FFA1R_FFA11 },
	    { CAN_FM1R_FBM12, CAN_FS1R_FSC12, CAN_FFA1R_FFA12 },
	    { CAN_FM1R_FBM13, CAN_FS1R_FSC13, CAN_FFA1R_FFA13 }
	  };
	 
    //необходимо сделать различные настройки фильтров для master'a и slave'ов
    switch (::Device)
	  {
	    case TDevice::_PLATF:
	         CAN->FFA1R &= ~BankSettings[BankNum].FifoAssignMask; //назначение фильтра для FIFO0
		       break;
	    case TDevice::_HC_1:
	    case TDevice::_HC_2:
	         CAN->FFA1R &= ~BankSettings[BankNum].FifoAssignMask; //назначение фильтра для FIFO0
		       break;
	    default:
		       break;
	  }	 
	  switch (Scale)
	  {
	    case TFilterScale::_16_Bit:
  	       CAN->FS1R &= ~BankSettings[BankNum].ScaleMask; //Dual 16-bit scale configuration
		       break;
	    case TFilterScale::_32_Bit:
  	       CAN->FS1R |= BankSettings[BankNum].ScaleMask;  //Single 32-bit scale configuration
		       break;
	    default:
		       break;
	  }
	  switch (Mode)
	  {
	    case TFilterMode::_ID_LIST:
           CAN->FM1R |= BankSettings[BankNum].ModeMask;  //Two 32-bit registers of filter bank x are in Identifier List mode
		       break;
	    case TFilterMode::_ID_MASK:
           CAN->FM1R &= ~BankSettings[BankNum].ModeMask; //Two 32-bit registers of filter bank x are in Identifier Mask mode
		       break;
	    default:
		       break;
	  }
	  switch (RxFifoNum)
	  {
	    case TRxFifoNum::_NULL:
	         CAN->FFA1R &= ~BankSettings[BankNum].FifoAssignMask; //Filter assigned to FIFO 0
		       break;
	    case TRxFifoNum::_ONE:
	         CAN->FFA1R |= BankSettings[BankNum].FifoAssignMask; //Filter assigned to FIFO 1
		       break;
	    default:
		       break;
	  }
   
//master принимает данные от двух slave'ов => в списке должно быть два идентификатора
//slave принимает данные от одного master'а => в списке должен быть один идентификатор	 
  }
  
  void TCAN::filter_activate(TFilterBankNum BankNum, TFilterActivate Activate)
  {
    uint32_t ActivateMask[] =
	 {
	   CAN_FA1R_FACT0, CAN_FA1R_FACT1, CAN_FA1R_FACT2, CAN_FA1R_FACT3,
	   CAN_FA1R_FACT4, CAN_FA1R_FACT5, CAN_FA1R_FACT6, CAN_FA1R_FACT7,
	   CAN_FA1R_FACT8, CAN_FA1R_FACT9, CAN_FA1R_FACT10, CAN_FA1R_FACT11,
	   CAN_FA1R_FACT12, CAN_FA1R_FACT13
	 };
	 switch (Activate)
	 {
	   case TFilterActivate::_F_NOT_ACTIVE:
		     if (CAN->FA1R & ActivateMask[BankNum])
			  {
	          CAN->FA1R &= ~ActivateMask[BankNum]; //фильтр не активирован
			  }
	        if ( !(CAN->FMR & CAN_FMR_FINIT) )
	        {
	          CAN->FMR |= CAN_FMR_FINIT; //включение режима инициализации для фильтров \
                                          прием деактивирован
	        }
		     break;
	   case TFilterActivate::_F_ACTIVE:
	        CAN->FMR &= ~CAN_FMR_FINIT;  //активация фильтров	 
           CAN->FA1R |= ActivateMask[BankNum]; //фильтр активирован 
		     break;
	   default:
		     break;
	 }	 
  }
  
  void TCAN::error_init()
  {
    //По CAN протоколу - ошибки обрабатываются аппаратно используя TEC и REC \
	   TEC и REC можно использовать для определения стабильности сети \
	   ESR - информация об ошибках от CAN движка \
	   TEC > 255 => Bus-Off => CAN не может передавать/принимать сообщения \
	   CAN восстановится после 128 последовательностей из 11 рецессивных битов на CANRX	 
	 CAN->MCR |= CAN_MCR_ABOM; //автоматическое восстановление
	                           //если программное => bxCAN должен войти в режим инициализации и выйти из него

	 CAN->IER |= CAN_IER_ERRIE; //разрешить прерывание по ошибкам из CAN_ESR

	 //индивидуальное разрешение прерываний
	 CAN->IER |= CAN_IER_BOFIE; //Bus-off interrupt enable
//	 CAN->IER |= CAN_IER_EPVIE; //Error passive interrupt enable
//	 CAN->IER |= CAN_IER_EWGIE; //Error warning interrupt enable
//    CAN->IER |= CAN_IER_LECIE; //Last error code interrupt enable
  }

  void TCAN::set_11_id(T11Id sValue, TMailboxNum MailboxNum)
  {
    struct TIdSetup
	  {
	    uint32_t Mask;
	    uint32_t Position;
		  uint32_t EmptyFlagMask;
	  };

	  constexpr TIdSetup IdSelect[] = 
	  {
	    { CAN_TI0R_STID, CAN_TI0R_STID_Pos, CAN_TSR_TME0 },
	    { CAN_TI1R_STID, CAN_TI1R_STID_Pos, CAN_TSR_TME1 },
	    { CAN_TI2R_STID, CAN_TI2R_STID_Pos, CAN_TSR_TME2 }
	  };

	  struct TIdSettingMask
	  {
	    uint32_t Rtr; //Remote transmission request
	    uint32_t Ide; //Identifier extension
	  };
	  constexpr TIdSettingMask IdSettingMask[] =
	  {
	    {CAN_TI0R_RTR, CAN_TI0R_IDE},
	    {CAN_TI1R_RTR, CAN_TI1R_IDE},
	    {CAN_TI2R_RTR, CAN_TI2R_IDE},
	  };

	  TIdSetup IdSetup         = IdSelect[MailboxNum];
	  TIdSettingMask IdSetting = IdSettingMask[MailboxNum];

    CAN->sTxMailBox[MailboxNum].TIR &= ~IdSetting.Rtr; //Data frame
    CAN->sTxMailBox[MailboxNum].TIR &= ~IdSetting.Ide; //Standard identifier
	  CAN->sTxMailBox[MailboxNum].TIR = (CAN->sTxMailBox[MailboxNum].TIR & ~IdSetup.Mask) 
		                                | ((uint32_t)sValue.Val << IdSetup.Position);
  }

  void TCAN::set_29_id(T29Id sValue, TMailboxNum MailboxNum)
  {
    //реализовать по необходимости
  }

  void TCAN::set_data_len_code(TMailboxNum MailboxNum, uint8_t LenData)
  {
    struct TDlcSetup
	 {
	   uint32_t Mask;
	   uint32_t Position;
		uint32_t EmptyFlagMask;
	 };

	 constexpr TDlcSetup Dlc[] = 
	 {
	   { CAN_TDT0R_DLC, CAN_TDT0R_DLC_Pos, CAN_TSR_TME0 },
	   { CAN_TDT1R_DLC, CAN_TDT1R_DLC_Pos, CAN_TSR_TME1 },
	   { CAN_TDT2R_DLC, CAN_TDT2R_DLC_Pos, CAN_TSR_TME2 }
	 };
	 
	 TDlcSetup DlcSetup = Dlc[MailboxNum];
	 
	 if (LenData > DATA_SIZE)
	 {
	   LenData = DATA_SIZE;
	 }	   
	 CAN->sTxMailBox[MailboxNum].TDTR = (CAN->sTxMailBox[MailboxNum].TDTR & ~DlcSetup.Mask) 
                                       | (LenData << DlcSetup.Position);
  }

  void TCAN::set_data(TMailboxNum MailboxNum, TData TxData)
  {
	  CAN->sTxMailBox[MailboxNum].TDLR = 0U;
	  CAN->sTxMailBox[MailboxNum].TDHR = 0U;
    
    uint8_t TDLR[4] = {0, 0, 0, 0};
    uint8_t TDLH[4] = {0, 0, 0, 0};
    
    uint8_t ix = 0;
    for (auto item : TxData)
    {
      if (ix < 4)
      {
        TDLR[ix] = item;
      }
      else
      {
        TDLH[ix-4] = item;
      }
    
      ++ix;      
    }
    CAN->sTxMailBox[MailboxNum].TDLR = *(uint32_t *)TDLR;
	  CAN->sTxMailBox[MailboxNum].TDHR = *(uint32_t *)TDLH;    
  }

  bool TCAN::check_empty_mailbox(uint32_t EmptyFlagMask)
  {
    return CAN->TSR & EmptyFlagMask;
  }
  
  void TCAN::aux_init()
  {
	  //тактирование
    if (!LL_AHB2_GRP1_IsEnabledClock(CANHw.RsPinClkMask)) //включение тактирования GPIO для порта с Rs CAN, если не включено
    {
      LL_AHB2_GRP1_EnableClock(CANHw.RsPinClkMask);
    }
    
	  if (!LL_AHB2_GRP1_IsEnabledClock(CANHw.LbkPinClkMask)) //включение тактирования GPIO для порта с Lbk CAN, если не включено
    {
      LL_AHB2_GRP1_EnableClock(CANHw.LbkPinClkMask);
    }
	 
	  //выводы
	  LL_GPIO_InitTypeDef GPIO_InitStruct;	 
	  do
    {
      GPIO_InitStruct.Pin = CANHw.RsPinMask;
      switch ( get_rs_type() )
	    {
	      case SN65HVD233D::TPin::_INPUT:
             GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
             GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	 	         break;
	      case SN65HVD233D::TPin::_OUTPUT:
		         //реализовать по необходимости
             GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	 	         break;
	      default:
	 	         break;
	    }
      GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
      GPIO_InitStruct.Alternate = CANHw.RsAlterMask;
    } while ( SUCCESS != LL_GPIO_Init(CANHw.GpioRs, &GPIO_InitStruct) );
	 
	  do
	  {
      GPIO_InitStruct.Pin = CANHw.LbkPinMask;
      switch ( get_lbk_type() )
	    {
	      case SN65HVD233D::TPin::_INPUT:
             GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
             GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	           break;
	      case SN65HVD233D::TPin::_OUTPUT:
	           //реализовать по необходимости
             GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	           break;
	      default:
	           break;
	    }
      GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
      GPIO_InitStruct.Alternate = CANHw.LbkAlterMask;
	  } while ( SUCCESS != LL_GPIO_Init(CANHw.GpioLbk, &GPIO_InitStruct) );
	 
	  switch ( TCANPhy::get_mode() )
	  {
	    case SN65HVD233D::TMode::_NORMAL:
		 	     LL_GPIO_ResetOutputPin(CANHw.GpioLbk, CANHw.LbkPinMask);
		       break;
	    case SN65HVD233D::TMode::_LOOPBACK:
		 	     LL_GPIO_SetOutputPin(CANHw.GpioLbk, CANHw.LbkPinMask);
		       break;
	    default:
		       break;
	  }
	    
	  switch ( TCANPhy::get_rate() )
	  {
	    case SN65HVD233D::TRate::_HIGH_SPEED:
		       //на текущей аппаратной версии с резистором между входом Rs трансивера и микроконтроллером в 3кОм, реализовать этот режим невозможно
		 	     LL_GPIO_ResetOutputPin(CANHw.GpioRs, CANHw.RsPinMask);
		       break;
	    case SN65HVD233D::TRate::_SLOPE_CONTROL:
		 	     LL_GPIO_ResetOutputPin(CANHw.GpioRs, CANHw.RsPinMask);
		       break;
	    case SN65HVD233D::TRate::_STANDBY:
		       //реализовать по необходимости
		 	     LL_GPIO_SetOutputPin(CANHw.GpioRs, CANHw.RsPinMask);
		       break;
	    default:
		       break;
	  }
  }
  
  void TCAN::set_mode(TMode _Mode)
  {
    switch (get_mode())                //В зависимости от действующего режима устанавливаем новый
    {
      case TMode::_INITIALIZATION: //в этом режиме необходимо делать программную инициализацию bxCAN, прием и передача сообщений прекращаются, CANTX в high
           switch (_Mode)
           {
             case TMode::_INITIALIZATION:
                  // режим остается прежним			  
  	              break;
             case TMode::_NORMAL:  					   
  					      //Перед входом в Normal Mode должны были быть сконфигурированы filter scale и mode configuration  	               
  					      CAN->MCR &= ~(CAN_MCR_INRQ | CAN_MCR_SLEEP);
  					      do {} while (CAN->MSR & CAN_MSR_INAK); //ждать сброса INAK бита аппаратной частью
  					      do {} while (CAN->MSR & CAN_MSR_SLAK); //ждать сброса SLAK бита аппаратной частью
  	              break;
             case TMode::_SLEEP:
  	              CAN->MCR = (CAN->MCR & ~CAN_MCR_INRQ) | CAN_MCR_SLEEP; //сбрасываем INRQ, устанавливаем SLEEP
					        do {} while (CAN->MSR & CAN_MSR_INAK);                 //ждать сброса INAK бита аппаратной частью 
					        do {} while (!(CAN->MSR & CAN_MSR_SLAK));              //ждать установки SLAK бита аппаратной частью
  	              break;
             default:
  	              break;
           }
  	       break;
      case TMode::_NORMAL: //перед входом в normal bxCAN всегда должен синхронизироваться с CAN шиной
           switch (_Mode)
           {
             case TMode::_INITIALIZATION:
  	              CAN->MCR = (CAN->MCR & ~CAN_MCR_SLEEP) | CAN_MCR_INRQ; //сбрасываем SLEEP, устанавливаем INRQ
					        do {} while (CAN->MSR & CAN_MSR_SLAK);                 //ждать сброса SLAK бита аппаратной частью 
					        do {} while (!(CAN->MSR & CAN_MSR_INAK));              //ждать установки INAK бита аппаратной частью	
  	              break;
             case TMode::_NORMAL:
                  // режим остается прежним	
  	              break;
             case TMode::_SLEEP:
  	              CAN->MCR = (CAN->MCR & ~CAN_MCR_INRQ) | CAN_MCR_SLEEP; //сбрасываем INRQ, устанавливаем SLEEP
					        do {} while (CAN->MSR & CAN_MSR_INAK);                 //ждать сброса INAK бита аппаратной частью 
					        do {} while (!(CAN->MSR & CAN_MSR_SLAK));              //ждать установки SLAK бита аппаратной частью
  	              break;
             default:
  	              break;
           }
  	       break;
      case TMode::_SLEEP: //после сброса установлен этот режим
           switch (_Mode)
           {
             case TMode::_INITIALIZATION:
  	              CAN->MCR = (CAN->MCR & ~CAN_MCR_SLEEP) | CAN_MCR_INRQ; //сбрасываем SLEEP, устанавливаем INRQ
					        do {} while (CAN->MSR & CAN_MSR_SLAK);                 //ждать сброса SLAK бита аппаратной частью 
					        do {} while (!(CAN->MSR & CAN_MSR_INAK));              //ждать установки INAK бита аппаратной частью
  	              break;
             case TMode::_NORMAL:  			       
  					      //Перед входом в Normal Mode должны были быть сконфигурированы filter scale и mode configuration
  	              CAN->MCR &= ~(CAN_MCR_SLEEP| CAN_MCR_INRQ); //сбрасываем SLEEP, сбрасываем INRQ
  	       		    do {} while (CAN->MSR & CAN_MSR_SLAK);      //ждать сброса SLAK бита аппаратной частью
  					      do {} while (CAN->MSR & CAN_MSR_INAK);      //ждать сброса INAK бита аппаратной частью
  	              break;
             case TMode::_SLEEP:
                  // режим остается прежним	
  	              break;
             default:
                  break;
           }			  
  	       break;
      default:
  	       break;
    }
    
	  Mode = _Mode;
  }
  
  TMode TCAN::get_mode()
  {
    return Mode;
  }
  
  void TCAN::enable_test_mode(TTestMode TestMode)
  {
    if (get_mode() != TMode::_INITIALIZATION)
    {
      set_mode(TMode::_INITIALIZATION); //выбор тестового режима делается только в режиме инициализации
    }
    
    switch (TestMode) //выбор тестового режима
    {
      case TTestMode::_SILENT: //можно принять data frames и remote frames
		                         //отпрвляются на шину только рецессивные биты
										 //доминантный бит перенаправляется CAN ядру, на шину не влияет
										 //используется для анализа траффика на CAN шине
  	        CAN->BTR &= ~CAN_BTR_LBKM;
  	        CAN->BTR |= CAN_BTR_SILM;
  	        break;
      case TTestMode::_LOOP_BACK: //CAN трактует передаваемые сообщения как принятые
		                            //принятые сохраняются в Receive mailbox
											 //Acknowledge errors игнорируются
											 //включена внутренняя обратная связь выхода Tx на вход Rx
  	        CAN->BTR &= ~CAN_BTR_SILM;
  	        CAN->BTR |= CAN_BTR_LBKM;
  	        break;
      case TTestMode::_COMBINED: //"Hot Selftest"
		                           //без взаимодействия с внешней CAN шиной
											//вывод CANTX в рецессивном состоянии 
  	        CAN->BTR |= CAN_BTR_SILM;
  	        CAN->BTR |= CAN_BTR_LBKM;
  	        break;
      default:
  	        break;
    }
    set_mode(TMode::_NORMAL);
  }
  
  void TCAN::disable_all_test_mode()
  {
    //реализовать при необходимости
  }
   
  void TCAN::transmit_req(TMailboxNum MailboxNum)
  {
	 constexpr uint32_t TransmitReqMask[] = {CAN_TI0R_TXRQ, CAN_TI1R_TXRQ, CAN_TI2R_TXRQ};

    CAN->sTxMailBox[MailboxNum].TIR |= TransmitReqMask[MailboxNum]; //сбрасывается аппаратно, когда почтовый ящик становится пустой \
                                                                    _ почтовый ящик с номером MailboxNum переходит в pending \
                                                                   |  ящик ждет установки для него наивысшего приоритета \
                                                         аппаратно |  ящик переходит в scheduled \
                                                         		       |  когда CAN шина становится idle, ящик переходит в transmit => \
                                                         		       |  => запускается передача сообщения \
                                                         		        - ящик переходит в empty, если передан успешно
  }

  void TCAN::transmit_msg(TData TxData)
  {
    //выбрать один empty почтовый ящик передачи	 
    constexpr uint32_t EmptyFlagMask[] = {CAN_TSR_TME0, CAN_TSR_TME1, CAN_TSR_TME2};

    if ( check_empty_mailbox(EmptyFlagMask[Mailbox]) ) //Если для соответствующего почтового ящика нет ожидающего запроса передачи 
	  {
//	    switch (::Device)
//		  {
//		    case TDevice::_PLATF:
//		         switch (MsgType)
//		  		   {
//               case TMsgType::_CHECK_CONNECT:
//		                CurrentId = Id[::Device];
//		                break;
//		  		   }
//		         break;
//		    case TDevice::_HC_1:
//		    case TDevice::_HC_2:
//		         switch (MsgType)
//             {
//               case TMsgType::_DATA_S1:
//                    CurrentId = Id[];
//                    break;
//               case TMsgType::_DATA_S2:
//                    CurrentId = Slave2Data;
//                    break;
//             }
//		         break;
//		    default:
//		         break;
//		  }

      T11Id CurId = Id[::Device];

	    switch (IdType) //установить Id
	    {
	      case TIdType::_STANDARD:
	           set_11_id(CurId, Mailbox);
             break;
	      case TIdType::_EXTENDED:
             break;
	      default:
             break;
	    }

	    set_data_len_code(Mailbox, DATA_SIZE); //установить DLC
	    set_data(Mailbox, TxData);             //установить данные

	    switch (::Device)
		  {
		    case TDevice::_PLATF:
		         CAN->MCR &= ~CAN_MCR_NART;            //включить автоматическое выполнение повторной передачи
		         break;
		    case TDevice::_HC_1:
		    case TDevice::_HC_2:
//	  	       CAN->MCR |= CAN_MCR_NART;            //отключить автоматическое выполнение повторной передачи
		         CAN->MCR &= ~CAN_MCR_NART;            //включить автоматическое выполнение повторной передачи
		         break;
		    default:
		         break;
		  }

		  transmit_req(Mailbox);                       //выполнить запрос на передачу почтового ящика ( TXRQ в CAN_TIxR)

      CAN->IER |= CAN_IER_TMEIE; //разрешить прерывание по установке бита RQCPx

		  xSemaphoreTake(CANTxSemphr, portMAX_DELAY);  //ждать семафор успешной передачи кадра с данными из прерывания \
                                                     можно отправлять следующий кадр с данными
	  }
	  else
	  {
	    //Tx регистры защищены от записи, когда почтовый ящик в pending
	  }
  }

  void TCAN::receive_msg(TData RxData, TRxFifoNum RxFifoNum, uint32_t *Id)
  {
    SemaphoreHandle_t *Semphr = nullptr;
    uint8_t *Fifo             = nullptr;
    uint32_t *_Id              = nullptr;

    switch (RxFifoNum)
    {
      case TRxFifoNum::_NULL:
           Semphr = &CANRx0Semphr;
           Fifo   = Fifo0CanTmp;
           _Id    = &IdFifo0;
           break;
      case TRxFifoNum::_ONE:
           Semphr = &CANRx1Semphr;
           Fifo   = Fifo1CanTmp;
           _Id    = &IdFifo1;
           break;
      default:
           break;
    }

    xSemaphoreTake(*Semphr, portMAX_DELAY); //ждать семафор окончания считывания сообщения (появившегося в FIFO#) из прерывания

    taskENTER_CRITICAL();
      uint8_t ix = 0;
      for (auto &item : RxData)
      {
        item = Fifo[ix++];
      }
      *Id = *_Id;
	  taskEXIT_CRITICAL();
  }
}

//******************************************************************************
//  Обработчики прерываний
//******************************************************************************
extern "C" void CAN1_TX_IRQHandler(void)
{  
  struct TTxSet
  {
    uint32_t Rqcp; //request completed
    uint32_t TxOK; //transmission OK
    uint32_t Alst; //arbitration lost
    uint32_t Terr; //transmission error
  };

  constexpr TTxSet TxSet[] = 
  {
    { CAN_TSR_RQCP0, CAN_TSR_TXOK0, CAN_TSR_ALST0, CAN_TSR_TERR0 },
    { CAN_TSR_RQCP1, CAN_TSR_TXOK1, CAN_TSR_ALST1, CAN_TSR_TERR1 },
    { CAN_TSR_RQCP2, CAN_TSR_TXOK2, CAN_TSR_ALST2, CAN_TSR_TERR2 },
  };
//  RelFour.off();

  if ( (CAN->IER & CAN_IER_TMEIE) && (CAN->TSR & TxSet[Mailbox].Rqcp) ) //если запрос выполнен (transmit или abort)
  {
    if (CAN->TSR & TxSet[Mailbox].TxOK) //если передача выполнена успешно
	 {
      TxHigherPriorityTaskWoken = pdFALSE;
      if (xSemaphoreGiveFromISR(CANTxSemphr, &TxHigherPriorityTaskWoken) == pdFAIL) //отправить семафор успешной передачи кадра с данными
      {
        //семафор уже был доступен, т.е. ранее отдан другой задачей или прерыванием
      } 
      if (TxHigherPriorityTaskWoken == pdTRUE)
      {
        portYIELD_FROM_ISR(TxHigherPriorityTaskWoken);  //принудительное переключение контекста для разблокировки задачи - обработчика
      }
	 }
	 CAN->TSR |= TxSet[Mailbox].Rqcp;   //сброс RQCP, TXOK, ALST и TERR 
  }
  else if (CAN->TSR & TxSet[Mailbox].Alst) //потеря арбитража
  {
    CAN->TSR |= TxSet[Mailbox].Alst;
	 ++CanError[bxCAN::_ALST]; //инкрементировать счетчик ошибок для отладки
  }
  else if (CAN->TSR & TxSet[Mailbox].Terr) //ошибка при передаче
  {
    CAN->TSR |= TxSet[Mailbox].Terr;
	 ++CanError[bxCAN::_TERR]; //инкрементировать счетчик ошибок для отладки
  }
}

extern "C" void CAN1_RX0_IRQHandler(void)
{
//  RelFour.toggle();
//  RelFour.on();
  rx_handler( &FifoCtrl[(uint8_t)bxCAN::TRxFifoNum::_NULL] );
}

extern "C" void CAN1_RX1_IRQHandler(void)
{
  rx_handler( &FifoCtrl[(uint8_t)bxCAN::TRxFifoNum::_ONE] );
}

extern "C" inline void can_error_handler(uint32_t *ErrorBuf, uint8_t Size)
{
  for (uint8_t Ctr = 0; Ctr < Size ; ++Ctr)
  {
    if (ErrorBuf[Ctr] > 50U)
	 {
	   ErrorBuf[Ctr] = 0U; 
      CAN->MSR |= CAN_MSR_ERRI; //сброс флагов
	 }
	 else
	 {
	 
	 }
  }
}

extern "C" void CAN1_SCE_IRQHandler(void)
{
  if (CAN->ESR & CAN_ESR_BOFF && CAN->IER & CAN_IER_BOFIE)
  {
    ++CanError[bxCAN::_BUS_OFF];
	 can_error_handler(CanError, sizeof CanError);	 
//	 for (auto &item : CanError)
//	 {
//	   if (item > 50U)
//	   {
//	     item = 0U;
//		  CAN->MSR |= CAN_MSR_ERRI; //сброс флагов
//	   }
//	 }  
  }
  else if (CAN->ESR & CAN_ESR_LEC && CAN->IER & CAN_IER_LECIE)
  {
    ++LastErrorCode[(CAN->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos];
	 can_error_handler(CanError, sizeof LastErrorCode);
//	 for (auto &item : LastErrorCode)
//	 {
//	   if (item > 50U)
//	   {
//	     item = 0U;
//		  CAN->MSR |= CAN_MSR_ERRI; //сброс флагов
//	   }
//	 }
  }
  else if (CAN->ESR & CAN_ESR_EWGF)
  {
    ++CanError[bxCAN::_ERROR_WARNING];
	 can_error_handler(CanError, sizeof CanError);
//	 for (auto &item : CanError)
//	 {
//	   if (item > 50U)
//	   {
//		  CAN->MSR |= CAN_MSR_ERRI; //сброс флагов
//	     item = 0U;
//	   }
//	 }
  }
  else if (CAN->ESR & CAN_ESR_EPVF)
  {
    ++CanError[bxCAN::_ERROR_PASSIVE];
	 can_error_handler(CanError, sizeof CanError);
//	 for (auto &item : CanError)
//	 {
//	   if (item > 50U)
//	   {
//	     item = 0U;
//		  CAN->MSR |= CAN_MSR_ERRI; //сброс флагов
//	   }
//	 }
  }
  else
  {
    CAN->MSR |= CAN_MSR_ERRI; //сброс флагов
  }
}
//ruct TFifoCtrl
//{
//  __IO uint32_t *RxFifoReg;
//  uint32_t MSG_PEND_MASK;
//  uint8_t (&Buf)[bxCAN::TCAN::DATA_SIZE];
//  uint32_t *pIdFifo;
//  __IO uint8_t *RxFifoData;
//  __IO uint32_t *RxFifoId;
//  uint32_t RELEASE_MAILBOX_MASK;
//  BaseType_t *pHPTaskWoken;
//  SemaphoreHandle_t *Semphr;
//  uint32_t FULL_IE_MASK;
//  uint32_t FIFO_FULL_MASK;
//  uint32_t *FifoWarinig;
//  uint32_t FIFO_OVR_IE_MASK;
//  uint32_t FIFO_OVR_REG_MASK;
//};

//static TFifoCtrl FifoCtrl[(uint8_t)bxCAN::TRxFifoNum::_MAX] =
//{
//  {
//    &CAN->RF0R,
//    CAN_RF0R_FMP0,
//    Fifo0CanTmp,
//    &IdFifo0,
//    (__IO uint8_t *)&CAN->sFIFOMailBox[0].RDLR,
//    &CAN->sFIFOMailBox[0].RIR,
//    CAN_RF0R_RFOM0,
//    &Rx0HigherPriorityTaskWoken,
//    &CANRx0Semphr,
//    CAN_IER_FFIE0,
//    CAN_RF0R_FULL0,
//    Fifo0Warinig,
//    CAN_IER_FOVIE0,
//    CAN_RF0R_FOVR0,
//  },
static void rx_handler(TFifoCtrl *Ctrl)
{
  if ( *Ctrl->RxFifoReg & Ctrl->MSG_PEND_MASK) //если в приемном FIFO0 есть сообщение/сообщения, которые находятся в pending
  {
    uint8_t ix = 0;
    
    for (auto &item : Ctrl->Buf)
    {
      item = Ctrl->RxFifoData[ix++];
    }
    *Ctrl->pIdFifo = (*Ctrl->RxFifoId & CAN_RI0R_STID) >> CAN_RI0R_STID_Pos;
    *Ctrl->RxFifoReg |= Ctrl->RELEASE_MAILBOX_MASK; //освободить почтовый ящик FIFO#
    *Ctrl->pHPTaskWoken = pdFALSE;
    
    if ( xSemaphoreGiveFromISR(*Ctrl->Semphr, Ctrl->pHPTaskWoken) == pdFAIL ) //отправить семафор - сообщение CAN считано
    {
      //семафор уже был доступен, т.е. ранее отдан другой задачей или прерыванием
    }
    if ( *Ctrl->pHPTaskWoken == pdTRUE )
    {
      portYIELD_FROM_ISR( *Ctrl->pHPTaskWoken ); //принудительное переключение контекста для разблокировки задачи - обработчика
    }    
  }
  else if ( ( CAN->IER & Ctrl->FULL_IE_MASK ) && ( *Ctrl->RxFifoReg &  Ctrl->FIFO_FULL_MASK ) )
  {
    *Ctrl->RxFifoReg &= ~Ctrl->FIFO_FULL_MASK;
    ++Ctrl->FifoWarinig[bxCAN::_FULL];
  }
  else if ( ( CAN->IER & Ctrl->FIFO_OVR_IE_MASK ) && ( *Ctrl->RxFifoReg &  Ctrl->FIFO_OVR_REG_MASK ) )
  {
    *Ctrl->RxFifoReg &= ~Ctrl->FIFO_OVR_REG_MASK;
    ++Ctrl->FifoWarinig[bxCAN::_OVR]; //инкремент количества набеганий - FIF0 в pending_3 и пришло новое валидное сообщение => потеря сообщения \
                                        сообщение, которое при этом будет потеряно зависит от конфигурации FIFO
  }
}
