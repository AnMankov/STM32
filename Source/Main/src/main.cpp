/* Includes ------------------------------------------------------------------*/
#include <cassert>
#include <algorithm>

#include "rtos_headers.h"
#include "main.h"
#include "lib.h"
#include "interface.h"
#include "SensProc.h"
#include "SensDev.h"
#include "prog_ver.h"

//#include "dev_determ.h"
//#include "coding_switch.h"
//#include "discrete_out.h"
//#include "mems.h"
#include "usart_driver_.h"


//******************************************************************************
//  Секция определения переменных, используемых в модуле
//******************************************************************************

uint32_t RtosHeapSize = xPortGetFreeHeapSize();

//uint8_t DevTypeTmrTrigCtr = 0U;    //счетчик срабатываний таймера определения типа устройства
//bool DevTypeTmrFlag       = false; //флаг - прерывание таймера определения типа устройства произошло

//bool MagCalReq  = false;           //запрос на калибровку магнитометра, принятый через интерфейсы

//uint32_t MagCalDataSize; //размер буфера калибровочных данных магнитометра
//constexpr uint8_t DEF_MAG_CAL_SIZE = 50U;
//uint8_t MagCalBuf[DEF_MAG_CAL_SIZE];

//---- Экземпляры Светодиодов -------------------------------------
//-----------------------------------------------------------------

//---- Экземпляры Реле --------------------------------------------
//const relay::TRelay RelOne{
//                            GPIOB,
//                            LL_AHB2_GRP1_PERIPH_GPIOB,
//							              LL_GPIO_PIN_5,
//							              "RELAY_ONE"
//							            };
//const relay::TRelay RelTwo{
//                            GPIOB,
//                            LL_AHB2_GRP1_PERIPH_GPIOB,
//							              LL_GPIO_PIN_6,
//							              "RELAY_TWO"
//							            };
//const relay::TRelay RelThree{
//                              GPIOB,
//                              LL_AHB2_GRP1_PERIPH_GPIOB,
//							                LL_GPIO_PIN_8,
//							                "RELAY_THREE"
//							              };
//const relay::TRelay RelFour{
//                             GPIOB,
//                             LL_AHB2_GRP1_PERIPH_GPIOB,
//							               LL_GPIO_PIN_9,
//							               "RELAY_FOUR"
//						               };

//#ifdef __SENSOR__
//  const relay::TRelay DOUT1{
//                             GPIOB,
//                             LL_AHB2_GRP1_PERIPH_GPIOB,
//  							             LL_GPIO_PIN_14,
//  							             "RELAY_ONE"
//  							           };
//#endif

//-----------------------------------------------------------------


//USART::TUSART Usart(
//                    USART2, 
//                    USART::TMode::_RS485_DE,
//                    TProc::_DKS_TO_PC
//                   );

//USART::TUSART Usart(
//                    USART2, 
//                    USART::TMode::_RS485_DE,
//                    USART::TProc::_AUTO
//                   );



//******************************************************************************
//  Секция определений объектов RTOS
//******************************************************************************

//TDevice Device; //тип устройства: master, slave1, slave2

//----- Дескрипторы задач ------------------------------------------------------
//------------------------------------------------------------------------------

//----- Дескрипторы очередей ---------------------------------------------------
//------------------------------------------------------------------------------

//----- Дескрипторы семафоров --------------------------------------------------
SemaphoreHandle_t I2C_RxSem;            //данные считаны по I2C
SemaphoreHandle_t I2C_TxSem;            //данные записаны по I2C
SemaphoreHandle_t CodeSwTmr_TrigSem;    //таймер CodeSwTmr сработал
SemaphoreHandle_t DoUncalibTmr_TrigSem; //таймер DoUncalibTmr сработал                                    
SemaphoreHandle_t RawDataMems_RdySem;   //данные датчика готовы (можно считывать). из прерывания от INT
SemaphoreHandle_t SlaveRtoTrigSem;      //для слейва: сработал флаг RTO для Slave-устройства в USART
SemaphoreHandle_t SlaveCommErrSem;      //для слейва: есть ошибка/ошибки связи по USART для Slave-устройства
SemaphoreHandle_t MasterRtoTrigSem;     //для мастера: сработал флаг RTO для Slave-устройства в USART
SemaphoreHandle_t MasterCommErrSem;     //для мастера: есть ошибка/ошибки связи по USART для Slave-устройства
SemaphoreHandle_t PdSem;                //тестовый периодический опрос СЕНС ПД
SemaphoreHandle_t WrFlash;              //команда на запись данных во Flash
SemaphoreHandle_t BtnTimer_TrigSem;     //
SemaphoreHandle_t LedTimer_TrigSem;     //
SemaphoreHandle_t DmaAdcMeas_CompSem;   //

SemaphoreHandle_t Tmr_TrigSem;
SemaphoreHandle_t DiffExti_TrigSem;
//------------------------------------------------------------------------------

//----- Дескрипторы мьютексов --------------------------------------------------
SemaphoreHandle_t MainMut;                  //мьютекс буфера, хранящего основные данные усройства
SemaphoreHandle_t SENSMut;                  //мьютекс данных протокола СЕНС
//------------------------------------------------------------------------------

//----- Дескрипторы таймеров ---------------------------------------------------
TimerHandle_t CodeSwTmr;                    //таймер обработки кодового переключателя
TimerHandle_t DoUncalibTmr;                 //таймер обработки дискретного выхода при некалиброванном акселерометре
TimerHandle_t PdTmr;                        //тестовый таймер
//TimerHandle_t MemsTmr;                    //таймер для нужд задачи обработки Mems-датчика

TimerHandle_t BtnTimer;
//------------------------------------------------------------------------------

//******************************************************************************
//  Секция объявлений функций (declaration)
//******************************************************************************
static void init_ll();
void cfg_sys_clk();
void err_handler();
static void init_mx_gpio();

static void create_semphrs();
static void create_mutexes();
static void create_tasks();
static void create_timers();
static void check_create_task( portBASE_TYPE );
static void check_create_mutex( xSemaphoreHandle );
static void check_create_semphr( xSemaphoreHandle );
static void check_create_timer( TimerHandle_t );
//------------------------------------------------------------------------------

//******************************************************************************
//  Секция определений функций (definition)
//******************************************************************************

int main(void)
{
  init_ll();
  cfg_sys_clk();
  
  init_mx_gpio();

//----- Создание семафоров ----------------------------------------------------------------------------------------------------
  RtosHeapSize = xPortGetFreeHeapSize(); //размер кучи до создания семафоров
  create_semphrs();
//-----------------------------------------------------------------------------------------------------------------------------

//----- Создание мьютексов ----------------------------------------------------------------------------------------------------
  RtosHeapSize = xPortGetFreeHeapSize(); //размер кучи до создания мьютексов
  create_mutexes();
//-----------------------------------------------------------------------------------------------------------------------------

//----- Создание таймеров ----------------------------------------------------------------------------------------------------- 
  RtosHeapSize = xPortGetFreeHeapSize(); //размер кучи до создания таймеров
  create_timers();
//-----------------------------------------------------------------------------------------------------------------------------

//----- Создание задач --------------------------------------------------------------------------------------------------------
  RtosHeapSize = xPortGetFreeHeapSize(); //размер кучи до создания задач
  create_tasks();
  RtosHeapSize = xPortGetFreeHeapSize(); //размер кучи после создания задач
//-----------------------------------------------------------------------------------------------------------------------------


  vTaskStartScheduler(); //запуск планировщика + автоматическое создание задачи бездействие (Idle task)

  for (;;)
  {

  }
}

static void create_semphrs()
{
  check_create_semphr( I2C_RxSem               = xSemaphoreCreateBinary() );
  check_create_semphr( I2C_TxSem               = xSemaphoreCreateBinary() );
  check_create_semphr( CodeSwTmr_TrigSem       = xSemaphoreCreateBinary() );
  check_create_semphr( DoUncalibTmr_TrigSem    = xSemaphoreCreateBinary() );
                                               
  check_create_semphr( RawDataMems_RdySem      = xSemaphoreCreateBinary() );
                                               
  check_create_semphr( SlaveRtoTrigSem         = xSemaphoreCreateBinary() );
  check_create_semphr( SlaveCommErrSem         = xSemaphoreCreateBinary() );
  check_create_semphr( MasterRtoTrigSem        = xSemaphoreCreateBinary() );
  check_create_semphr( MasterCommErrSem        = xSemaphoreCreateBinary() );
  check_create_semphr( PdSem                   = xSemaphoreCreateBinary() );
  check_create_semphr( WrFlash                 = xSemaphoreCreateBinary() );
  check_create_semphr( BtnTimer_TrigSem        = xSemaphoreCreateBinary() );
  check_create_semphr( LedTimer_TrigSem        = xSemaphoreCreateBinary() );
  
  check_create_semphr( Tmr_TrigSem             = xSemaphoreCreateBinary() );
  check_create_semphr( DiffExti_TrigSem        = xSemaphoreCreateBinary() );
  check_create_semphr( DmaAdcMeas_CompSem      = xSemaphoreCreateBinary() );  
}

static void create_mutexes()
{  
  MainMut = xSemaphoreCreateMutex();
  check_create_mutex(MainMut);
  
  SENSMut = xSemaphoreCreateMutex();
  check_create_mutex(SENSMut);
}


static void create_tasks()
{
  constexpr uint16_t LEV_GAUGE_TO_PC_TASK_STACK_SIZE_B = 1000U;
  constexpr uint16_t LEV_GAUGE_TO_PC_TASK_STACK_SIZE_W = LEV_GAUGE_TO_PC_TASK_STACK_SIZE_B / 4U;
  check_create_task( xTaskCreate(lev_gauge_to_pc,
                     (char *)"LEV_GAUGE_TO_PC",
							       LEV_GAUGE_TO_PC_TASK_STACK_SIZE_W,
							       NULL,
							       4,
							       NULL));
  RtosHeapSize = xPortGetFreeHeapSize();
}

static void create_timers()
{
//  check_create_timer( CodeSwTmr = xTimerCreate(
//                                                "CodeSwTmr",
//							                                  pdMS_TO_TICKS( TCodingSwitch::SAMPLE_TIME_MS ),
//							                                  pdFALSE,      //интервальный таймер
//							                                  ( void * ) 0, //идентификатор не нужен, т.к. создаем только один экземпляр таймера
//							                                  code_sw_tmr
//											                        )
//                    );
}

extern "C" void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName );

//функция - ловушка задачи переполнения стека
//подробности https://www.freertos.org/Stacks-and-stack-overflow-checking.html
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
  static uint32_t k = 0;
  ++k;
}

void vApplicationIdleHook( void )
{
//  RelTwo.on();
//  RelTwo.off();
}


static void check_create_task(portBASE_TYPE TaskStatus)
{
  if (TaskStatus != pdTRUE)
  {
    //недостаточно памяти кучи
  }
}

static void check_create_mutex(xSemaphoreHandle MutexStatus)
{
  if (MutexStatus == NULL)
  {
    //недостаточно памяти кучи
  }
}

static void check_create_semphr(xSemaphoreHandle SemphrStatus)
{
  if (SemphrStatus == NULL)
  {
    //недостаточно памяти кучи
  }
}

static void check_create_timer(TimerHandle_t TimerHandle)
{
  if (TimerHandle == NULL)
  {
    //недостаточно памяти кучи FreeRTOS для успешного размещения структур данных таймера
  }
}

//----- Инициализация -------------------------------------------------------
static void init_ll()
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
}

/** System Clock Configuration
*/
void cfg_sys_clk()
{
  struct TFreqRange
  {
    uint8_t Min;
    uint8_t Max;
  };

  constexpr TFreqRange FreqRange[TClkSrc::_MAX][TVcoreRange::_MAX] =
  {
    // Range 1       Range 2
    { { 26U, 48U }, { 0U, 26U } }, //_HSE
    { {  0U, 16U }, { 0U, 16U } }, //_HSI16
    { { 24U, 48U }, { 0U, 24U } }, //_MSI
    { { 26U, 80U }, { 0U, 26U } }  //_PLL
  };

  //After reset, the CPU clock frequency is 4 MHz and 0 wait state (WS)
  constexpr uint32_t HCLK             = HSE_VALUE / 1000000U; //MHz
//  constexpr uint32_t HCLK             = 32; //MHz
  constexpr TClkSrc::T ClkSrc = TClkSrc::_HSE;
//  constexpr TClkSrc::T ClkSrc = TClkSrc::_PLL;
  TVcoreRange::T VcoreRange;
  TLatency::T    Latency;

  assert( HCLK <= 80U ); //максимально допустимая частота для STM32L431CCT6
 
  for ( TVcoreRange::T ctr = TVcoreRange::_MIN; ctr < TVcoreRange::_MAX; ++(uint8_t)ctr )
  {
    if (
        HCLK >= FreqRange[ClkSrc][ctr].Min
        &&
        HCLK <= FreqRange[ClkSrc][ctr].Max
       )
    {
      VcoreRange = ctr;
      break;
    }    
  }
  
  constexpr TFreqRange FreqRangeWs[TVcoreRange::_MAX][TLatency::_MAX] =
  {
    //  _0_WS        _1_WS         _2_WS         _3_WS         _4_WS
    { { 6U, 16U }, { 12U, 32U }, { 18U, 48U }, { 26U, 64U }, { 26U, 80U } }, //Range 1
    { { 0U,  6U }, {  0U, 12U }, {  0U, 18U }, {  0U, 26U }, {  0U, 26U } }, //Range 2
  };
  
  for ( TLatency::T ctr = TLatency::_MIN; ctr < TLatency::_MAX; ++(uint8_t)ctr )
  {
    if (
        HCLK >= FreqRangeWs[VcoreRange][ctr].Min
        &&
        HCLK <= FreqRangeWs[VcoreRange][ctr].Max
       )
    {
      Latency = ctr;
      break;
    }
  }

  uint32_t LatMask[] =
  {
    LL_FLASH_LATENCY_0,
    LL_FLASH_LATENCY_1,
    LL_FLASH_LATENCY_2,
    LL_FLASH_LATENCY_3,
    LL_FLASH_LATENCY_4,
  };
  
  uint32_t VsMask[] =
  {
    LL_PWR_REGU_VOLTAGE_SCALE1,
    LL_PWR_REGU_VOLTAGE_SCALE2,
  };
    
  do { LL_FLASH_SetLatency( LatMask[Latency] ); } while ( LL_FLASH_GetLatency() != LatMask[Latency] );
  
  LL_PWR_SetRegulVoltageScaling( VsMask[VcoreRange] );


  //Voltage scaling range selection по умолчанию в Range 1 - до 80МГц

  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }

  LL_RCC_MSI_Disable();
  
  do {} while ( LL_RCC_MSI_IsReady() != 1);
  
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_7);
  LL_RCC_MSI_Enable();
  
  do {} while ( LL_RCC_MSI_IsReady() != 1);

  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  
//  HSITRIM: The default value is 64 when added to the HSICAL value, trim the HSI16 to 16 MHz ± 1 %
//  LL_RCC_HSI_SetCalibTrimming(16);

  if ( ClkSrc == TClkSrc::_PLL )
  {

//#ifndef __DEBUG__
//  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_2, LL_RCC_PLL_MUL_12);
//#else
//  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
//#endif

    LL_RCC_PLL_EnableDomain_SYS();
    
    constexpr uint32_t PLLN = 8U; //Main PLL multiplication factor for VCO (8..86)
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, PLLN, LL_RCC_PLLR_DIV_2); //f(PLL_R) = 26МГц
    
    LL_RCC_PLL_Enable();
    
     /* Wait till PLL is ready */
    while(LL_RCC_PLL_IsReady() != 1)
    {
    
    }
  }

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  struct TSrc
  {
    uint32_t Src;
    uint32_t Status;
  };

  constexpr TSrc ClkSrcArr[] =
  {
    { LL_RCC_SYS_CLKSOURCE_HSE, LL_RCC_SYS_CLKSOURCE_STATUS_HSE },
    { LL_RCC_SYS_CLKSOURCE_HSI, LL_RCC_SYS_CLKSOURCE_STATUS_HSI },
    { LL_RCC_SYS_CLKSOURCE_MSI, LL_RCC_SYS_CLKSOURCE_STATUS_MSI },
    { LL_RCC_SYS_CLKSOURCE_PLL, LL_RCC_SYS_CLKSOURCE_STATUS_PLL },
  };
  
  LL_RCC_SetSysClkSource( ClkSrcArr[ClkSrc].Src );

   /* Wait till System clock is ready */
   
  uint32_t SysClkSource = 0U;
  while( ( SysClkSource = LL_RCC_GetSysClkSource() ) != ClkSrcArr[ClkSrc].Status )
  {

  }

#ifndef __DEBUG__
  LL_Init1msTick(12000000);
#else
  LL_Init1msTick(64000000);
#endif

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

#ifndef __DEBUG__
  LL_SetSystemCoreClock(HSE_VALUE);
#else
  LL_SetSystemCoreClock(64000000);
#endif
  
  LL_RCC_ConfigMCO( LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_1 );

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
}

static void init_mx_gpio()
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  GPIO_InitStruct.Pin  = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin        = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate  = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

//extern PRIVILEGED_DATA TCB_t * volatile pxCurrentTCB;
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
  static uint8_t Ctr;
  ++Ctr;
} 


void err_handler()
{
  while(1) 
  {
  
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}


#endif

//  uint8_t Arr[] =
//  { 2U, 5U, 4U, 10U, 5U };
//  constexpr uint8_t ArrLen = sizeof Arr / sizeof Arr[0];
//  
//  uint8_t ArrCopy[ArrLen] = {};
//  
//  auto *beg = Arr;
//  auto *end = &Arr[ArrLen];
//  
//  std::for_each( beg, end, [](uint8_t &item){
//    item *= 3U;
//  } );
//  
////  beg = ArrCopy;
////  end = &ArrCopy[ArrLen];
//  
//  static uint8_t *end_cpy;
//  end_cpy =
//  std::remove_copy_if( beg, end, ArrCopy, [](uint8_t &item){
//    return item == 15U;
//  } );

//  static uint8_t Cnt = 0U;

//  Cnt = std::count( beg, end, 15U );
//  Cnt =
//  std::count_if( beg, end, [](const uint8_t &item){
//    return item % 2U == 0U;
//  } );
