#include <string.h>
#include <algorithm>

#include "rtos_headers.h"
#include "main.h"
#include "lib.h"
#include "Flash_driver.h"
#include "old_flash_lib.h"
#include "discrete_out.h"

uint8_t FlashErrCtr = 0U;

static bool FlashTmrTrig = false;

TFlash Flash;

TFlash::TFlash()
:
Tmr( TIM6 ),
DataPageCtr( 0U )
{
//  rdp_ob();
  init_tmr();
}

TFlash::~TFlash()
{

}

void TFlash::get_sets( uint8_t *Dest, uint16_t BytesQty )
{
  auto Beg = reinterpret_cast<uint8_t *>( FLASH_USER_START_ADDR );
  auto End = reinterpret_cast<uint8_t *>( FLASH_USER_START_ADDR ) + BytesQty;
  
  std::copy( Beg, End, Dest );
}

/*************************************************************
 *
 *  Function:       rdp_ob
 *
 *------------------------------------------------------------
 *
 *  description:    установка бита защиты от считывания (RDP) при первом включении 
 *
 *  parameters:     -
 *
 *  on return:      -
 *
 *************************************************************
 * Список изменений
 * 
 * (d.m.y):
 *   1. 
 *
 ***************************************************/
void TFlash::rdp_ob() const
{
  do {} while( FLASH_GetStatus() == FLASH_BUSY );

  TOptr *Optr = (TOptr *)&FLASH->OPTR;

  if ( Optr->RDP == TRdpVal::OB_RDP_Level_0 ) //если защита от считывания не установлена
  {
    /* Установить защиту от считывания
     */
    FLASH_Unlock();                                                      //разблокировка FPEC
    FLASH_OB_Unlock();                                                   //авторизация программирования OB

    Optr->RDP = TRdpVal::OB_RDP_Level_1;
    
    FLASH->CR |= FLASH_CR_OPTSTRT;                                       //запуск работы опций
    
    do {} while(FLASH_GetStatus() == FLASH_BUSY);
  }
}

//Регулярка для классов
//(\x29;)
//)\n  {\n\n  }\n

void TFlash::write_sets( const uint8_t *Src, uint16_t BytesQty )
{
  do {} while( FLASH_GetStatus() == FLASH_BUSY );
  
  bool ProgStatus = false;
  do
  {
    erase_page( FLASH_USER_START_ADDR );
    ProgStatus = write_page( Src, BytesQty, FLASH_USER_START_ADDR );
  } while ( ProgStatus != true );
}

void TFlash::write_data( const uint8_t *Src )
{
  do {} while( FLASH_GetStatus() == FLASH_BUSY );
  
  bool ProgStatus = false;
  uint32_t Addr = DATA_START_ADDR + PAGE_SIZE * DataPageCtr++;
  do
  {
		
		erase_page( Addr );
    ProgStatus = write_page( Src, PAGE_SIZE, Addr );
  } while ( ProgStatus != true );
}



/* Стирание области Flash
 * Область определяется FLASH_USER_START_ADDR и FLASH_USER_END_ADDR
*/
void TFlash::erase_page( uint32_t Addr ) const
{
  //проверка Flash на незанятость должна быть сделана ранее
  FLASH_Unlock(); //разблокировка Flash для включения доступа к регистру управления Flash

  clear_all_error(); //сброс всех установленных флагов ошибок программирования

  uint32_t PAGE_NBR = ( Addr - FLASH_BASE ) / PAGE_SIZE; //номер страницы
  
  FLASH->CR |= FLASH_CR_PER;
//  FLASH->CR = FLASH->CR & ~FLASH_CR_PNB | ( FLASH_USER_PAGE_NBR << FLASH_CR_PNB_Pos );
  FLASH->CR = FLASH->CR & ~FLASH_CR_PNB | ( PAGE_NBR << FLASH_CR_PNB_Pos );

  FLASH->CR |= FLASH_CR_STRT; //запуск стирания, автоматически включается HSI16
  
  do {} while( FLASH_GetStatus() == FLASH_BUSY ); //вместе с BSY автоматически сбрасывается STRT и отключается HSI16
  FLASH->CR &= ~FLASH_CR_PER;

  FLASH_Lock(); //Блокировка Flash для отключения доступа к регистру управления Flash \
                  (Рекомендуется для защиты Flash от нежелательных операций)
}

bool TFlash::write_page( const uint8_t *Src, uint16_t BytesQty, uint32_t Addr ) const
{
  bool CurProgStatus = false;
  //проверка Flash на незанятость должна быть сделана ранее
  FLASH_Unlock(); //разблокировка Flash для включения доступа к регистру управления Flash
  
  /* Запись во Flash
   * Область определяется FLASH_USER_START_ADDR и FLASH_USER_END_ADDR
  */ 
  uint32_t CurFlashAddr = Addr;
  uint32_t EndFlashAddr = Addr + BytesQty;
    
  //запись настроек
  CurProgStatus = prog( ( const uint32_t * )Src, CurFlashAddr, EndFlashAddr );
  
  FLASH_Lock(); //Блокировка Flash для отключения доступа к регистру управления Flash \
                  (Рекомендуется для защиты Flash от нежелательных операций)
                  
  return CurProgStatus;
}

bool TFlash::prog( const uint32_t *pData, uint32_t StartAddr, const uint32_t EndAddr ) const
{
  
  //проверка Flash на незанятость должна быть сделана ранее
  bool CurProgStatus = true;

  FLASH->CR |= FLASH_CR_EOPIE;

  while ( StartAddr < EndAddr )
  {
    clear_all_error();
    __disable_irq;
    __disable_fault_irq();
    FLASH->CR |= FLASH_CR_PG;
    
    *((__IO uint32_t *)StartAddr)++ = *pData++;
    *((__IO uint32_t *)StartAddr)++ = *pData++;
    
    do {} while( FLASH_GetStatus() == FLASH_BUSY );
    __enable_irq;
    __enable_fault_irq();
    
    start_tmr();
    do 
    {
      if ( FlashTmrTrig == true ) //если время на прошивку элемента Flash истекло
      {
        FlashTmrTrig  = false;
        CurProgStatus = false;
        break;
      }    
    } while( !(FLASH->SR & FLASH_SR_EOP) );   //операция программирования прошла успешно
    FLASH->SR |= FLASH_SR_EOP;                    //сброс флага End of operation
    stop_tmr();
    
//    StartAddr += sizeof(*pData);
  }
  FLASH->CR &= ~FLASH_CR_PG;
  
  FLASH->CR &= ~FLASH_CR_EOPIE;
  
  
  return CurProgStatus;


  
//  //проверка Flash на незанятость должна быть сделана ранее
//  bool CurProgStatus = true;

//  FLASH->CR |= FLASH_CR_FSTPG;
//  
//  clear_all_error();
// __disable_fault_irq();  

//  while ( StartAddr < EndAddr )
//  {
//    
//    *((__IO uint32_t *)StartAddr)++ = *pData++;
//    *((__IO uint32_t *)StartAddr)++ = *pData++;
//    

//    
////    StartAddr += sizeof(*pData);
//  }

//  __enable_fault_irq(); 
//  
//  
//  start_tmr();
//  do {} while( FLASH_GetStatus() == FLASH_BUSY );

//    
//  do 
//  {
//    if ( FlashTmrTrig == true ) //если время на прошивку элемента Flash истекло
//    {
//      FlashTmrTrig  = false;
//      CurProgStatus = false;
//      break;
//    }    
//  } while( !(FLASH->SR & FLASH_SR_EOP) );   //операция программирования прошла успешно
//  
//  
//  FLASH->SR |= FLASH_SR_EOP;                    //сброс флага End of operation
//  stop_tmr();
//  
//  FLASH->CR &= ~FLASH_CR_PG;
//  
//  FLASH->CR &= ~FLASH_CR_EOPIE;
//  
//  
//  return CurProgStatus;





}

void TFlash::clear_all_error() const
{
  FLASH->SR |= FLASH_SR_OPTVERR | //сброс всех установленных флагов ошибок программирования
               FLASH_SR_RDERR   |
               FLASH_SR_FASTERR |
               FLASH_SR_MISERR  |
               FLASH_SR_PGSERR  |
               FLASH_SR_SIZERR  |
               FLASH_SR_PGAERR  |
               FLASH_SR_WRPERR  |
               FLASH_SR_PROGERR |
               FLASH_SR_OPERR   |
               FLASH_SR_EOP;
}

void TFlash::start_tmr() const
{
  LL_TIM_EnableIT_UPDATE( Tmr );  // Enable update interrupt
  LL_TIM_EnableCounter( Tmr );    // Enable timer counter
}                                 
                                  
void TFlash::stop_tmr() const           
{                                 
  LL_TIM_DisableIT_UPDATE( Tmr ); // Disable update interrupt
  LL_TIM_DisableCounter( Tmr );
}


void TFlash::init_tmr()
{    
    LL_TIM_InitTypeDef TIM_InitStruct;

//  ----- Включить тактирование таймера ---------------------------------
    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_TIM6 );

    LL_RCC_ClocksTypeDef RCC_Clocks;
    LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
//    RCC_Clocks.PCLK1_Frequency

    constexpr uint32_t F_CNT             = 2000U; //частота работы счетчика таймера, Гц
    constexpr uint32_t F_TIMER           = 100U;  //частота работы таймера, Гц
    uint16_t PRESCALER_VALUE             = (RCC_Clocks.PCLK1_Frequency + (F_CNT / 2U)) / F_CNT - 1U;
    constexpr uint32_t AUTORELOAD_VALUE  = (F_CNT + (F_TIMER / 2U)) / F_TIMER - 1U;
    constexpr uint32_t START_TIMER_VALUE = 0x00;

//  ----- Инициализация таймера -----------------------------------------
    TIM_InitStruct.Prescaler     = PRESCALER_VALUE;
    TIM_InitStruct.CounterMode   = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload    = AUTORELOAD_VALUE;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
//    TIM_InitStruct.RepetitionCounter = ;

    LL_TIM_Init(Tmr, &TIM_InitStruct);                            // Configure the TIMx time base unit
    LL_TIM_ClearFlag_UPDATE(Tmr);

//    LL_TIM_DisableUpdateEvent(MAIN_TIMER);                        // Enable update event generation 
    LL_TIM_SetUpdateSource(Tmr, LL_TIM_UPDATESOURCE_COUNTER);     // Set update event source  
    LL_TIM_SetOnePulseMode(Tmr, LL_TIM_ONEPULSEMODE_REPETITIVE);  // Set one pulse mode
    LL_TIM_SetCounterMode(Tmr, LL_TIM_COUNTERMODE_UP);            // Set the timer counter counting mode
    LL_TIM_DisableARRPreload(Tmr);                                // Enable auto-reload (ARR) preload
    LL_TIM_SetCounter(Tmr, START_TIMER_VALUE);                    // Set the counter value

    //настройка NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));  // 5 - максимальный уровень приоритета для прерывания из которого можно вызывать API функции FreeRTOS 
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

//******************************************************************************
//  Обработчики прерываний
//******************************************************************************
extern "C" void FLASH_IRQHandler( void )
{
  volatile static uint8_t Ctr = 0;
  Ctr++;
}

extern "C" void TIM6_DAC_IRQHandler(void)
{
  if ( LL_TIM_IsActiveFlag_UPDATE( TIM6 ) )
  {
    FlashTmrTrig = true;
    ++FlashErrCtr;
    LL_TIM_ClearFlag_UPDATE( TIM6 ); // Clear the update interrupt flag (UIF)
  }
}
