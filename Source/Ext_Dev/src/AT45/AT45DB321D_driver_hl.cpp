#include <algorithm>

#include "AT45DB321D_driver_hl.h"
#include "rtos_headers.h"

using CMD = AT45DB321D_DRIVER_LL;

static uint8_t RxBytesQty;
static uint8_t RxByte;

static BaseType_t TxHigherPriorityTaskWoken;
static BaseType_t RxHigherPriorityTaskWoken;

AT45DB321D_DRIVER_HL::AT45DB321D_DRIVER_HL( 
                                           const TSpi_HW &_HW,
                                           TRate _Rate
                                          )
:
AT45DB321D_DRIVER_LL(),
TSPI( _HW, _Rate )
{

}

AT45DB321D_DRIVER_HL::~AT45DB321D_DRIVER_HL()
{

}

void AT45DB321D_DRIVER_HL::init_driver()
{
  pin_clk_config();
  hw_init();
}

bool AT45DB321D_DRIVER_HL::check_chip()
{
  //необходимо записать один байт команды и ждать 1 байт в ответ
  while ( write_byte( CMD::Additional::__STATUS_RD ) != TSpiState::__START ){}
//  xSemaphoreTake( TxSPI1_RdySem, portMAX_DELAY ); //waiting external interrupt  
  LL_SPI_EnableIT_RXNE( HW.If ); //включить прерывание RXNE
  
  //ожидание одного принятого байта
//  xSemaphoreTake( RxSPI1_RdySem, portMAX_DELAY ); //waiting external interrupt  
  LL_SPI_DisableIT_RXNE( HW.If ); //отключить прерывание RXNE
  
  LL_GPIO_SetOutputPin( HW.CS.Gpio, HW.CS.Nbr );
  LL_SPI_Disable( HW.If ); //Disable SPI peripheral

  return true;
}

void AT45DB321D_DRIVER_HL::write( const uint8_t *Buf, uint8_t Size )
{
  if ( Size == 1U || Size == 0U ) //для записи подобных буферов функция не исползуется
  {
    return ;
  }
  
  uint16_t NewIx = Size + Tx.Ix;
  
  if ( NewIx >= __BUF_SIZE ) //если новые данные переполнят буфер
  {
    //записать во флэш то, что на данный момент есть в буфере
    Tx.Buf[ 0 ] = CMD::Program_Erase::__MM_PAGE_PROGRAM_BUF_1;
    write_frame( Tx.Buf, Tx.Ix + 1U );
    
    //очистить буфер
    auto beg = reinterpret_cast<uint8_t *>( Tx.Buf );
    auto end = reinterpret_cast<uint8_t *>( &Tx.Buf[ __BUF_SIZE ] );
    
    std::fill( beg, end, 0U ); //обнуление массива с выборками
    
    //записать в пустой буфер новую запись
    auto End = reinterpret_cast<const uint8_t *>( &Buf[ Size ] );
    std::copy( Buf, End, reinterpret_cast<uint8_t *>(Tx.Buf) );
    Tx.Ix = Size - 1U;
  }
  else
  {
    //записать в буфер новую запись
    auto beg = reinterpret_cast<const uint8_t *>( Tx.Buf[ Tx.Ix ] );
    auto end = reinterpret_cast<const uint8_t *>( &Buf[ NewIx + 1U ] );
    std::copy( beg, end, reinterpret_cast<uint8_t *>( Tx.Buf ) );
    Tx.Ix = NewIx;
  }
}

void AT45DB321D_DRIVER_HL::read( uint8_t *Buf, uint8_t Size )
{
  
}

//TxHigherPriorityTaskWoken
//RxHigherPriorityTaskWoken

//обработчик прерываний SPI
extern "C" void SPI1_IRQHandler( void )
{
//  if ( LL_SPI_IsActiveFlag_TXE( SPI1 ) && LL_SPI_IsEnabledIT_TXE( SPI1 ) )
//  {
//    LL_SPI_DisableIT_TXE( SPI1 );  //отключить прерывание TXE

//    TxHigherPriorityTaskWoken = pdFALSE;
//    if (xSemaphoreGiveFromISR( TxSPI1_RdySem, &TxHigherPriorityTaskWoken ) == pdFAIL) //отправить семафор окончания записи
//    {

//    }  
//    if ( TxHigherPriorityTaskWoken == pdTRUE )
//    {
//      portYIELD_FROM_ISR( TxHigherPriorityTaskWoken ); //принудительное переключение контекста для разблокировки задачи - обработчика
//                                                       //максимально быстро перейти к считыванию данных с датчиков микросхемы MPU-9250
//                                                       //для FreeRTOS время от выдачи семафора до перехода к задаче на stm32f3 - 7мкс
//    }
//  }
//  
//  if ( LL_SPI_IsActiveFlag_RXNE( SPI1 ) && LL_SPI_IsEnabledIT_RXNE( SPI1 ) )
//  {
//    RxByte = LL_SPI_ReceiveData8( SPI1 );

//    RxHigherPriorityTaskWoken = pdFALSE;
//    if (xSemaphoreGiveFromISR( RxSPI1_RdySem, &RxHigherPriorityTaskWoken ) == pdFAIL) //отправить семафор окончания записи
//    {

//    }  
//    if ( RxHigherPriorityTaskWoken == pdTRUE )
//    {
//      portYIELD_FROM_ISR( RxHigherPriorityTaskWoken ); //принудительное переключение контекста для разблокировки задачи - обработчика
//                                                       //максимально быстро перейти к считыванию данных с датчиков микросхемы MPU-9250
//                                                       //для FreeRTOS время от выдачи семафора до перехода к задаче на stm32f3 - 7мкс
//    }
//  }
//  
//  if ( LL_SPI_IsEnabledIT_ERR( SPI1 ) )
//  {
//    if ( LL_SPI_IsActiveFlag_OVR( SPI1 ) )
//    {
//      LL_SPI_ClearFlag_OVR( SPI1 );
//    }
//    if ( LL_SPI_IsActiveFlag_MODF( SPI1 ) )
//    {
//      LL_SPI_ClearFlag_MODF( SPI1 );
//    }
//  }
    
}

