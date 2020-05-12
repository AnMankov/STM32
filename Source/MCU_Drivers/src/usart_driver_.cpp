
#include <algorithm>
#include "usart_driver_.h"
//#include "discrete_out.h"

const TUsart::TSets TUsart::DEF_SETS = {
                                        TBaudRate::_19200,
                                        TParity::_EVEN,
                                        TStops::_STOPBITS_1
                                       };
                            
TUsart::TUsart(
               const TUsart_HW &_HW,
               const TSets &_Sets,
               TMode _Mode
              )
:
HW(_HW),
Sets( _Sets ),
Mode( _Mode )
{
  
}

TUsart::~TUsart()
{

}

void TUsart::pin_clk_config()
{
  HW.TX.en_clk( HW.TX.ClkPortMask ); //включение тактирования GPIO для порта с Tx
  HW.RX.en_clk( HW.RX.ClkPortMask ); //включение тактирования GPIO для порта с Rx
  HW.DE.en_clk( HW.DE.ClkPortMask ); //включение тактирования GPIO для порта с DE

  HW.Clk.en_periph( HW.Clk.PeriphMask ); //включение тактирования USART'a
  
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  
  do
  {
    GPIO_InitStruct.Pin        = HW.TX.Nbr;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_7;
  } while ( SUCCESS != LL_GPIO_Init( HW.TX.Gpio, &GPIO_InitStruct ) );
  do
  {
    GPIO_InitStruct.Pin        = HW.RX.Nbr;
  } while ( SUCCESS != LL_GPIO_Init( HW.RX.Gpio, &GPIO_InitStruct ) );
  do
  {
    GPIO_InitStruct.Pin        = HW.DE.Nbr;
  } while ( SUCCESS != LL_GPIO_Init( HW.DE.Gpio, &GPIO_InitStruct ) );

  LL_RCC_SetUSARTClockSource( HW.Clk.SrcMask ); //The clock source must be chosen before enabling the USART \
                                                  (by setting the UE bit)
}

void TUsart::hw_init( uint8_t TimeoutBitsQty )
{  
  struct TParityCtrl
	{
	  uint32_t ParityMask;
		uint32_t StopsMask;
		uint32_t DataWidth;
	};

	TParityCtrl ParityCtrl[TParity::_MAX_PARITY][TStops::_MAX_STOPS] =
	{
	  {
		  {
			  LL_USART_PARITY_NONE, LL_USART_STOPBITS_1, LL_USART_DATAWIDTH_8B
			},
		  {
			  LL_USART_PARITY_NONE, LL_USART_STOPBITS_2, LL_USART_DATAWIDTH_8B
			}
		},
		{
		  {
			  LL_USART_PARITY_EVEN, LL_USART_STOPBITS_1, LL_USART_DATAWIDTH_9B
			},
		  {
			  LL_USART_PARITY_EVEN, LL_USART_STOPBITS_2, LL_USART_DATAWIDTH_8B
			}		
		},
		{
		  {
			  LL_USART_PARITY_ODD, LL_USART_STOPBITS_1, LL_USART_DATAWIDTH_9B
			},
		  {
			  LL_USART_PARITY_ODD, LL_USART_STOPBITS_2, LL_USART_DATAWIDTH_8B
			}		
		}
	};
  
  LL_USART_InitTypeDef USART_InitStruct;

  USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;
//  USART_InitStruct.DataWidth           = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.DataWidth           = ParityCtrl[ Sets.Parity ][ Sets.Stops ].DataWidth;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
//  USART_InitStruct.Parity              = ParityMask[Sets.Parity];                          //значение из данных объекта
  USART_InitStruct.Parity              = ParityCtrl[ Sets.Parity ][ Sets.Stops ].ParityMask; //значение из данных объекта
//  USART_InitStruct.StopBits            = StopsMask[Sets.Stops];                            //значение из данных объекта
  USART_InitStruct.StopBits            = ParityCtrl[ Sets.Parity ][ Sets.Stops ].StopsMask;  //значение из данных объекта
//  USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;                         //Transmitter and Receiver are enabled
  USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_NONE;                            //Transmitter and Receiver are disabled

  LL_USART_Init( HW.If, &USART_InitStruct );

  uint32_t Fck = LL_RCC_GetUSARTClockFreq( HW.Clk.FreqMask );
  uint32_t OvS = LL_USART_GetOverSampling( HW.If );
    
  LL_USART_SetTransferBitOrder( HW.If, LL_USART_BITORDER_LSBFIRST ); 
  LL_USART_ClearFlag_TC( HW.If );
  
  if ( IS_UART_DRIVER_ENABLE_INSTANCE( HW.If ) )
  {
    switch ( Mode )
    {
      case TMode::_RS485_DE:
           switch ( get_config() )
           {
             case ( TConfigurable::_HALF_DUPLEX ):
  	              set_485_hd(Fck, OvS);
    	            break;
             case ( TConfigurable::_FULL_DUPLEX ):
    	      	    break;
             default:
                  break;
           }     
  	       break;
  	  case TMode::_STANDARD:
           set_232(Fck, OvS);
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
  
  LL_USART_Enable( HW.If ); //Idle кадр отправляется после установки бита TE
  clr_error_flags();
}

void TUsart::clr_error_flags()
{
  LL_USART_ClearFlag_PE( HW.If );
  LL_USART_ClearFlag_FE( HW.If );
  LL_USART_ClearFlag_NE( HW.If );
  LL_USART_ClearFlag_ORE( HW.If );
}

void TUsart::dis_if()
{
  LL_USART_Disable( HW.If );
}

void TUsart::set_485_hd( uint32_t Fck, uint32_t OvS )
{
  LL_USART_EnableDEMode( HW.If );                                   //включается внутренняя подтяжка на Rx
  LL_USART_SetBaudRate( HW.If, Fck, OvS, (uint32_t)Sets.BaudRate ); //значение из данных объекта
  LL_USART_SetDEDeassertionTime( HW.If, MAX_DEDT );
  LL_USART_SetDEAssertionTime( HW.If, MAX_DEAT );
  LL_USART_SetDESignalPolarity( HW.If, LL_USART_DE_POLARITY_HIGH );
}
  
void TUsart::set_232( uint32_t Fck, uint32_t OvS )
{
  LL_USART_SetBaudRate( HW.If, Fck, OvS, (uint32_t)Sets.BaudRate );
}

void TUsart::en_eob_detect( uint8_t Timeout )
{
  //----- Настройка определения конца блока по программируемому таймауту ------------------------------------
  LL_USART_EnableRxTimeout( HW.If ); //включить функцию определения конца блока по программируемому таймауту
  
	LL_USART_SetRxTimeout( HW.If, Timeout ); //установка величины таймаута в битах принимаемого символа
	
	LL_USART_ClearFlag_RTO( HW.If );
	LL_USART_EnableIT_RTO( HW.If );   //разрешение прерывания по RTO
//  LL_USART_EnableIT_RXNE( HW.If );
	LL_USART_EnableIT_PE( HW.If );    //разрешение прерывания по PE
	LL_USART_EnableIT_ERROR( HW.If ); //разрешение прерываний: framing error, overrun error or noise flag
  //---------------------------------------------------------------------------------------------------------
}

void TUsart::dis_eob_detect()
{
  LL_USART_DisableRxTimeout( HW.If ); //отключить функцию определения конца блока по программируемому таймауту
	LL_USART_ClearFlag_RTO( HW.If );
	LL_USART_DisableIT_RTO( HW.If );    //запрет прерывания по RTO
	LL_USART_DisableIT_PE( HW.If );     //запрет прерывания по PE
	LL_USART_DisableIT_ERROR( HW.If );  //запрет прерывания: framing error, overrun error or noise flag
}

void TUsart::set_interrupt()
{
  //Настройка NVIC
  NVIC_SetPriority( HW.IRQ, NVIC_EncodePriority( NVIC_GetPriorityGrouping(), 5, 0 ) ); //обработчик вызывает API функцию RTOS => приоритет д.б. логически ниже или равен, \
                                                                                         но численно больше или равен, установленнму в макросе configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
  NVIC_EnableIRQ( HW.IRQ );
}

const TUsart::TSets &TUsart::get_sets() const
{
  return Sets;
}

void TUsart::set_sets( const TSets &_Sets )
{
  if ( chk_sets( _Sets ) == true ) //если новые настройки совпадают с уже установленными в usart
  {
  
  }
  else //если не совпадают
  {
    dis_if();  
    Sets = _Sets;
    hw_init( 17U );
  }
}

bool TUsart::chk_sets( const TSets &_Sets )
{
  return (
          Sets.BaudRate == _Sets.BaudRate
          &&
          Sets.Parity == _Sets.Parity
          &&
          Sets.Stops == Sets.Stops
         );
}

void TUsart::read_burst( uint8_t *DataBurst, uint8_t Size )        //многобайтное считывание
{                                                                  
  for ( uint8_t Ctr = 0; Ctr < Size ; ++Ctr )                      
  {                                                                
    clr_error_flags();                                             
    do {} while ( LL_USART_IsActiveFlag_RXNE( HW.If ) == false );  //блокировка пока байт не принят
                                                                   
    DataBurst[ Ctr ] = LL_USART_ReceiveData8( HW.If );             //флаг RXNE сбрасывается при считывании байта
  }  
}

void TUsart::write_burst( const uint8_t *DataBurst, uint16_t Size ) //многобайтная запись
{
  clr_error_flags();
  LL_USART_EnableDirectionTx( HW.If );
  
  for ( uint16_t Ctr = 0; Ctr < Size ; ++Ctr )
  { 
    do 
    {
    
    } while ( !LL_USART_IsActiveFlag_TXE( HW.If ) );
    
    LL_USART_TransmitData8( HW.If, DataBurst[ Ctr ] );
  }
  
  do 
  {
  
  }
  while ( !LL_USART_IsActiveFlag_TC( HW.If ) );
  
  LL_USART_DisableDirectionTx( HW.If );
  clr_error_flags();
}

uint8_t TUsart::read_byte()                                        //считывание одного байта
{
  clr_error_flags();
  
  do
  {
    
  } while ( LL_USART_IsActiveFlag_RXNE( HW.If ) == false );
  
  return LL_USART_ReceiveData8( HW.If );
}

void TUsart::enable_rx()
{ 
  LL_USART_EnableDirectionRx( HW.If ); 
}

void TUsart::disable_rx()
{ 
  LL_USART_DisableDirectionRx( HW.If ); 
}
