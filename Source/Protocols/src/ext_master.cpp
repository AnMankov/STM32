#include <algorithm>

#include "ext_master.h"

TExtMaster::TExtMaster(
                       const TUsart_HW &Usart_HW 
                      )
:
TUsart( 
       Usart_HW,
       {
        TBaudRate::_500000,
        TParity::_NONE,
        TStops::_STOPBITS_1
       }       
      ),
ReqPkt( reinterpret_cast< TReqPkt * >( Rx ) )
{

}
          
TExtMaster::~TExtMaster()
{

}

void TExtMaster::parse_pkt()
{ 
  uint8_t Byte = 0U;
  
  enum TRxState
  {
    __PRE  = 0U,
    __DATA = 1U,
    __POST = 2U,
    __END  = 3U,
  };
  
  TRxState RxState = TRxState::__PRE;
  
  uint8_t Ctr = 0U; //счетчик принятых байтов
  
  enable_rx();
  
  do 
  {
    Byte = read_byte();
    
    switch ( RxState )
    {
      case TRxState::__PRE:
           if ( Byte == PREAMBLE )
           {
             RxState = TRxState::__DATA;
             Rx[ Ctr++ ] = Byte;
           }
           
           break;
      case TRxState::__DATA:
           Rx[ Ctr++ ] = Byte;           
           if ( Ctr >= ( sizeof(TReqPkt) - 1 ) )
           {
             RxState = TRxState::__POST;
           }
      
           break;
      case TRxState::__POST:
           if ( Byte == POSTAMBLE )
           {
             Rx[ Ctr ] = Byte;
             RxState   = TRxState::__END;
           }
           else //пакет считается невалидным, ожидаем приема заново
           {
             RxState = TRxState::__PRE;
             Ctr = 0U;
           }
           
           break;
      default:
           
           break;
    }
  } while ( RxState != __END );
  
  disable_rx();
}

TExtMaster::TData TExtMaster::get_data()
{
  return ReqPkt->Data;
}

void TExtMaster::tx_data( const uint8_t *Data, const uint16_t Size, TWrapTxSign WrapTxSign  )
{  
  typedef const uint8_t *( TExtMaster::*TFormHandler )( const uint8_t *Data, uint16_t Qty );
  
  struct TFormPkt
  {
    TFormHandler FormHandler;
    TWrapTxSign  WrapTxSign;
    uint8_t      WrapQty;
  };

  TFormPkt FormPkt[] =
  {   
    { &TExtMaster::no_wrap_tx,     TWrapTxSign::__NO_WRAP_TX    , 0U                   }, //__NO_WRAP_TX
    { &TExtMaster::start_wrap_tx,  TWrapTxSign::__START_WRAP_TX , sizeof( TStartPkt )  }, //__START_WRAP_TX
    { &TExtMaster::finish_wrap_tx, TWrapTxSign::__FINISH_WRAP_TX, sizeof( TFinishPkt ) }, //__FINISH_WRAP_TX
    { &TExtMaster::all_pkt_tx,     TWrapTxSign::__ALL_WRAP_TX   , sizeof( TWrapPkt )   }, //__ALL_WRAP_TX
  };
  
  for ( auto item : FormPkt )
  {
    if ( item.WrapTxSign == WrapTxSign )
    {
      const uint8_t *TxBufAddr = ( this->*item.FormHandler )( Data, Size ); //формирование буфера на отправку
    
      write_burst( TxBufAddr, Size + item.WrapQty );                  //отправка сформированного буфера
      
      break;
    }
  }
}

const uint8_t *TExtMaster::no_wrap_tx( const uint8_t *Data, uint16_t Qty )
{
//  auto Last = Data + Qty;
//  
//  std::copy( Data, Last, Tx );
  return Data;
}

const uint8_t *TExtMaster::start_wrap_tx( const uint8_t *Data, uint16_t Qty )
{
  TStartPkt *pStart = reinterpret_cast< TStartPkt * >( Tx );
  pStart->PreAmble  = PREAMBLE;
  pStart->Cmd       = get_data().Cmd;
  
  return Tx;
}

const uint8_t *TExtMaster::finish_wrap_tx( const uint8_t *Data, uint16_t Qty )
{
  Tx[ 0U ] = POSTAMBLE;
  
  return Tx;
}

const uint8_t *TExtMaster::all_pkt_tx( const uint8_t *Data, uint16_t Qty )
{
  start_wrap_tx( Data, Qty );                                       //Data и Qty в функции не используются
  std::copy( 
            Data, 
            Data + Qty, 
            Tx + sizeof( TStartPkt ) 
           );
  Tx[ sizeof( TStartPkt ) + Qty ] = POSTAMBLE;
  
  return Tx;
}
